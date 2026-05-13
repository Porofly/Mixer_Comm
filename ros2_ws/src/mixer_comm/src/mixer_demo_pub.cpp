// Demo publisher + RTT echoer.
//
// Each node periodically transmits a 16-byte DemoFrame on its dongle's
// /mixer/tx topic. The frame carries:
//   - this node's id + seq + origin timestamp (us, local steady_clock)
//   - an echo slot: when we recently received a frame from another node, the
//     next outgoing frame mirrors that (sender_id, seq, origin_ts) back.
//
// The original sender then sees its own (sender_id, seq, origin_ts) bounce
// back via /mixer/rx and computes RTT = now - origin_ts. Because origin_ts
// is read from the same steady_clock the receiving comparison uses, no clock
// sync between hosts is needed -- each node measures RTT against itself.
//
// Subscriber-side accounting (loss, dup, RTT histogram) lives in
// mixer_demo_sub.cpp; this file only generates and echoes traffic.
//
// Auto-rate sync: when rate_hz <= 0, the pub holds off transmitting and
// observes the local mixer/stats topic instead. Each MixerStats message marks
// the end of one Mixer round, so the steady-state round period can be measured
// directly. After auto_sync_samples rounds, the timer is started at
// (round_rate_hz * auto_sync_safety_factor) -- a small safety margin keeps the
// host below the dongle's per-round TX queue ceiling. If stats never arrives
// (USB stalled, firmware not booted), the auto_sync_timeout_s fallback kicks
// in with rate=auto_sync_fallback_hz and a WARN. Pass rate_hz>0 to skip
// auto-sync entirely.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "mixer_comm_msgs/msg/mixer_stats.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace mixer_comm
{

// 16-byte wire layout. Mirrored in mixer_demo_sub.cpp.
struct __attribute__((packed)) DemoFrame
{
  std::uint8_t  sender_id;          // who originated this frame
  std::uint16_t seq;                // sender's monotonic seq
  std::uint32_t origin_ts_us;       // sender's steady_clock us (32-bit)
  std::uint8_t  echo_sender_id;     // 0 if no echo, else the id we are echoing
  std::uint16_t echo_seq;           // echoed seq
  std::uint32_t echo_origin_ts_us;  // echoed origin_ts_us
  std::uint8_t  reserved[2];
};
static_assert(sizeof(DemoFrame) == 16, "DemoFrame must fit MX_PAYLOAD_SIZE");

class DemoPub : public rclcpp::Node
{
public:
  DemoPub()
  : Node("mixer_demo_pub")
  {
    node_id_ = declare_parameter<int>("node_id", 0);
    slot_ = declare_parameter<int>("slot", 0);
    // rate_hz semantics:
    //   > 0  -> use this rate verbatim (original behavior)
    //   <= 0 -> auto-sync from /mixer/stats round period
    const double rate_hz = declare_parameter<double>("rate_hz", 0.0);
    tx_count_ = declare_parameter<int>("tx_count", 0);
    auto_sync_samples_ = declare_parameter<int>("auto_sync_samples", 4);
    auto_sync_safety_factor_ = declare_parameter<double>("auto_sync_safety_factor", 0.95);
    auto_sync_timeout_s_ = declare_parameter<double>("auto_sync_timeout_s", 10.0);
    auto_sync_fallback_hz_ = declare_parameter<double>("auto_sync_fallback_hz", 1.0);

    if (node_id_ < 1 || node_id_ > 255) {
      throw std::runtime_error("node_id must be in [1, 255]");
    }
    if (slot_ < 0 || slot_ > 255) {
      throw std::runtime_error("slot must be in [0, 255]");
    }
    if (tx_count_ < 0) {
      throw std::runtime_error("tx_count must be >= 0 (0 = unlimited)");
    }
    if (auto_sync_samples_ < 2) {
      throw std::runtime_error("auto_sync_samples must be >= 2");
    }
    if (auto_sync_safety_factor_ <= 0.0 || auto_sync_safety_factor_ > 1.0) {
      throw std::runtime_error("auto_sync_safety_factor must be in (0, 1]");
    }
    if (auto_sync_fallback_hz_ <= 0.0) {
      throw std::runtime_error("auto_sync_fallback_hz must be positive");
    }

    pub_ = create_publisher<mixer_comm_msgs::msg::MixerPayload>("mixer/tx", 10);

    rx_sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/rx", rclcpp::QoS(50),
      [this](mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_rx(*msg); });

    if (rate_hz > 0.0) {
      start_timer_at(rate_hz, "manual");
    } else {
      RCLCPP_INFO(get_logger(),
        "demo_pub up: node_id=%d slot=%d rate=AUTO (waiting for stats, "
        "samples=%d safety=%.2f timeout=%.1fs fallback=%.2fHz) tx_count=%d (%s)",
        node_id_, slot_, auto_sync_samples_, auto_sync_safety_factor_,
        auto_sync_timeout_s_, auto_sync_fallback_hz_, tx_count_,
        tx_count_ == 0 ? "unlimited" : "bounded");

      stats_sub_ = create_subscription<mixer_comm_msgs::msg::MixerStats>(
        "mixer/stats", rclcpp::QoS(50),
        [this](mixer_comm_msgs::msg::MixerStats::SharedPtr msg) { on_stats(*msg); });

      const auto to = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(auto_sync_timeout_s_));
      auto_sync_timeout_ = create_wall_timer(to, [this]() {
        auto_sync_timeout_->cancel();
        if (timer_) return;
        RCLCPP_WARN(get_logger(),
          "demo_pub: auto-sync timed out after %.1fs (no usable stats), "
          "falling back to %.2f Hz", auto_sync_timeout_s_, auto_sync_fallback_hz_);
        start_timer_at(auto_sync_fallback_hz_, "fallback");
      });
    }
  }

private:
  void start_timer_at(double rate_hz, const char * source)
  {
    if (timer_) return;
    if (rate_hz <= 0.0) {
      RCLCPP_ERROR(get_logger(), "refusing to start timer with non-positive rate %.4f", rate_hz);
      return;
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "demo_pub: rate=%.3f Hz (%s), tx_count=%d (%s)",
      rate_hz, source, tx_count_, tx_count_ == 0 ? "unlimited" : "bounded");
    if (auto_sync_timeout_) auto_sync_timeout_->cancel();
  }

  // Stats observer: collect (steady_clock now, round counter) pairs from each
  // MixerStats message. Once we have N samples covering at least one full
  // measurement window, derive the round rate and start the tx timer at a
  // safety-discounted multiple of it.
  void on_stats(const mixer_comm_msgs::msg::MixerStats & msg)
  {
    if (timer_) return;
    const auto now = std::chrono::steady_clock::now();
    stats_samples_.push_back({now, msg.round});
    if (static_cast<int>(stats_samples_.size()) < auto_sync_samples_) return;

    // Derive rate from first/last sample using round delta in case stats
    // messages were dropped mid-window (the 'round' field is monotonic).
    const auto & first = stats_samples_.front();
    const auto & last = stats_samples_.back();
    const auto dt = std::chrono::duration<double>(last.t - first.t).count();
    const std::int64_t dround = static_cast<std::int64_t>(last.round) - static_cast<std::int64_t>(first.round);
    if (dt <= 0.0 || dround <= 0) {
      // Bad data; drop oldest and keep collecting.
      stats_samples_.erase(stats_samples_.begin());
      return;
    }
    const double round_rate_hz = static_cast<double>(dround) / dt;
    const double safe_rate_hz = round_rate_hz * auto_sync_safety_factor_;
    RCLCPP_INFO(get_logger(),
      "demo_pub: auto-sync measured round rate = %.3f Hz over %d samples (%.2fs); "
      "starting tx at %.3f Hz (= %.0f%% of round)",
      round_rate_hz, auto_sync_samples_, dt, safe_rate_hz,
      auto_sync_safety_factor_ * 100.0);
    start_timer_at(safe_rate_hz, "auto-sync");
    stats_sub_.reset();
    stats_samples_.clear();
  }

  static std::uint32_t now_us()
  {
    const auto epoch = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(epoch).count());
  }

  void on_rx(const mixer_comm_msgs::msg::MixerPayload & msg)
  {
    if (msg.data.size() != sizeof(DemoFrame)) return;
    DemoFrame f{};
    std::memcpy(&f, msg.data.data(), sizeof(f));
    if (f.sender_id == 0 || f.sender_id == node_id_) return;  // idle / self

    // Stash the most-recent peer frame; the next tick() will echo it. If
    // multiple peers' frames arrive between ticks we only echo the latest.
    // For 1 Hz traffic across a handful of nodes this is fine; if a higher
    // rate is needed, extend to a per-peer queue.
    std::lock_guard<std::mutex> lk(echo_mu_);
    pending_echo_ = PendingEcho{f.sender_id, f.seq, f.origin_ts_us};
  }

  void tick()
  {
    const bool originate = (tx_count_ == 0)
      || (sent_ < static_cast<std::uint64_t>(tx_count_));

    DemoFrame frame{};
    frame.sender_id = static_cast<std::uint8_t>(node_id_);
    if (originate) {
      // Fresh frame: bump seq + take a new origin timestamp.
      frame.seq = seq_++;
      frame.origin_ts_us = now_us();
      last_seq_ = frame.seq;
      last_origin_ts_us_ = frame.origin_ts_us;
    } else {
      // Keep-alive: do NOT bump seq or take a new origin timestamp. The
      // subscriber will see this as a duplicate of last_seq (which it
      // counts but ignores for loss accounting). The point is to keep a
      // slot in every Mixer round so the echo field below still reaches
      // the peer -- otherwise the peer can never close out its own RTT
      // measurements after we stop originating.
      frame.seq = last_seq_;
      frame.origin_ts_us = last_origin_ts_us_;
    }

    std::optional<PendingEcho> echo;
    {
      std::lock_guard<std::mutex> lk(echo_mu_);
      echo = pending_echo_;
      pending_echo_.reset();
    }
    if (echo) {
      frame.echo_sender_id = echo->sender_id;
      frame.echo_seq = echo->seq;
      frame.echo_origin_ts_us = echo->origin_ts_us;
    }

    mixer_comm_msgs::msg::MixerPayload msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mixer_demo_pub";
    msg.node_id = static_cast<std::uint16_t>(node_id_);
    msg.slot = static_cast<std::uint8_t>(slot_);
    msg.data.resize(sizeof(DemoFrame));
    std::memcpy(msg.data.data(), &frame, sizeof(DemoFrame));
    pub_->publish(msg);

    if (originate) {
      sent_++;
      if (tx_count_ > 0 && sent_ >= static_cast<std::uint64_t>(tx_count_)) {
        RCLCPP_INFO(get_logger(),
          "demo_pub: sent %lu / %d frames, switching to echo-only keep-alive",
          sent_, tx_count_);
      }
    }
  }

  struct PendingEcho
  {
    std::uint8_t  sender_id;
    std::uint16_t seq;
    std::uint32_t origin_ts_us;
  };

  struct StatsSample
  {
    std::chrono::steady_clock::time_point t;
    std::uint32_t round;
  };

  int node_id_;
  int slot_;
  int tx_count_ = 0;
  int auto_sync_samples_ = 4;
  double auto_sync_safety_factor_ = 0.95;
  double auto_sync_timeout_s_ = 10.0;
  double auto_sync_fallback_hz_ = 1.0;
  std::uint64_t sent_ = 0;
  std::uint16_t seq_ = 0;
  std::uint16_t last_seq_ = 0;
  std::uint32_t last_origin_ts_us_ = 0;
  std::mutex echo_mu_;
  std::optional<PendingEcho> pending_echo_;
  std::vector<StatsSample> stats_samples_;
  rclcpp::Publisher<mixer_comm_msgs::msg::MixerPayload>::SharedPtr pub_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr rx_sub_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerStats>::SharedPtr stats_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr auto_sync_timeout_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mixer_comm::DemoPub>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_demo_pub: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
