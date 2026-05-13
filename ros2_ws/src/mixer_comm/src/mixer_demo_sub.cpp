// Demo subscriber: subscribes to this node's own /mixer/rx, splits incoming
// DemoFrames by sender_id, and reports per-peer statistics.
//
// For each peer it tracks:
//   - received / lost / dup / reorder of the peer's own (sender_id, seq)
// For frames whose echo_sender_id == our node_id (a peer is echoing our
// earlier frame back), it computes RTT = now_us - echo_origin_ts_us.
//
// Because origin_ts_us was read from this node's steady_clock and we compare
// against the same clock, no inter-host time sync is required. Single-host
// minimum RTT establishes the dongle's loopback floor; cross-host RTT is the
// real RF round-trip (plus 2 dongle hops).
//
// Bounded mode: when expected_rx_count > 0, the node arms a one-shot drain
// timer the moment any peer reaches that many received frames. After
// drain_grace_s seconds (during which late echoes are still folded in), it
// prints a final cumulative report, optionally writes report_path as JSON,
// and shuts the executor down -- producing a deterministic test run.
//
// This node depends on knowing its own node_id (so it can recognize echoes).
// Pass via parameter.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace mixer_comm
{

struct __attribute__((packed)) DemoFrame
{
  std::uint8_t  sender_id;
  std::uint16_t seq;
  std::uint32_t origin_ts_us;
  std::uint8_t  echo_sender_id;
  std::uint16_t echo_seq;
  std::uint32_t echo_origin_ts_us;
  std::uint8_t  reserved[2];
};
static_assert(sizeof(DemoFrame) == 16, "DemoFrame must be 16 bytes");

// Two RTT accumulators per peer: a windowed one that resets every periodic
// report, and a cumulative one that survives until shutdown for the final
// summary.
struct RttAccum
{
  std::uint64_t sum_us = 0;
  std::uint64_t max_us = 0;
  std::uint64_t min_us = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t count = 0;

  void add(std::uint64_t v)
  {
    sum_us += v;
    count++;
    if (v > max_us) max_us = v;
    if (v < min_us) min_us = v;
  }
  void reset() { *this = RttAccum{}; }
};

struct PeerStats
{
  std::uint64_t received = 0;
  std::uint64_t lost = 0;
  std::uint64_t duplicate = 0;
  std::uint64_t reordered = 0;
  bool          have_last_seq = false;
  std::uint16_t last_seq = 0;

  RttAccum rtt_window;  // resets every periodic report
  RttAccum rtt_total;   // accumulates over the whole run
};

class DemoSub : public rclcpp::Node
{
public:
  DemoSub()
  : Node("mixer_demo_sub")
  {
    node_id_ = declare_parameter<int>("node_id", 0);
    const double report_period = declare_parameter<double>("report_period_s", 2.0);
    expected_rx_count_ = declare_parameter<int>("expected_rx_count", 0);
    drain_grace_s_ = declare_parameter<double>("drain_grace_s", 3.0);
    report_path_ = declare_parameter<std::string>("report_path", "");
    // Hard deadline. 0 = disabled. When > 0, the run is forcibly finalized
    // after this many seconds even if expected_rx_count was never reached
    // (e.g. RF losses kept us short of N). Without it, packet loss makes
    // the bounded run hang forever.
    timeout_s_ = declare_parameter<double>("timeout_s", 0.0);

    if (node_id_ < 1 || node_id_ > 255) {
      throw std::runtime_error("node_id must be in [1, 255]");
    }
    if (expected_rx_count_ < 0) {
      throw std::runtime_error("expected_rx_count must be >= 0 (0 = unlimited)");
    }
    if (drain_grace_s_ < 0.0) {
      throw std::runtime_error("drain_grace_s must be >= 0");
    }
    if (timeout_s_ < 0.0) {
      throw std::runtime_error("timeout_s must be >= 0 (0 = disabled)");
    }

    sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/rx", rclcpp::QoS(50),
      [this](mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_rx(*msg); });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(report_period));
    report_timer_ = create_wall_timer(period, [this]() { report(); });

    if (timeout_s_ > 0.0) {
      const auto to = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_s_));
      timeout_timer_ = create_wall_timer(to, [this]() {
        if (drain_armed_) return;  // already on the planned exit path
        timeout_timer_->cancel();
        RCLCPP_WARN(get_logger(),
          "demo_sub: timeout_s=%.1f reached without hitting expected_rx_count, "
          "finalizing now", timeout_s_);
        finalize_and_shutdown();
      });
    }

    start_steady_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
      "demo_sub up: node_id=%d listening on mixer/rx, period=%.1fs "
      "expected_rx=%d drain=%.1fs timeout=%.1fs report=%s",
      node_id_, report_period, expected_rx_count_, drain_grace_s_, timeout_s_,
      report_path_.empty() ? "(disabled)" : report_path_.c_str());
  }

private:
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
    if (f.sender_id == 0) return;
    if (f.sender_id == node_id_) return;

    auto & s = peers_[f.sender_id];
    s.received++;

    if (s.have_last_seq) {
      const std::uint16_t expected = static_cast<std::uint16_t>(s.last_seq + 1);
      if (f.seq == s.last_seq) {
        s.duplicate++;
      } else if (f.seq == expected) {
        // perfect
      } else {
        const std::int16_t diff = static_cast<std::int16_t>(f.seq - expected);
        if (diff > 0) {
          s.lost += static_cast<std::uint64_t>(diff);
        } else {
          s.reordered++;
        }
      }
    }
    s.have_last_seq = true;
    s.last_seq = f.seq;

    if (f.echo_sender_id == static_cast<std::uint8_t>(node_id_)) {
      const std::uint32_t rtt32 = now_us() - f.echo_origin_ts_us;
      const std::uint64_t rtt_us = static_cast<std::uint64_t>(rtt32);
      s.rtt_window.add(rtt_us);
      s.rtt_total.add(rtt_us);
    }

    maybe_arm_drain();
  }

  // Once any peer has reached the expected rx count, start the drain grace
  // timer. Late echoes are still folded into rtt_total during this window.
  void maybe_arm_drain()
  {
    if (drain_armed_ || expected_rx_count_ <= 0) return;
    bool reached = false;
    for (const auto & [_, s] : peers_) {
      if (s.received >= static_cast<std::uint64_t>(expected_rx_count_)) {
        reached = true;
        break;
      }
    }
    if (!reached) return;

    drain_armed_ = true;
    RCLCPP_INFO(get_logger(),
      "demo_sub: hit expected_rx_count=%d, draining for %.1fs before final report",
      expected_rx_count_, drain_grace_s_);

    const auto grace = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(drain_grace_s_));
    drain_timer_ = create_wall_timer(grace, [this]() {
      drain_timer_->cancel();
      finalize_and_shutdown();
    });
  }

  void report()
  {
    if (peers_.empty()) {
      RCLCPP_INFO(get_logger(), "no peer demo frames received yet");
      return;
    }
    for (auto & [peer, s] : peers_) {
      const auto & w = s.rtt_window;
      double rtt_mean = 0.0;
      std::uint64_t rtt_min = 0;
      std::uint64_t rtt_max = w.max_us;
      if (w.count > 0) {
        rtt_mean = static_cast<double>(w.sum_us) / w.count;
        rtt_min = w.min_us;
      }
      RCLCPP_INFO(get_logger(),
        "peer=%u rx=%lu lost=%lu dup=%lu reord=%lu "
        "rtt[n=%lu min/mean/max us]=%lu/%.0f/%lu",
        static_cast<unsigned>(peer),
        s.received, s.lost, s.duplicate, s.reordered,
        w.count, rtt_min, rtt_mean, rtt_max);
      s.rtt_window.reset();
    }
  }

  // Hand-rolled JSON: we have a tiny, fixed schema and don't want to drag in
  // nlohmann/json or rapidjson just for this.
  std::string build_json(double duration_s) const
  {
    std::ostringstream o;
    o << "{\n";
    o << "  \"node_id\": " << node_id_ << ",\n";
    o << "  \"duration_s\": " << duration_s << ",\n";
    o << "  \"expected_rx_count\": " << expected_rx_count_ << ",\n";
    o << "  \"peers\": {\n";
    bool first = true;
    for (const auto & [peer, s] : peers_) {
      const auto & t = s.rtt_total;
      const std::uint64_t rtt_min = t.count > 0 ? t.min_us : 0;
      const double rtt_mean = t.count > 0
        ? static_cast<double>(t.sum_us) / t.count : 0.0;
      const std::uint64_t rtt_max = t.max_us;
      if (!first) o << ",\n";
      first = false;
      o << "    \"" << static_cast<unsigned>(peer) << "\": {";
      o << " \"received\": " << s.received;
      o << ", \"lost\": " << s.lost;
      o << ", \"duplicate\": " << s.duplicate;
      o << ", \"reordered\": " << s.reordered;
      o << ", \"rtt_count\": " << t.count;
      o << ", \"rtt_min_us\": " << rtt_min;
      o << ", \"rtt_mean_us\": " << rtt_mean;
      o << ", \"rtt_max_us\": " << rtt_max;
      o << " }";
    }
    o << "\n  }\n";
    o << "}\n";
    return o.str();
  }

  // Idempotent: safe to call from both the drain timer (planned exit) and
  // an on_shutdown handler (Ctrl-C). The flag prevents the final report from
  // being printed/written twice.
public:
  void finalize(bool trigger_shutdown)
  {
    if (finalized_.exchange(true)) return;

    const auto duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_steady_).count();

    RCLCPP_INFO(get_logger(), "===== FINAL REPORT (cumulative, duration=%.2fs) =====",
                duration);
    if (peers_.empty()) {
      RCLCPP_WARN(get_logger(), "no peer frames received during the run");
    }
    for (const auto & [peer, s] : peers_) {
      const auto & t = s.rtt_total;
      double rtt_mean = 0.0;
      std::uint64_t rtt_min = 0;
      std::uint64_t rtt_max = t.max_us;
      if (t.count > 0) {
        rtt_mean = static_cast<double>(t.sum_us) / t.count;
        rtt_min = t.min_us;
      }
      RCLCPP_INFO(get_logger(),
        "peer=%u TOTAL rx=%lu lost=%lu dup=%lu reord=%lu "
        "rtt[n=%lu min/mean/max us]=%lu/%.0f/%lu",
        static_cast<unsigned>(peer),
        s.received, s.lost, s.duplicate, s.reordered,
        t.count, rtt_min, rtt_mean, rtt_max);
    }

    if (!report_path_.empty()) {
      std::ofstream f(report_path_);
      if (!f) {
        RCLCPP_ERROR(get_logger(),
          "failed to open report_path '%s' for writing", report_path_.c_str());
      } else {
        f << build_json(duration);
        RCLCPP_INFO(get_logger(), "wrote JSON report to %s", report_path_.c_str());
      }
    }

    if (trigger_shutdown) {
      rclcpp::shutdown();
    }
  }

private:
  void finalize_and_shutdown() { finalize(/*trigger_shutdown=*/true); }

  int node_id_;
  int expected_rx_count_ = 0;
  double drain_grace_s_ = 3.0;
  double timeout_s_ = 0.0;
  std::string report_path_;
  bool drain_armed_ = false;
  std::atomic<bool> finalized_{false};
  std::chrono::steady_clock::time_point start_steady_;

  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::map<std::uint8_t, PeerStats> peers_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mixer_comm::DemoSub>();
    // Make sure Ctrl-C still produces a final summary + JSON before exit.
    // on_shutdown fires once during rclcpp::shutdown(); finalize() is
    // idempotent so the planned (drain_timer) path also works.
    rclcpp::on_shutdown([node]() { node->finalize(/*trigger_shutdown=*/false); });
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_demo_sub: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
