// Sends "Hello, Mixer N-K\0..." (16 bytes, zero-padded) to mixer/tx at the
// firmware's auto-detected round rate. N is this node's id, K is a per-node
// monotonic counter. Designed as a human-readable demo of bidirectional RF
// traffic; pair with mixer_hello_sub on each peer to see incoming messages.
//
// Auto-sync logic mirrors mixer_demo_pub: rate_hz <= 0 -> watch mixer/stats
// for a few rounds, derive the round rate, start the timer at ~95% of it.
// Pass rate_hz>0 to override.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "mixer_comm_msgs/msg/mixer_stats.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mixer_comm
{

constexpr std::size_t kPayloadSize = 16;  // matches MX_PAYLOAD_SIZE

class HelloPub : public rclcpp::Node
{
public:
  HelloPub()
  : Node("mixer_hello_pub")
  {
    node_id_ = declare_parameter<int>("node_id", 0);
    slot_ = declare_parameter<int>("slot", 0);
    const double rate_hz = declare_parameter<double>("rate_hz", 0.0);
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

    if (rate_hz > 0.0) {
      start_timer_at(rate_hz, "manual");
    } else {
      RCLCPP_INFO(get_logger(),
        "hello_pub up: node_id=%d slot=%d rate=AUTO (waiting for stats)",
        node_id_, slot_);

      stats_sub_ = create_subscription<mixer_comm_msgs::msg::MixerStats>(
        "mixer/stats", rclcpp::QoS(50),
        [this](mixer_comm_msgs::msg::MixerStats::SharedPtr msg) { on_stats(*msg); });

      const auto to = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(auto_sync_timeout_s_));
      auto_sync_timeout_ = create_wall_timer(to, [this]() {
        auto_sync_timeout_->cancel();
        if (timer_) return;
        RCLCPP_WARN(get_logger(),
          "hello_pub: auto-sync timed out, falling back to %.2f Hz",
          auto_sync_fallback_hz_);
        start_timer_at(auto_sync_fallback_hz_, "fallback");
      });
    }
  }

private:
  void start_timer_at(double rate_hz, const char * source)
  {
    if (timer_) return;
    if (rate_hz <= 0.0) return;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "hello_pub: rate=%.3f Hz (%s)", rate_hz, source);
    if (auto_sync_timeout_) auto_sync_timeout_->cancel();
  }

  void on_stats(const mixer_comm_msgs::msg::MixerStats & msg)
  {
    if (timer_) return;
    const auto now = std::chrono::steady_clock::now();
    stats_samples_.push_back({now, msg.round});
    if (static_cast<int>(stats_samples_.size()) < auto_sync_samples_) return;

    const auto & first = stats_samples_.front();
    const auto & last = stats_samples_.back();
    const auto dt = std::chrono::duration<double>(last.t - first.t).count();
    const std::int64_t dround = static_cast<std::int64_t>(last.round)
                              - static_cast<std::int64_t>(first.round);
    if (dt <= 0.0 || dround <= 0) {
      stats_samples_.erase(stats_samples_.begin());
      return;
    }
    const double round_rate_hz = static_cast<double>(dround) / dt;
    const double safe_rate_hz = round_rate_hz * auto_sync_safety_factor_;
    RCLCPP_INFO(get_logger(),
      "hello_pub: auto-sync measured round rate = %.3f Hz; starting tx at %.3f Hz",
      round_rate_hz, safe_rate_hz);
    start_timer_at(safe_rate_hz, "auto-sync");
    stats_sub_.reset();
    stats_samples_.clear();
  }

  void tick()
  {
    // Format "Hello, Mixer N-K" into a 16-byte zero-padded buffer. snprintf
    // truncates if N-K grows past 16 chars; that just clips the message
    // visibly without overflowing.
    std::array<char, kPayloadSize> buf{};
    std::snprintf(buf.data(), buf.size(), "Hello, Mixer %d-%lu",
                  node_id_, static_cast<unsigned long>(counter_));

    mixer_comm_msgs::msg::MixerPayload msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mixer_hello_pub";
    msg.node_id = static_cast<std::uint16_t>(node_id_);
    msg.slot = static_cast<std::uint8_t>(slot_);
    msg.data.assign(buf.begin(), buf.end());
    pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "tx: %s", buf.data());
    counter_++;
  }

  struct StatsSample
  {
    std::chrono::steady_clock::time_point t;
    std::uint32_t round;
  };

  int node_id_;
  int slot_;
  int auto_sync_samples_ = 4;
  double auto_sync_safety_factor_ = 0.95;
  double auto_sync_timeout_s_ = 10.0;
  double auto_sync_fallback_hz_ = 1.0;
  std::uint64_t counter_ = 0;
  std::vector<StatsSample> stats_samples_;
  rclcpp::Publisher<mixer_comm_msgs::msg::MixerPayload>::SharedPtr pub_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerStats>::SharedPtr stats_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr auto_sync_timeout_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mixer_comm::HelloPub>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_hello_pub: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
