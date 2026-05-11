// Demo subscriber: subscribes to one or more /mixer/node<id>/mixer/rx topics
// (passed as the `listen_topics` parameter, a string array), interprets each
// MixerPayload as a DemoFrame, and reports sequence loss + latency per sender
// every report_period_s seconds.
//
// Expected layout of `data` matches mixer_demo_pub.cpp.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
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
  std::uint8_t  reserved0;
  std::uint16_t seq;
  std::uint32_t timestamp_us;
  std::uint8_t  reserved1[8];
};
static_assert(sizeof(DemoFrame) == 16, "DemoFrame must be 16 bytes");

struct SenderStats
{
  std::uint64_t received = 0;        // total demo-frames seen
  std::uint64_t lost = 0;             // gaps detected in seq
  std::uint64_t duplicate = 0;        // same seq seen twice
  std::uint64_t reordered = 0;        // seq < last_seq (not wraparound)
  bool          have_last_seq = false;
  std::uint16_t last_seq = 0;
  // latency (microseconds), reset every report period
  std::uint64_t lat_sum_us = 0;
  std::uint64_t lat_max_us = 0;
  std::uint64_t lat_count = 0;
  // monotonic local timebase, must match publisher's clock choice
  std::uint64_t lat_min_us = std::numeric_limits<std::uint64_t>::max();
};

class DemoSub : public rclcpp::Node
{
public:
  DemoSub()
  : Node("mixer_demo_sub")
  {
    listen_topics_ = declare_parameter<std::vector<std::string>>(
      "listen_topics", std::vector<std::string>{});
    const double report_period = declare_parameter<double>("report_period_s", 2.0);

    if (listen_topics_.empty()) {
      throw std::runtime_error(
        "listen_topics is empty -- pass at least one /.../mixer/rx topic");
    }

    for (const auto & topic : listen_topics_) {
      auto sub = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
        topic, rclcpp::QoS(50),
        [this, topic](const mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) {
          on_rx(topic, *msg);
        });
      subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "listening on %s", topic.c_str());
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(report_period));
    timer_ = create_wall_timer(period, [this]() { report(); });
  }

private:
  void on_rx(const std::string & /*topic*/, const mixer_comm_msgs::msg::MixerPayload & msg)
  {
    if (msg.data.size() != sizeof(DemoFrame)) {
      return;  // not a demo payload (e.g. zero-padded idle frame from firmware)
    }
    DemoFrame f{};
    std::memcpy(&f, msg.data.data(), sizeof(f));

    // The all-zero idle frame would have sender_id=0; skip it.
    if (f.sender_id == 0) return;

    auto & s = senders_[f.sender_id];
    s.received++;

    if (s.have_last_seq) {
      const std::uint16_t expected = static_cast<std::uint16_t>(s.last_seq + 1);
      if (f.seq == s.last_seq) {
        s.duplicate++;
      } else if (f.seq == expected) {
        // perfect
      } else {
        // gap or reorder (16-bit wraparound aware via signed diff)
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

    // Latency: difference between current local clock and the timestamp the
    // sender embedded in the frame. Both clocks here are steady_clock on the
    // *same* host when the sender and receiver run on the same machine.
    // Across hosts this becomes a clock-skew measurement -- still useful as a
    // smoke signal, but not a true latency. Document this.
    const auto epoch = std::chrono::steady_clock::now().time_since_epoch();
    const auto now_us = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(epoch).count());
    const std::uint32_t lat_us32 = now_us - f.timestamp_us;  // 32-bit wraparound OK for <1 hour
    const std::uint64_t lat_us = static_cast<std::uint64_t>(lat_us32);

    s.lat_sum_us += lat_us;
    s.lat_count++;
    if (lat_us > s.lat_max_us) s.lat_max_us = lat_us;
    if (lat_us < s.lat_min_us) s.lat_min_us = lat_us;
  }

  void report()
  {
    if (senders_.empty()) {
      RCLCPP_INFO(get_logger(), "no demo frames received yet");
      return;
    }
    for (auto & [sender, s] : senders_) {
      double mean_us = 0.0;
      std::uint64_t min_us = 0;
      std::uint64_t max_us = s.lat_max_us;
      if (s.lat_count > 0) {
        mean_us = static_cast<double>(s.lat_sum_us) / s.lat_count;
        min_us = s.lat_min_us;
      }
      RCLCPP_INFO(get_logger(),
        "sender=%u rx=%lu lost=%lu dup=%lu reord=%lu lat[min/mean/max us]=%lu/%.0f/%lu",
        static_cast<unsigned>(sender),
        s.received, s.lost, s.duplicate, s.reordered,
        min_us, mean_us, max_us);
      // reset latency window for the next period
      s.lat_sum_us = 0;
      s.lat_count = 0;
      s.lat_max_us = 0;
      s.lat_min_us = std::numeric_limits<std::uint64_t>::max();
    }
  }

  std::vector<std::string> listen_topics_;
  std::vector<rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr> subs_;
  std::map<std::uint8_t, SenderStats> senders_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mixer_comm::DemoSub>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_demo_sub: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
