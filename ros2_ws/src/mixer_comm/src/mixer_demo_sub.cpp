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
// This node depends on knowing its own node_id (so it can recognize echoes).
// Pass via parameter.

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
  std::uint16_t seq;
  std::uint32_t origin_ts_us;
  std::uint8_t  echo_sender_id;
  std::uint16_t echo_seq;
  std::uint32_t echo_origin_ts_us;
  std::uint8_t  reserved[2];
};
static_assert(sizeof(DemoFrame) == 16, "DemoFrame must be 16 bytes");

struct PeerStats
{
  std::uint64_t received = 0;
  std::uint64_t lost = 0;
  std::uint64_t duplicate = 0;
  std::uint64_t reordered = 0;
  bool          have_last_seq = false;
  std::uint16_t last_seq = 0;

  // RTT samples (us) accumulated this report window, only populated when this
  // peer is echoing our own frames back.
  std::uint64_t rtt_sum_us = 0;
  std::uint64_t rtt_max_us = 0;
  std::uint64_t rtt_min_us = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t rtt_count = 0;
};

class DemoSub : public rclcpp::Node
{
public:
  DemoSub()
  : Node("mixer_demo_sub")
  {
    node_id_ = declare_parameter<int>("node_id", 0);
    const double report_period = declare_parameter<double>("report_period_s", 2.0);
    if (node_id_ < 1 || node_id_ > 255) {
      throw std::runtime_error("node_id must be in [1, 255]");
    }

    sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/rx", rclcpp::QoS(50),
      [this](mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_rx(*msg); });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(report_period));
    timer_ = create_wall_timer(period, [this]() { report(); });

    RCLCPP_INFO(get_logger(),
      "demo_sub up: node_id=%d listening on mixer/rx, reporting every %.1fs",
      node_id_, report_period);
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
    if (f.sender_id == 0) return;  // idle / zero-padded slot
    if (f.sender_id == node_id_) {
      // Our own frame coming back via Mixer's self-decode. We don't need to
      // count this for loss/RTT, but we do still inspect echo fields below
      // (some firmware variants might let us hear our own echo from a peer
      // -- defensively guard against that by not double-counting).
      return;
    }

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

    // RTT: peer is echoing one of OUR frames back to us.
    if (f.echo_sender_id == static_cast<std::uint8_t>(node_id_)) {
      const std::uint32_t rtt32 = now_us() - f.echo_origin_ts_us;
      const std::uint64_t rtt_us = static_cast<std::uint64_t>(rtt32);
      s.rtt_sum_us += rtt_us;
      s.rtt_count++;
      if (rtt_us > s.rtt_max_us) s.rtt_max_us = rtt_us;
      if (rtt_us < s.rtt_min_us) s.rtt_min_us = rtt_us;
    }
  }

  void report()
  {
    if (peers_.empty()) {
      RCLCPP_INFO(get_logger(), "no peer demo frames received yet");
      return;
    }
    for (auto & [peer, s] : peers_) {
      double rtt_mean = 0.0;
      std::uint64_t rtt_min = 0;
      std::uint64_t rtt_max = s.rtt_max_us;
      if (s.rtt_count > 0) {
        rtt_mean = static_cast<double>(s.rtt_sum_us) / s.rtt_count;
        rtt_min = s.rtt_min_us;
      }
      RCLCPP_INFO(get_logger(),
        "peer=%u rx=%lu lost=%lu dup=%lu reord=%lu "
        "rtt[n=%lu min/mean/max us]=%lu/%.0f/%lu",
        static_cast<unsigned>(peer),
        s.received, s.lost, s.duplicate, s.reordered,
        s.rtt_count, rtt_min, rtt_mean, rtt_max);
      s.rtt_sum_us = 0;
      s.rtt_count = 0;
      s.rtt_max_us = 0;
      s.rtt_min_us = std::numeric_limits<std::uint64_t>::max();
    }
  }

  int node_id_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::uint8_t, PeerStats> peers_;
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
