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

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
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
    const double rate_hz = declare_parameter<double>("rate_hz", 1.0);

    if (node_id_ < 1 || node_id_ > 255) {
      throw std::runtime_error("node_id must be in [1, 255]");
    }
    if (slot_ < 0 || slot_ > 255) {
      throw std::runtime_error("slot must be in [0, 255]");
    }
    if (rate_hz <= 0.0) {
      throw std::runtime_error("rate_hz must be positive");
    }

    pub_ = create_publisher<mixer_comm_msgs::msg::MixerPayload>("mixer/tx", 10);

    // Listen to our own dongle's rx so we can echo any frame from a peer.
    // Mixer decodes every slot at every node, so this single subscription
    // sees all peers' frames.
    rx_sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/rx", rclcpp::QoS(50),
      [this](mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_rx(*msg); });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "demo_pub up: node_id=%d slot=%d rate=%.2f Hz", node_id_, slot_, rate_hz);
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
    DemoFrame frame{};
    frame.sender_id = static_cast<std::uint8_t>(node_id_);
    frame.seq = seq_++;
    frame.origin_ts_us = now_us();

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
  }

  struct PendingEcho
  {
    std::uint8_t  sender_id;
    std::uint16_t seq;
    std::uint32_t origin_ts_us;
  };

  int node_id_;
  int slot_;
  std::uint16_t seq_ = 0;
  std::mutex echo_mu_;
  std::optional<PendingEcho> pending_echo_;
  rclcpp::Publisher<mixer_comm_msgs::msg::MixerPayload>::SharedPtr pub_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr rx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
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
