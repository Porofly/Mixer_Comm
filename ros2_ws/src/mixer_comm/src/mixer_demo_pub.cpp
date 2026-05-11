// Demo publisher: emits a synthetic 16-byte payload on its own dongle's
// /mixer/tx topic at a configurable rate. Each frame carries the local node
// id, a monotonically-increasing sequence number, and the current host
// timestamp so a paired subscriber can verify ordering and measure latency.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace mixer_comm
{

// Wire layout of the demo payload (16 bytes total). Keep in sync with
// mixer_demo_sub.cpp.
struct __attribute__((packed)) DemoFrame
{
  std::uint8_t  sender_id;
  std::uint8_t  reserved0;
  std::uint16_t seq;
  std::uint32_t timestamp_us;
  std::uint8_t  reserved1[8];
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

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "demo_pub up: node_id=%d slot=%d rate=%.2f Hz", node_id_, slot_, rate_hz);
  }

private:
  void tick()
  {
    DemoFrame frame{};
    frame.sender_id = static_cast<std::uint8_t>(node_id_);
    frame.seq = seq_++;
    const auto epoch = std::chrono::steady_clock::now().time_since_epoch();
    frame.timestamp_us = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(epoch).count());

    mixer_comm_msgs::msg::MixerPayload msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mixer_demo_pub";
    msg.node_id = static_cast<std::uint16_t>(node_id_);
    msg.slot = static_cast<std::uint8_t>(slot_);
    msg.data.resize(sizeof(DemoFrame));
    std::memcpy(msg.data.data(), &frame, sizeof(DemoFrame));
    pub_->publish(msg);
  }

  int node_id_;
  int slot_;
  std::uint16_t seq_ = 0;
  rclcpp::Publisher<mixer_comm_msgs::msg::MixerPayload>::SharedPtr pub_;
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
