// Subscribes to mixer/rx and prints incoming frames as ASCII. Skips frames
// originated by this node (they come back through Mixer's self-decode) and
// idle/zero-padded slots. Designed to pair with mixer_hello_pub.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mixer_comm
{

class HelloSub : public rclcpp::Node
{
public:
  HelloSub()
  : Node("mixer_hello_sub")
  {
    node_id_ = declare_parameter<int>("node_id", 0);
    if (node_id_ < 1 || node_id_ > 255) {
      throw std::runtime_error("node_id must be in [1, 255]");
    }

    sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/rx", rclcpp::QoS(50),
      [this](mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_rx(*msg); });

    RCLCPP_INFO(get_logger(),
      "hello_sub up: node_id=%d listening on mixer/rx", node_id_);
  }

private:
  void on_rx(const mixer_comm_msgs::msg::MixerPayload & msg)
  {
    if (msg.data.empty()) return;
    // Skip the all-zero idle frame the firmware sends when the host has
    // nothing to put in this slot.
    if (std::all_of(msg.data.begin(), msg.data.end(),
                    [](std::uint8_t b) { return b == 0; })) {
      return;
    }
    // Skip our own slot bouncing back via self-decode. The Mixer slot index
    // for sender N is N-1 in the cyclic payload_distribution, so a frame on
    // our own slot is from us. (We don't trust msg.node_id for this -- it's
    // the local dongle id, not the originator.)
    const int our_slot = node_id_ - 1;
    if (msg.slot == our_slot) return;

    // Treat payload as text. Find the first NUL to compute string length;
    // anything past it is zero padding from the publisher.
    const auto & d = msg.data;
    const auto nul = std::find(d.begin(), d.end(), '\0');
    const std::size_t len = static_cast<std::size_t>(nul - d.begin());
    std::string text(reinterpret_cast<const char *>(d.data()), len);

    RCLCPP_INFO(get_logger(),
      "rx (slot %u): \"%s\"", static_cast<unsigned>(msg.slot), text.c_str());
  }

  int node_id_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr sub_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mixer_comm::HelloSub>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_hello_sub: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
