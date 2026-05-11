#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "mixer_comm/frame_codec.hpp"
#include "mixer_comm/serial_port.hpp"

#include "mixer_comm_msgs/msg/mixer_payload.hpp"
#include "mixer_comm_msgs/msg/mixer_stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace mixer_comm
{

class MixerSerialNode : public rclcpp::Node
{
public:
  MixerSerialNode()
  : Node("mixer_serial_node"),
    decoder_([this](std::uint8_t t, std::uint8_t s, const std::uint8_t * p, std::size_t l) {
      on_frame(t, s, p, l);
    })
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "mixer");

    stats_pub_ = create_publisher<mixer_comm_msgs::msg::MixerStats>("mixer/stats", 10);
    rx_pub_ = create_publisher<mixer_comm_msgs::msg::MixerPayload>("mixer/rx", 50);
    log_pub_ = create_publisher<std_msgs::msg::String>("mixer/log", 10);

    tx_sub_ = create_subscription<mixer_comm_msgs::msg::MixerPayload>(
      "mixer/tx", 10,
      [this](const mixer_comm_msgs::msg::MixerPayload::SharedPtr msg) { on_tx(*msg); });

    try {
      port_.open(device_, baud_);
      RCLCPP_INFO(get_logger(), "Opened %s at %d baud", device_.c_str(), baud_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Serial open failed: %s", e.what());
      throw;
    }

    timer_ = create_wall_timer(20ms, [this]() { poll(); });
  }

private:
  void poll()
  {
    std::uint8_t buf[512];
    const ssize_t n = port_.read(buf, sizeof(buf));
    if (n > 0) {
      decoder_.feed(buf, static_cast<std::size_t>(n));
    }
  }

  void on_frame(std::uint8_t type, std::uint8_t slot, const std::uint8_t * payload, std::size_t len)
  {
    switch (type) {
      case kTypeRxPayload:
        publish_rx(slot, payload, len);
        break;
      case kTypeRoundStats:
        publish_stats(payload, len);
        break;
      case kTypeLog:
        publish_log(payload, len);
        break;
      default:
        RCLCPP_WARN(get_logger(), "unknown frame type 0x%02x len=%zu", type, len);
        break;
    }
  }

  void publish_rx(std::uint8_t slot, const std::uint8_t * payload, std::size_t len)
  {
    mixer_comm_msgs::msg::MixerPayload msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.node_id = current_node_id_;
    msg.slot = slot;
    msg.data.assign(payload, payload + len);
    rx_pub_->publish(msg);
  }

  void publish_stats(const std::uint8_t * payload, std::size_t len)
  {
    if (len != sizeof(RoundStatsWire)) {
      RCLCPP_WARN(get_logger(), "round stats: bad size %zu", len);
      return;
    }
    RoundStatsWire w;
    std::memcpy(&w, payload, sizeof(w));

    current_node_id_ = static_cast<std::uint16_t>(w.node_id);

    mixer_comm_msgs::msg::MixerStats msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.node_id = current_node_id_;
    msg.round = w.round;
    msg.rank = w.rank;
    msg.decoded = w.decoded;
    msg.not_decoded = w.not_decoded;
    msg.weak = w.weak;
    msg.wrong = w.wrong;
    stats_pub_->publish(msg);
  }

  void publish_log(const std::uint8_t * payload, std::size_t len)
  {
    std_msgs::msg::String msg;
    msg.data.assign(reinterpret_cast<const char *>(payload), len);
    log_pub_->publish(msg);
    if (!msg.data.empty()) {
      RCLCPP_INFO(get_logger(), "dongle log: %s",
                  (msg.data.back() == '\n' ? msg.data.substr(0, msg.data.size() - 1) : msg.data).c_str());
    }
  }

  void on_tx(const mixer_comm_msgs::msg::MixerPayload & msg)
  {
    constexpr std::size_t kMxPayloadSize = 16;  // must match firmware MX_PAYLOAD_SIZE
    std::uint8_t payload[kMxPayloadSize] = {};
    const std::size_t n = std::min<std::size_t>(msg.data.size(), kMxPayloadSize);
    std::memcpy(payload, msg.data.data(), n);
    // Trailing bytes already zero-initialised.

    std::uint8_t frame[4 + kMxPayloadSize];
    const std::size_t total = encode_frame(
      kTypeTxPayload, msg.slot, payload, kMxPayloadSize, frame, sizeof(frame));

    try {
      port_.write_all(frame, total);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "serial write failed: %s", e.what());
    }
  }

  std::string device_;
  int baud_;
  std::string frame_id_;

  SerialPort port_;
  FrameDecoder decoder_;
  std::uint16_t current_node_id_ = 0;

  rclcpp::Publisher<mixer_comm_msgs::msg::MixerStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<mixer_comm_msgs::msg::MixerPayload>::SharedPtr rx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::Subscription<mixer_comm_msgs::msg::MixerPayload>::SharedPtr tx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mixer_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mixer_comm::MixerSerialNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mixer_serial_node terminated: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
