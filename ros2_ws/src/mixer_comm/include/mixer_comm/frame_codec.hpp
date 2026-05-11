#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mixer_comm
{

// Wire format (matches firmware/host_proto.h):
//   [size_lo][size_hi][type][slot][payload(size-2 bytes)]
// size_le16 counts type+slot+payload, i.e. size = 2 + payload_len.

constexpr std::uint8_t kTypeTxPayload  = 0x10;
constexpr std::uint8_t kTypeRxPayload  = 0x20;
constexpr std::uint8_t kTypeRoundStats = 0x30;
constexpr std::uint8_t kTypeLog        = 0x40;

constexpr std::size_t kMaxFrameSize = 256;  // hard cap, must exceed firmware HP_RX_MAX

struct __attribute__((packed)) RoundStatsWire
{
  std::uint32_t round;
  std::uint32_t rank;
  std::uint32_t decoded;
  std::uint32_t not_decoded;
  std::uint32_t weak;
  std::uint32_t wrong;
  std::uint32_t node_id;
};

// Serialise a frame into a buffer. Returns the total number of bytes written.
std::size_t encode_frame(
  std::uint8_t type, std::uint8_t slot,
  const void * payload, std::size_t payload_len,
  std::uint8_t * out, std::size_t out_cap);

// Streaming decoder for the dongle->host byte stream. Feed it bytes, it
// invokes the callback once per complete frame. Implementation is bounded
// (max kMaxFrameSize); implausible size fields trigger one-byte resync.
class FrameDecoder
{
public:
  using FrameCallback = std::function<
    void(std::uint8_t type, std::uint8_t slot, const std::uint8_t * payload, std::size_t payload_len)>;

  explicit FrameDecoder(FrameCallback cb) : cb_(std::move(cb)) {}

  void feed(const std::uint8_t * data, std::size_t n);

private:
  enum class State { WaitSizeLo, WaitSizeHi, Body };

  FrameCallback cb_;
  State state_ = State::WaitSizeLo;
  std::uint16_t expected_ = 0;
  std::uint16_t have_ = 0;
  std::uint8_t buf_[kMaxFrameSize];
};

}  // namespace mixer_comm
