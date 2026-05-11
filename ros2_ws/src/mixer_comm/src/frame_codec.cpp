#include "mixer_comm/frame_codec.hpp"

#include <cstring>
#include <stdexcept>

namespace mixer_comm
{

std::size_t encode_frame(
  std::uint8_t type, std::uint8_t slot,
  const void * payload, std::size_t payload_len,
  std::uint8_t * out, std::size_t out_cap)
{
  const std::size_t size_field = 2 + payload_len;
  const std::size_t total = 2 + size_field;
  if (total > out_cap || size_field > 0xFFFFu) {
    throw std::runtime_error("encode_frame: buffer overflow");
  }
  out[0] = static_cast<std::uint8_t>(size_field & 0xFF);
  out[1] = static_cast<std::uint8_t>((size_field >> 8) & 0xFF);
  out[2] = type;
  out[3] = slot;
  if (payload_len > 0) {
    std::memcpy(out + 4, payload, payload_len);
  }
  return total;
}

void FrameDecoder::feed(const std::uint8_t * data, std::size_t n)
{
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t b = data[i];
    switch (state_) {
      case State::WaitSizeLo:
        expected_ = b;
        state_ = State::WaitSizeHi;
        break;
      case State::WaitSizeHi:
        expected_ |= static_cast<std::uint16_t>(b) << 8;
        if (expected_ < 2 || expected_ > kMaxFrameSize) {
          // Implausible; resync.
          state_ = State::WaitSizeLo;
        } else {
          have_ = 0;
          state_ = State::Body;
        }
        break;
      case State::Body:
        buf_[have_++] = b;
        if (have_ == expected_) {
          const std::uint8_t type = buf_[0];
          const std::uint8_t slot = buf_[1];
          cb_(type, slot, &buf_[2], static_cast<std::size_t>(expected_ - 2));
          state_ = State::WaitSizeLo;
        }
        break;
    }
  }
}

}  // namespace mixer_comm
