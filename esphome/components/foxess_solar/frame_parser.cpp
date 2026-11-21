#include "frame_parser.h"

#include <algorithm>
#include <cstring>

namespace esphome {
namespace foxess_solar {
namespace {

using Result = FoxessFrameParser::Result;

constexpr std::array<uint8_t, 3> TELEMETRY_HEADER{0x7E, 0x7E, 0x02};
constexpr std::array<uint8_t, 2> FRAME_FOOTER{0xE7, 0xE7};
constexpr std::size_t TYPE_OFFSET = 2;
constexpr std::size_t PAYLOAD_LENGTH_OFFSET = 7;
constexpr std::size_t HEADER_SIZE = PAYLOAD_LENGTH_OFFSET + 2;
constexpr std::size_t CHECKSUM_SIZE = 2;
constexpr uint32_t FRAME_OVERHEAD = HEADER_SIZE + CHECKSUM_SIZE + FRAME_FOOTER.size();
constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 100;

// Recovery can leave a partial frame behind a validated frame in the buffer.
static_assert(FoxessFrameParser::MAX_FRAME_SIZE < 2 * FoxessFrameParser::ORIGINAL_LAYOUT_FRAME_SIZE_163,
              "The parser buffers at most one complete frame");

std::size_t supported_frame_size(const uint8_t *header) {
  // Widen the advertised payload length before adding the framing overhead.
  const uint32_t payload_size =
      (static_cast<uint32_t>(header[PAYLOAD_LENGTH_OFFSET]) << 8) | header[PAYLOAD_LENGTH_OFFSET + 1];
  const uint32_t frame_size = payload_size + FRAME_OVERHEAD;
  switch (frame_size) {
    case FoxessFrameParser::ORIGINAL_LAYOUT_FRAME_SIZE_163:
    case FoxessFrameParser::ORIGINAL_LAYOUT_FRAME_SIZE_165:
    case FoxessFrameParser::ORIGINAL_LAYOUT_FRAME_SIZE_245:
    case FoxessFrameParser::SHIFTED_LAYOUT_FRAME_SIZE_251:
      return frame_size;
    default:
      return 0;
  }
}

// Compute the reflected CRC-16/MODBUS checksum used on the wire.
uint16_t frame_checksum(const uint8_t *data, std::size_t size) {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint16_t feedback = (crc & 1U) != 0 ? 0xA001U : 0;
      crc = (crc >> 1) ^ feedback;
    }
  }
  return crc;
}

// The caller has already checked the type and collected the advertised number of bytes.
Result validate_frame(const uint8_t *data, std::size_t size) {
  // Check the footer first so a framing error can be reported without calculating the CRC.
  const std::size_t footer_offset = size - FRAME_FOOTER.size();
  if (!std::equal(FRAME_FOOTER.begin(), FRAME_FOOTER.end(), data + footer_offset)) {
    return Result::INVALID_FOOTER;
  }

  // Cover the type byte through the end of the payload and compare with the little-endian checksum.
  const std::size_t checksum_offset = footer_offset - CHECKSUM_SIZE;
  const uint16_t expected = frame_checksum(data + TYPE_OFFSET, checksum_offset - TYPE_OFFSET);
  const uint16_t received = data[checksum_offset] | (static_cast<uint16_t>(data[checksum_offset + 1]) << 8);
  return expected == received ? Result::FRAME_READY : Result::INVALID_CHECKSUM;
}

}  // namespace

FoxessFrameParser::Result FoxessFrameParser::expire(uint32_t now_ms) {
  // Check for an idle gap using only the bytes left after the last delivered frame.
  this->consume_frame();
  if (this->buffered_ == 0 || now_ms - this->last_byte_ms_ < INTER_BYTE_TIMEOUT_MS) {
    return Result::WAITING;
  }
  // A truncated transmission can leave a complete later frame inside the buffer.
  return this->resynchronize(1, Result::TIMED_OUT);
}

FoxessFrameParser::Result FoxessFrameParser::feed(uint8_t byte, uint32_t now_ms) {
  // 1. Release the previous frame and start collecting at the first possible marker.
  this->consume_frame();
  if (this->buffered_ == 0 && byte != TELEMETRY_HEADER.front()) {
    return Result::WAITING;
  }
  this->buffer_[this->buffered_++] = byte;
  this->last_byte_ms_ = now_ms;

  // 2. Validate the marker, type, and payload length as their header bytes arrive.
  if (this->buffered_ <= TELEMETRY_HEADER.size() && byte != TELEMETRY_HEADER[this->buffered_ - 1]) {
    return this->resynchronize(1, Result::WAITING);
  }
  if (this->buffered_ == HEADER_SIZE) {
    this->expected_size_ = supported_frame_size(this->buffer_.data());
    if (this->expected_size_ == 0) {
      return this->resynchronize(1, Result::INVALID_LENGTH);
    }
  }

  // 3. Wait for the advertised byte count, then check the footer and CRC before exposing the frame.
  if (this->expected_size_ == 0 || this->buffered_ < this->expected_size_) {
    return Result::WAITING;
  }

  const Result result = validate_frame(this->buffer_.data(), this->expected_size_);
  if (result != Result::FRAME_READY) {
    return this->resynchronize(1, result);
  }
  this->ready_size_ = this->expected_size_;
  return result;
}

void FoxessFrameParser::consume_frame() {
  if (this->ready_size_ != 0) {
    // Release the caller's view and look for a new frame in any bytes that followed it.
    const std::size_t end = this->ready_size_;
    this->ready_size_ = 0;
    this->resynchronize(end, Result::WAITING);
  }
}

FoxessFrameParser::Result FoxessFrameParser::resynchronize(std::size_t start, Result failure) {
  std::size_t partial_start = this->buffered_;
  std::size_t partial_size = 0;

  // Scan past the first byte of a rejected candidate or past the last delivered frame.
  for (std::size_t offset = start; offset < this->buffered_; ++offset) {
    // 1. Match the available header prefix so a split or overlapping start can be recovered.
    const uint8_t *candidate = this->buffer_.data() + offset;
    const std::size_t remaining = this->buffered_ - offset;
    const std::size_t marker_size = std::min(remaining, TELEMETRY_HEADER.size());
    if (!std::equal(TELEMETRY_HEADER.begin(), TELEMETRY_HEADER.begin() + marker_size, candidate)) {
      continue;
    }

    // 2. Read the advertised size once the full header is buffered.
    const std::size_t frame_size = remaining >= HEADER_SIZE ? supported_frame_size(candidate) : 0;

    // 3. Prefer a complete frame that passes the footer and CRC checks.
    if (frame_size != 0 && remaining >= frame_size) {
      if (validate_frame(candidate, frame_size) == Result::FRAME_READY) {
        // Keep the following bytes for the next scan. A timeout expires that partial tail.
        this->buffered_ = failure == Result::TIMED_OUT ? frame_size : remaining;
        std::memmove(this->buffer_.data(), candidate, this->buffered_);
        this->ready_size_ = frame_size;
        return Result::FRAME_READY;
      }
    } else if (partial_start == this->buffered_ && (remaining < HEADER_SIZE || frame_size != 0)) {
      // Remember the earliest viable partial frame while continuing the search.
      partial_start = offset;
      partial_size = frame_size;
    }
  }

  // 4. Move an unexpired partial frame to the front for the next read. Otherwise reset the input state.
  this->expected_size_ = 0;
  if (failure == Result::TIMED_OUT || partial_start == this->buffered_) {
    this->buffered_ = 0;
  } else {
    this->buffered_ -= partial_start;
    std::memmove(this->buffer_.data(), this->buffer_.data() + partial_start, this->buffered_);
    this->expected_size_ = partial_size;
  }
  return failure;
}

}  // namespace foxess_solar
}  // namespace esphome
