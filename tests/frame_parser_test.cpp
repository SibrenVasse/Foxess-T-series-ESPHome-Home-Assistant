#include "frame_parser.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

using esphome::foxess_solar::FoxessFrameParser;
using Result = FoxessFrameParser::Result;

namespace {

template<std::size_t Size> std::array<uint8_t, Size> make_frame(uint16_t checksum, uint8_t type = 0x02) {
  std::array<uint8_t, Size> bytes{};
  constexpr std::size_t payload_size = Size - 13;
  bytes[0] = bytes[1] = 0x7E;
  bytes[2] = type;
  bytes[7] = static_cast<uint8_t>(payload_size >> 8);
  bytes[8] = static_cast<uint8_t>(payload_size);
  bytes[11] = 0x12;
  bytes[12] = 0x34;
  bytes[Size - 4] = static_cast<uint8_t>(checksum);
  bytes[Size - 3] = static_cast<uint8_t>(checksum >> 8);
  bytes[Size - 2] = bytes[Size - 1] = 0xE7;
  return bytes;
}

// Checksums for these fixtures are calculated independently of the parser.
const auto STANDARD = make_frame<163>(0x0BAE);
const auto LONG_STANDARD = make_frame<245>(0x8F0D);
const auto SHIFTED = make_frame<251>(0xC6D2);

Result feed(FoxessFrameParser &parser, const uint8_t *data, std::size_t size, uint32_t now_ms = 0) {
  Result result = Result::WAITING;
  for (std::size_t i = 0; i < size; ++i) {
    result = parser.feed(data[i], now_ms);
    assert(result != Result::FRAME_READY || i == size - 1);
  }
  return result;
}

template<std::size_t Size>
Result feed(FoxessFrameParser &parser, const std::array<uint8_t, Size> &bytes, uint32_t now_ms = 0) {
  return feed(parser, bytes.data(), bytes.size(), now_ms);
}

template<std::size_t Size>
void expect_frame(const FoxessFrameParser &parser, const std::array<uint8_t, Size> &expected, bool shifted = false) {
  const auto frame = parser.frame();
  assert(frame.size == expected.size());
  assert(frame.uses_shifted_layout() == shifted);
  assert(std::equal(expected.begin(), expected.end(), frame.data));
}

void accepts_complete_frames_without_gaps() {
  FoxessFrameParser parser;
  assert(feed(parser, STANDARD.data(), STANDARD.size() - 1) == Result::WAITING);
  assert(parser.feed(STANDARD.back(), 1) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
  assert(feed(parser, LONG_STANDARD) == Result::FRAME_READY);
  expect_frame(parser, LONG_STANDARD);
  assert(feed(parser, SHIFTED) == Result::FRAME_READY);
  expect_frame(parser, SHIFTED, true);
}

void rejects_unsupported_frames() {
  FoxessFrameParser parser;
  const auto other_type = make_frame<163>(0x0A3A, 0x01);
  assert(feed(parser, other_type) == Result::WAITING);

  const auto short_frame = make_frame<162>(0x04A8);
  constexpr std::size_t header_size = 9;
  assert(feed(parser, short_frame.data(), header_size) == Result::INVALID_LENGTH);
  assert(feed(parser, short_frame.data() + header_size, short_frame.size() - header_size) == Result::WAITING);

  constexpr std::array<uint16_t, 9> invalid_lengths{0, 151, 231, 233, 237, 239, 243, 65523, 65535};
  for (uint16_t length : invalid_lengths) {
    const std::array<uint8_t, header_size> header{
        0x7E, 0x7E, 0x02, 0, 0, 0, 0, static_cast<uint8_t>(length >> 8), static_cast<uint8_t>(length)};
    assert(feed(parser, header) == Result::INVALID_LENGTH);
  }

  auto bad_footer = STANDARD;
  bad_footer.back() = 0;
  assert(feed(parser, bad_footer) == Result::INVALID_FOOTER);
  auto bad_checksum = SHIFTED;
  bad_checksum[bad_checksum.size() - 4] ^= 1;
  assert(feed(parser, bad_checksum) == Result::INVALID_CHECKSUM);
  assert(feed(parser, STANDARD) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
}

void recovers_from_noise_and_overlapping_headers() {
  FoxessFrameParser parser;
  constexpr std::array<uint8_t, 6> noise{0x3A, 0xE7, 0x7E, 0x00, 0xB4, 0x7E};
  assert(feed(parser, noise) == Result::WAITING);
  assert(feed(parser, STANDARD) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
}

void keeps_a_partial_header_after_a_truncated_frame() {
  FoxessFrameParser parser;
  assert(feed(parser, STANDARD.data(), STANDARD.size() - 1) == Result::WAITING);
  assert(parser.feed(SHIFTED.front(), 0) == Result::INVALID_FOOTER);
  assert(feed(parser, SHIFTED.data() + 1, SHIFTED.size() - 1) == Result::FRAME_READY);
  expect_frame(parser, SHIFTED, true);
}

void keeps_bytes_after_a_recovered_frame() {
  FoxessFrameParser parser;
  constexpr std::size_t truncated_size = 20;
  const std::size_t recover_at = SHIFTED.size() - truncated_size - STANDARD.size();
  assert(feed(parser, SHIFTED.data(), truncated_size) == Result::WAITING);
  assert(feed(parser, STANDARD) == Result::WAITING);
  assert(feed(parser, STANDARD.data(), recover_at) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
  assert(feed(parser, STANDARD.data() + recover_at, STANDARD.size() - recover_at) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
}

void expires_incomplete_frames_across_clock_wrap() {
  FoxessFrameParser parser;
  const uint32_t start = UINT32_MAX - 50;
  assert(feed(parser, STANDARD.data(), 20, start) == Result::WAITING);
  assert(parser.expire(48) == Result::WAITING);
  assert(parser.expire(49) == Result::TIMED_OUT);
  assert(parser.expire(50) == Result::WAITING);
  assert(feed(parser, STANDARD, 51) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
}

void recovers_a_shorter_frame_when_the_shifted_frame_times_out() {
  FoxessFrameParser parser;
  assert(feed(parser, SHIFTED.data(), 20, 1) == Result::WAITING);
  assert(feed(parser, STANDARD, 2) == Result::WAITING);
  assert(parser.expire(102) == Result::FRAME_READY);
  expect_frame(parser, STANDARD);
  assert(feed(parser, SHIFTED, 103) == Result::FRAME_READY);
  expect_frame(parser, SHIFTED, true);
}

}  // namespace

int main() {
  accepts_complete_frames_without_gaps();
  rejects_unsupported_frames();
  recovers_from_noise_and_overlapping_headers();
  keeps_a_partial_header_after_a_truncated_frame();
  keeps_bytes_after_a_recovered_frame();
  expires_incomplete_frames_across_clock_wrap();
  recovers_a_shorter_frame_when_the_shifted_frame_times_out();
}
