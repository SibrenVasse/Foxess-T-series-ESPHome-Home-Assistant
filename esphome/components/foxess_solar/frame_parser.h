#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace foxess_solar {

/**
 * @brief Extracts validated FoxESS telemetry frames from a UART byte stream.
 *
 * Each instance tracks one stream. Type 0x02 frames of 163, 165, or 245 bytes
 * use the original layout, and 251-byte frames use the shifted layout.
 * Feed each queued byte, then call expire() after draining the UART.
 * Read frame() immediately whenever either operation returns FRAME_READY.
 */
class FoxessFrameParser {
 public:
  static constexpr std::size_t ORIGINAL_LAYOUT_FRAME_SIZE_163 = 163;
  static constexpr std::size_t ORIGINAL_LAYOUT_FRAME_SIZE_165 = 165;
  static constexpr std::size_t ORIGINAL_LAYOUT_FRAME_SIZE_245 = 245;
  static constexpr std::size_t SHIFTED_LAYOUT_FRAME_SIZE_251 = 251;
  static constexpr std::size_t MAX_FRAME_SIZE = SHIFTED_LAYOUT_FRAME_SIZE_251;

  /// View of a validated frame in parser-owned storage.
  struct Frame {
    const uint8_t *data;
    std::size_t size;

    /// Return true for the measurement layout with six inserted bytes.
    bool uses_shifted_layout() const { return this->size == SHIFTED_LAYOUT_FRAME_SIZE_251; }
  };

  /// Outcome of advancing the parser or expiring an incomplete frame.
  enum class Result {
    WAITING,  ///< Continue reading. Noise and unsupported message types are skipped.
    FRAME_READY,
    INVALID_LENGTH,
    INVALID_FOOTER,
    INVALID_CHECKSUM,
    TIMED_OUT,
  };

  /**
   * @brief Expire a partial frame after a gap in software reads.
   * @param now_ms Monotonic clock in milliseconds. A 32-bit wrap is supported.
   * @return FRAME_READY if a complete frame was recovered from buffered bytes,
   * TIMED_OUT if a partial frame was discarded, or WAITING otherwise.
   */
  Result expire(uint32_t now_ms);

  /**
   * @brief Advance framing with one received byte.
   * @param byte Next byte in wire order.
   * @param now_ms Millisecond timestamp from the same clock used by expire().
   * @return The current framing outcome. Invalid frames are skipped, with any
   * viable start retained for recovery.
   */
  Result feed(uint8_t byte, uint32_t now_ms);

  /**
   * @brief Access the frame validated by the last parser operation.
   * @pre The most recent result from feed() or expire() was FRAME_READY.
   * @return A view valid until the next call to feed() or expire().
   */
  Frame frame() const { return {this->buffer_.data(), this->ready_size_}; }

 private:
  void consume_frame();
  Result resynchronize(std::size_t start, Result failure);

  std::array<uint8_t, MAX_FRAME_SIZE> buffer_{};
  std::size_t buffered_{0};
  std::size_t expected_size_{0};
  std::size_t ready_size_{0};
  uint32_t last_byte_ms_{0};
};

}  // namespace foxess_solar
}  // namespace esphome
