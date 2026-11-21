#pragma once

#include "frame_parser.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#define SENSOR_SETTER(X) \
  void set_##X##_sensor(sensor::Sensor *sensor) { this->X##_ = sensor; }

#define PHASE_SENSOR_SETTER(X) \
  void set_phase_##X##_sensor(uint8_t phase, sensor::Sensor *X##_sensor) { \
    this->phases_[phase].X##_sensor_ = X##_sensor; \
  }

#define PV_SENSOR_SETTER(X) \
  void set_pv_##X##_sensor(uint8_t pv, sensor::Sensor *X##_sensor) { this->pvs_[pv].X##_sensor_ = X##_sensor; }

namespace esphome {
namespace foxess_solar {

/**
 * @brief Receives FoxESS telemetry and publishes the configured sensor readings.
 *
 * ESPHome owns the UART, GPIO, and sensors and binds them before setup().
 * The main loop drains the UART. The polling interval controls when the
 * component checks for lost communication.
 */
class FoxessSolar : public PollingComponent, public uart::UARTDevice {
 public:
  /// Initialize the RS485 receiver and publish the waiting status.
  void setup() override;
  /// Consume buffered UART bytes and publish complete, validated frames.
  void loop() override;
  /// Mark live readings unavailable when no valid frame arrives within the timeout.
  void update() override;

  /// @name Inverter sensor bindings
  /// The status sensor reports 0 for offline, 1 for online, 2 for a fault, and 99 while waiting.
  /// @{
  SENSOR_SETTER(loads_power)
  SENSOR_SETTER(grid_power)
  SENSOR_SETTER(generation_power)
  SENSOR_SETTER(boost_temp)
  SENSOR_SETTER(inverter_temp)
  SENSOR_SETTER(ambient_temp)
  SENSOR_SETTER(energy_production_day)
  SENSOR_SETTER(total_energy_production)
  SENSOR_SETTER(inverter_status)
  /// @}

  /// @name AC phase sensor bindings
  /// Use zero-based indices 0 through 2 for phases A through C.
  /// @{
  PHASE_SENSOR_SETTER(voltage)
  PHASE_SENSOR_SETTER(current)
  PHASE_SENSOR_SETTER(active_power)
  PHASE_SENSOR_SETTER(frequency)
  /// @}

  /// @name PV input sensor bindings
  /// Use zero-based indices 0 through 3 for inputs PV1 through PV4.
  /// @{
  PV_SENSOR_SETTER(voltage)
  PV_SENSOR_SETTER(current)
  PV_SENSOR_SETTER(active_power)
  /// @}

  /// Bind the GPIO that holds the RS485 transceiver in receive mode.
  void set_fc_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }

  /**
   * @brief Configure how long to wait for a valid telemetry frame.
   * @param timeout_ms Interval in milliseconds, from 1000 through INT32_MAX.
   */
  void set_inverter_timeout(uint32_t timeout_ms) { this->inverter_timeout_ms_ = timeout_ms; }

 private:
  enum class InverterMode : uint8_t { OFFLINE = 0, ONLINE = 1, FAULT = 2, WAITING = 99 };

  void handle_frame_result(FoxessFrameParser::Result result);
  void publish_frame(const FoxessFrameParser::Frame &frame);
  void set_inverter_mode(InverterMode mode);

  GPIOPin *flow_control_pin_{nullptr};
  uint32_t last_frame_ms_{0};
  uint32_t inverter_timeout_ms_{300000};
  FoxessFrameParser parser_;

  InverterMode inverter_mode_{InverterMode::WAITING};
  sensor::Sensor *inverter_status_{nullptr};

  struct SolarPhase {
    sensor::Sensor *voltage_sensor_{nullptr};
    sensor::Sensor *current_sensor_{nullptr};
    sensor::Sensor *active_power_sensor_{nullptr};
    sensor::Sensor *frequency_sensor_{nullptr};
  } phases_[3];

  struct SolarPV {
    sensor::Sensor *voltage_sensor_{nullptr};
    sensor::Sensor *current_sensor_{nullptr};
    sensor::Sensor *active_power_sensor_{nullptr};
  } pvs_[4];

  sensor::Sensor *loads_power_{nullptr};
  sensor::Sensor *grid_power_{nullptr};
  sensor::Sensor *generation_power_{nullptr};

  sensor::Sensor *boost_temp_{nullptr};
  sensor::Sensor *inverter_temp_{nullptr};
  sensor::Sensor *ambient_temp_{nullptr};
  sensor::Sensor *energy_production_day_{nullptr};
  sensor::Sensor *total_energy_production_{nullptr};
};

}  // namespace foxess_solar
}  // namespace esphome
