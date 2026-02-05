#pragma once

#include "motion_sensor.hpp"

#define I2C_ADDRESSES {0x6B, 0x6A}
#define I2C_ADDRESS_COUNT 2

class QMI8658B : public MotionSensor {
public:
  QMI8658B(TwoWire *wire, uint8_t address) : MotionSensor(wire, address) {
    _chip_name = "QMI8658B";
  }

  bool begin() override;
  bool set_accel_range(uint8_t range);
  bool set_gyro_range(uint8_t range);
  bool reset() override;

protected:
  bool _read_temperature() override;
  bool _read_accel() override;
  bool _read_gyro() override;
  bool _read_all() override;

private:
  bool set_mode(uint8_t mode);
  bool set_ctrl1(
      // Enables SPI interface with: 0 = 4-wire, 1 = 3-wire, default 0
      uint8_t sim = 255,
      // Serial interface address: 0: none, 1 auto, default 0
      uint8_t addr_ai = 255,
      // Endianness: 0 = little endian, 1 = big endian, default 1
      uint8_t be = 255,
      // INT2: 0 = high-z mode, 1 = output enabled, default 0
      uint8_t int2_en = 255,
      // INT1: 0 = high-z mode, 1 = output enabled, default 0
      uint8_t int1_en = 255,
      // FIFO interrupt selection: 0 = INT2, 1 = INT1, default 0
      uint8_t fifo_int_sel = 255,
      // Internal high-speed oscillator: 0 = enabled, 1 = disabled, default 0
      uint8_t sensor_disable = 255);
  bool set_ctrl2(
      // Accelerometer Self-Test, default False (disabled)
      uint8_t acc_self_test = 255,
      // Accelerometer Full Scale, default 0 (±2g)
      uint8_t acc_fs = 255,
      // Accelerometer Output DataRate, default 0 (7174.4Hz)
      uint8_t acc_odr = 255);

  bool set_ctrl3(
      // Gyro self-Test, default False (disabled)
      uint8_t gyro_self_test = 255,
      // Gyroscope Full Scale, default 0 (±16 dps)
      uint8_t gyro_fs = 255,
      // Gyroscope Output DataRate, default 0 (7174.4 Hz)
      uint8_t gyro_odr = 255);
  bool set_ctrl5(
      // Gyroscope Low-Pass Filter mode, default 0 (2.66% of ODR)
      uint8_t gyro_lpf_mode = 255,
      // Gyroscope Low-Pass Filter Enable, default False (disabled)
      uint8_t gyro_lpf_en = 255,
      // Accelerometer Low-Pass Filter mode, default 0 (2.66% of ODR)
      uint8_t accel_lpf_mode = 255,
      // Accelerometer Low-Pass Filter enable, default False (disabled)
      uint8_t accel_lpf_en = 255);
  bool set_ctrl7(
      // SyncSample mode enable, default False (disabled)
      uint8_t sync_sample = 255,
      // DRDY (Data Ready) disable, default False (enabled)
      uint8_t drdy_dis = 255,
      // Gyroscope Snooze Mode, default False (Full Mode)
      uint8_t gyro_snooze = 255,
      // Gyroscope enable, default False (disabled)
      uint8_t gyro_en = 255,
      // Accelerometer enable, default False (disabled)
      uint8_t accel_en = 255);
  bool set_ctrl8(
      // CTRL9 handshake type, default False (use INT1)
      uint8_t ctrl9_handshake_type = 255,
      // Activity Detection interrupt selection, default False (INT2)
      uint8_t activity_int_sel = 255,
      // Significant Motion engine enable, default False (disabled)
      uint8_t sig_motion_en = 255,
      // No Motion engine enable, default False (disabled)
      uint8_t no_motion_en = 255,
      // Any Motion engine enable, default False (disabled)
      uint8_t any_motion_en = 255,
      // Tap engine enable, default False (disabled)
      uint8_t tap_en = 255);
};
