#pragma once

#include "motion_sensor.hpp"

#define I2C_ADDRESSES {0x36, 0x37}
#define I2C_ADDRESS_COUNT 2

class SH3001 : public MotionSensor {
public:
  SH3001(TwoWire *wire, uint8_t address) : MotionSensor(wire, address) {}

  bool begin() override;

  bool set_accel_low_power(bool enable) override;
  bool set_accel_adc_dither_enable(bool enable) override;
  bool set_accel_filter_enable(bool enable) override;
  bool set_accel_odr(uint8_t odr) override;
  bool set_accel_range(uint8_t range) override;
  bool set_accel_low_pass_filter(bool enable, uint8_t filter) override;

  bool set_gyro_inactive_detect_enable(bool enable) override;
  bool set_gyro_filter_enable(bool enable) override;
  bool set_gyro_odr(uint8_t odr) override;
  bool set_gyro_low_pass_filter(bool enable, uint8_t filter) override;
  bool set_gyro_x_range(uint8_t range) override;
  bool set_gyro_y_range(uint8_t range) override;
  bool set_gyro_z_range(uint8_t range) override;

  bool set_temperature_enable(bool enable) override;
  bool set_temperature_odr(uint8_t odr) override;

protected:
  char _chip_name[7] = "SH3001";
  uint8_t _chip_name_len = sizeof(_chip_name) - 1;
  // Data reading methods
  bool _read_temperature() override;
  bool _read_accel() override;
  bool _read_gyro() override;
  bool _read_all() override;

private:
  float _accel_range;
  float _gyro_range[3];
  float _room_temp;
};
