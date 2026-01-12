#pragma once

#include <Wire.h>

#include "barometer.hpp"
#include "magnetometer.hpp"
#include "motion_sensor.hpp"

#define Z_PLUS 0
#define Z_MINUS 1
#define X_PLUS 2
#define X_MINUS 3
#define Y_PLUS 4
#define Y_MINUS 5

class SunFounder_IMU {
public:
  SunFounder_IMU(TwoWire *wire);

  MotionSensor *motion_sensor;
  Magnetometer *magnetometer;
  Barometer *barometer;

  bool begin();

  bool read(bool raw = false);
  Vector3f get_accel();
  Vector3f get_gyro();
  Vector3f get_magnetometer();
  float get_temperature();
  float get_azimuth();
  float get_pressure();
  float get_altitude();

  void set_accel_bias(const float bias[3]);
  void set_accel_scale(const float scale[3]);
  void set_gyro_bias(const float bias[3]);
  void set_gyro_scale(const float scale[3]);
  void set_magnetometer_bias(const float bias[3]);
  void set_magnetometer_scale(const float scale[3]);
  void set_barometer_pressure_offset(const float offset);
  void set_barometer_sealevel_pressure(const float sealevel_pressure);
  void set_orientation(uint8_t up, uint8_t front);

private:
  TwoWire *_wire;

  MotionSensor *get_motion_sensor(uint8_t *addresses, uint8_t count);
  Magnetometer *get_magnetometer(uint8_t *addresses, uint8_t count);
  Barometer *get_barometer(uint8_t *addresses, uint8_t count);
};