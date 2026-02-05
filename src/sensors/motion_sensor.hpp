#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "i2c.hpp"
#include "utils.hpp"

#define G 9.80665f

class MotionSensor {
public:
  MotionSensor(TwoWire *wire, uint8_t address) {
    _address = address;
    _i2c = new I2C(wire, address);
  }
  virtual ~MotionSensor() { delete _i2c; }

  virtual bool begin() = 0;

  virtual bool set_accel_range(uint8_t range) = 0;
  virtual bool set_gyro_range(uint8_t range) = 0;
  virtual bool reset() = 0;

  void set_accel_bias(float x, float y, float z) {
    _accel_bias[0] = x;
    _accel_bias[1] = y;
    _accel_bias[2] = z;
  }
  void set_accel_scale(float x, float y, float z) {
    _accel_scale[0] = x;
    _accel_scale[1] = y;
    _accel_scale[2] = z;
  }
  void set_gyro_bias(float x, float y, float z) {
    _gyro_bias[0] = x;
    _gyro_bias[1] = y;
    _gyro_bias[2] = z;
  }
  void set_gyro_scale(float x, float y, float z) {
    _gyro_scale[0] = x;
    _gyro_scale[1] = y;
    _gyro_scale[2] = z;
  }

  void set_accel_bias(const float bias[3]) {
    set_accel_bias(bias[0], bias[1], bias[2]);
  }
  void set_accel_scale(const float scale[3]) {
    set_accel_scale(scale[0], scale[1], scale[2]);
  }
  void set_gyro_bias(const float bias[3]) {
    set_gyro_bias(bias[0], bias[1], bias[2]);
  }
  void set_gyro_scale(const float scale[3]) {
    set_gyro_scale(scale[0], scale[1], scale[2]);
  }
  void set_gravity(float g) { _gravity = g; }
  bool read_accel(bool raw = false) {
    if (!_read_accel()) {
      Serial.println("Failed to read accel data");
      return false;
    }
    if (!raw) {
      // Apply calibration (bias and scale)
      _accel_data.x = _accel_data.x * _accel_scale[0] - _accel_bias[0];
      _accel_data.y = _accel_data.y * _accel_scale[1] - _accel_bias[1];
      _accel_data.z = _accel_data.z * _accel_scale[2] - _accel_bias[2];
      // Convert to m/s^2
      _accel_data.x = _accel_data.x * _gravity;
      _accel_data.y = _accel_data.y * _gravity;
      _accel_data.z = _accel_data.z * _gravity;
    }
    return true;
  }
  bool read_gyro(bool raw = false) {
    if (!_read_gyro()) {
      Serial.println("Failed to read gyro data");
      return false;
    }
    if (!raw) {
      // Apply calibration (bias and scale)
      _gyro_data.x = _gyro_data.x * _gyro_scale[0] - _gyro_bias[0];
      _gyro_data.y = _gyro_data.y * _gyro_scale[1] - _gyro_bias[1];
      _gyro_data.z = _gyro_data.z * _gyro_scale[2] - _gyro_bias[2];
    }
    return true;
  }
  bool read_temperature() {
    if (!_read_temperature()) {
      Serial.println("Failed to read temperature data");
      return false;
    }
    return true;
  }
  bool read(bool raw = false) { return read_all(raw); }
  bool read_all(bool raw = false) {
    if (!_read_all()) {
      Serial.println("Failed to read all data");
      return false;
    }
    if (!raw) {
      // Apply calibration (bias and scale)
      _accel_data.x = _accel_data.x * _accel_scale[0] - _accel_bias[0];
      _accel_data.y = _accel_data.y * _accel_scale[1] - _accel_bias[1];
      _accel_data.z = _accel_data.z * _accel_scale[2] - _accel_bias[2];
      // Convert to m/s^2
      _accel_data.x = _accel_data.x * G;
      _accel_data.y = _accel_data.y * G;
      _accel_data.z = _accel_data.z * G;
      // Apply calibration (bias and scale)
      _gyro_data.x = _gyro_data.x * _gyro_scale[0] - _gyro_bias[0];
      _gyro_data.y = _gyro_data.y * _gyro_scale[1] - _gyro_bias[1];
      _gyro_data.z = _gyro_data.z * _gyro_scale[2] - _gyro_bias[2];
    }
    return true;
  }

  Vector3f get_accel_data() const { return _accel_data; }
  Vector3f get_gyro_data() const { return _gyro_data; }
  float get_temperature() const { return _temperature; }
  uint8_t get_address() const { return _address; }
  String get_name() const { return _chip_name; }

protected:
  uint8_t _address;
  I2C *_i2c;
  String _chip_name;
  float _gravity = G;

  float _accel_range;
  float _gyro_range;
  Vector3f _accel_data;
  Vector3f _gyro_data;
  float _temperature;

  virtual bool _read_temperature() = 0;
  virtual bool _read_accel() = 0;
  virtual bool _read_gyro() = 0;
  virtual bool _read_all() = 0;

  float _accel_bias[3] = {0.0f, 0.0f, 0.0f};
  float _accel_scale[3] = {1.0f, 1.0f, 1.0f};
  float _gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  float _gyro_scale[3] = {1.0f, 1.0f, 1.0f};
};