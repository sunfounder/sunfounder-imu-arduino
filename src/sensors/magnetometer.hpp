#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "i2c.hpp"
#include "utils.hpp"

class Magnetometer {
public:
  Magnetometer(TwoWire *wire, uint8_t address) {
    _address = address;
    _i2c = new I2C(wire, address);
  }
  virtual ~Magnetometer() { delete _i2c; }

  virtual bool begin() = 0;
  virtual bool set_range(uint8_t range) = 0;

  virtual bool reset() = 0;

  void read_azimuth() {
    float x = _data.x;
    float y = _data.y;
    float z = _data.z;

    switch (_up) {
    case VECTOR_X_PLUS:
      _azimuth = atan2(y, z) * 180.0f / PI;
      break;
    case VECTOR_X_MINUS:
      _azimuth = atan2(z, y) * 180.0f / PI;
      break;
    case VECTOR_Y_PLUS:
      _azimuth = atan2(x, z) * 180.0f / PI;
      break;
    case VECTOR_Y_MINUS:
      _azimuth = atan2(z, x) * 180.0f / PI;
      break;
    case VECTOR_Z_PLUS:
      _azimuth = atan2(x, y) * 180.0f / PI;
      break;
    case VECTOR_Z_MINUS:
      _azimuth = atan2(y, x) * 180.0f / PI;
      break;
    default:
      break;
    }

    switch (_front) {
    case VECTOR_X_PLUS:
      _azimuth += 90.0f;
      break;
    case VECTOR_X_MINUS:
      _azimuth += 270.0f;
      break;
    case VECTOR_Y_PLUS:
      _azimuth += 0.0f;
      break;
    case VECTOR_Y_MINUS:
      _azimuth += 180.0f;
      break;
    case VECTOR_Z_PLUS:
      break;
    case VECTOR_Z_MINUS:
      break;
    default:
      break;
    }

    // Ensure azimuth is in [0, 360) range
    if (_azimuth < 0.0f) {
      _azimuth += 360.0f;
    }
  };

  bool read(bool raw = false) {
    bool success = _read();
    if (!success) {
      return false;
    }
    if (!raw) {
      // Apply calibration (bias and scale)
      _data.x = (_data.x - _bias[0]) * _scales[0];
      _data.y = (_data.y - _bias[1]) * _scales[1];
      _data.z = (_data.z - _bias[2]) * _scales[2];
      read_azimuth();
    }
    return true;
  };
  void set_bias(float x, float y, float z) {
    _bias[0] = x;
    _bias[1] = y;
    _bias[2] = z;
  }
  void set_scale(float x, float y, float z) {
    _scales[0] = x;
    _scales[1] = y;
    _scales[2] = z;
  }
  void set_bias(const float a[3]) {
    _bias[0] = a[0];
    _bias[1] = a[1];
    _bias[2] = a[2];
  }
  void set_scale(const float a[3]) {
    _scales[0] = a[0];
    _scales[1] = a[1];
    _scales[2] = a[2];
  }

  void set_orientation(uint8_t up, uint8_t front) {
    if (abs(up - front) <= 1) {
      Serial.println("[Warning] up and front not be on a same axis");
      return;
    }
    _up = up;
    _front = front;
  }

  Vector3f get_data() const { return _data; }
  float get_azimuth() const { return _azimuth; }
  uint8_t get_address() const { return _address; }
  String get_name() const { return _chip_name; }

protected:
  uint8_t _address;
  I2C *_i2c;
  String _chip_name;

  float _range;
  Vector3f _data;
  float _azimuth;
  uint8_t _up = VECTOR_Z_PLUS;
  uint8_t _front = VECTOR_X_PLUS;

  float _bias[3] = {0.0f, 0.0f, 0.0f};
  float _scales[3] = {1.0f, 1.0f, 1.0f};
  virtual bool _read() = 0;
};
