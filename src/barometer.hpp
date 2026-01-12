#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "i2c.hpp"
#include "utils.hpp"

// Constants
#define P0 1013.25f // Standard sea level pressure in hPa

class Barometer {
public:
  Barometer(TwoWire *wire, uint8_t address) {
    _address = address;
    _i2c = new I2C(wire, address);
  }
  virtual ~Barometer() { delete _i2c; }

  virtual bool begin() = 0;

  virtual bool set_mode(uint8_t mode) = 0;
  virtual bool set_pressure_rate(uint8_t rate) = 0;
  virtual bool set_pressure_precision(uint8_t precision) = 0;
  virtual bool set_temperature_rate(uint8_t rate) = 0;
  virtual bool set_temperature_precision(uint8_t precision) = 0;
  virtual bool reset() = 0;

  bool read_temperature() { return _read_temperature(); };
  bool read_pressure(bool raw = false) {
    bool success = _read_pressure();
    if (!success) {
      return false;
    }
    if (!raw) {
      _pressure += _offset;
    }
    return true;
  };
  bool read(bool raw = false) {
    bool success;
    success = _read();
    if (!success) {
      return false;
    }
    if (!raw) {
      _pressure += _offset;
      _altitude =
          44330.0f * (1.0f - pow((_pressure / _sealevel_pressure), 0.1903f));
    }
    return true;
  };

  void set_offset(float offset) { _offset = offset; }
  void set_sealevel_pressure(float pressure) { _sealevel_pressure = pressure; }

  float get_temperature() const { return _temperature; }
  float get_pressure() const { return _pressure; }
  float get_altitude() const { return _altitude; }
  float get_offset() const { return _offset; }
  float get_sealevel_pressure() const { return _sealevel_pressure; }
  uint8_t get_address() const { return _address; };
  uint8_t get_chip_name(char *chip_name) const {
    strncpy(chip_name, _chip_name, _chip_name_len);
    chip_name[_chip_name_len - 1] = '\0';
    return _chip_name_len;
  };

protected:
  uint8_t _address;
  I2C *_i2c;
  char _chip_name[16];
  uint8_t _chip_name_len;

  float _offset = 0.0f;
  float _sealevel_pressure = P0;

  float _temperature = 0.0f;
  float _pressure = 0.0f;
  float _altitude = 0.0f;

  virtual bool _read_temperature(int32_t t_raw = 0) = 0;
  virtual bool _read_pressure(int32_t t_raw = 0, int32_t p_raw = 0) = 0;
  virtual bool _read() = 0;
};
