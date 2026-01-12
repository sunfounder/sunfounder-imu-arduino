#pragma once

#include <Arduino.h>
#include <Wire.h>

class I2C {
private:
  TwoWire *_wire;
  uint8_t _address;
  uint8_t _retry;

public:
  I2C(TwoWire *wire, uint8_t address);
  ~I2C();

  void begin(uint8_t retry = 5);
  bool write_byte(uint8_t data);
  bool write_byte_data(uint8_t reg, uint8_t data);
  bool write_word_data(uint8_t reg, uint16_t data, bool lsb = false);
  bool write_i2c_block_data(uint8_t reg, const uint8_t *data, uint8_t length);
  bool write_i2c_block_data(uint8_t reg, const uint8_t *data, size_t length) {
    return write_i2c_block_data(reg, data, static_cast<uint8_t>(length));
  }

  bool read_byte(uint8_t *data);
  bool read_byte_data(uint8_t reg, uint8_t *data);
  bool read_word_data(uint8_t reg, uint16_t *data, bool lsb = false);
  bool read_i2c_block_data(uint8_t reg, uint8_t *data, uint8_t length);

  static uint8_t scan(TwoWire *wire, uint8_t *addresses);
};
