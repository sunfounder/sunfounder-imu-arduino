#pragma once

#include "magnetometer.hpp"
#include "utils.hpp"
#include <Arduino.h>

class QMC6310 : public Magnetometer {
public:
  QMC6310(TwoWire *wire, uint8_t address) : Magnetometer(wire, address) {}

  bool begin() override;

  bool set_control_register_1(uint8_t osr2 = 255, uint8_t osr1 = 255,
                              uint8_t odr = 255, uint8_t mode = 255);
  bool set_control_register_2(uint8_t soft_reset = 255, uint8_t self_test = 255,
                              uint8_t range = 255,
                              uint8_t set_reset_mode = 255);
  bool set_sign(bool x, bool y, bool z) override;
  bool set_mode(uint8_t mode) override;
  bool set_odr(uint8_t odr) override;
  bool set_osr1(uint8_t osr1) override;
  bool set_osr2(uint8_t osr2) override;
  bool set_range(uint8_t range) override;
  bool set_reset_mode(uint8_t set_reset_mode) override;

  bool reset() override;

  bool _read() override;

protected:
  char _chip_name[8] = "QMC6310";
  uint8_t _chip_name_len = sizeof(_chip_name) - 1;
};
