#pragma once

#include "magnetometer.hpp"
#include "utils.hpp"
#include <Arduino.h>

class QMC6310 : public Magnetometer {
public:
  QMC6310(TwoWire *wire, uint8_t address) : Magnetometer(wire, address) {
    _chip_name = "QMC6310";
  }

  bool begin() override;
  bool reset() override;

protected:
  bool _read() override;

private:
  bool set_control_register_1(uint8_t osr2 = 255, uint8_t osr1 = 255,
                              uint8_t odr = 255, uint8_t mode = 255);
  bool set_control_register_2(uint8_t soft_reset = 255, uint8_t self_test = 255,
                              uint8_t range = 255,
                              uint8_t set_reset_mode = 255);
  bool set_sign(bool x, bool y, bool z);
  bool set_mode(uint8_t mode);
  bool set_odr(uint8_t odr);
  bool set_osr1(uint8_t osr1);
  bool set_osr2(uint8_t osr2);
  bool set_range(uint8_t range);
  bool set_reset_mode(uint8_t set_reset_mode);
};
