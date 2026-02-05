#pragma once

#include "magnetometer.hpp"
#include "utils.hpp"
#include <Arduino.h>

typedef struct {
  bool nvm_load_done;
  bool nvm_ready;
  bool self_test_ready;
  bool overflow;
  bool data_ready;
} qmc6309_status_t;

class QMC6309 : public Magnetometer {
public:
  QMC6309(TwoWire *wire, uint8_t address) : Magnetometer(wire, address) {
    _chip_name = "QMC6309";
  }

  bool begin() override;
  bool set_range(uint8_t range) override;
  bool reset() override;

protected:
  bool _read() override;

private:
  bool set_control_register_1(uint8_t mode = 255, uint8_t osr1 = 255,
                              uint8_t osr2 = 255);
  bool set_control_register_2(uint8_t set_reset_mode = 255, uint8_t odr = 255,
                              uint8_t range = 255);
  bool set_control_register_3(uint8_t self_test = 255);
  bool set_mode(uint8_t mode);
  bool set_osr1(uint8_t osr1);
  bool set_osr2(uint8_t osr2);
  bool set_reset_mode(uint8_t set_reset_mode);
  bool set_odr(uint8_t odr);
  bool self_test();
  qmc6309_status_t read_status();
};
