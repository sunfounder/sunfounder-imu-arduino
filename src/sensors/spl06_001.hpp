#pragma once

#include "barometer.hpp"

class SPL06_001 : public Barometer {
public:
  SPL06_001(TwoWire *wire, uint8_t address) : Barometer(wire, address) {
    _chip_name = "SPL06-001";
  }

  bool begin() override;

  bool reset() override;

private:
  struct CalibrationCoefficients {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
  };

  uint8_t _product_id;
  uint8_t _version;
  CalibrationCoefficients _calib_coeffs;
  uint32_t _pressure_scale_factor = 0;
  uint32_t _temperature_scale_factor = 0;

  bool _wait_ready(uint32_t timeout = 2000);
  bool _read_calibration_coefficients();
  bool _read_id();

  bool set_pressure_configuration(uint8_t measure_rate = 255,
                                  uint8_t precision = 255);
  bool set_temperature_configuration(uint8_t external = 255,
                                     uint8_t measure_rate = 255,
                                     uint8_t precision = 255);
  bool set_mode(uint8_t mode);

  bool set_temperature_external(bool enable);
  bool set_pressure_rate(uint8_t rate);
  bool set_pressure_precision(uint8_t precision);
  bool set_temperature_rate(uint8_t rate);
  bool set_temperature_precision(uint8_t precision);

protected:
  bool _read_raw_temperature_data_into(int32_t *t_raw);
  bool _read_raw_pressure_data_into(int32_t *p_raw);

  // Measurement methods
  bool _read_temperature(int32_t t_raw = 0) override;
  bool _read_pressure(int32_t t_raw = 0, int32_t p_raw = 0) override;
  bool _read() override;
};
