#include "spl06_001.hpp"
#include <Wire.h>
#include <math.h>

#define I2C_ADDRESSES {0x76, 0x77}
#define I2C_ADDRESS_COUNT 2

// Register addresses
#define REG_PRESSURE 0x00
#define REG_TEMPERATURE 0x03
#define REG_PRESSURE_CONFIGURATION 0x06
#define REG_TEMPERATURE_CONFIGURATION 0x07
#define REG_MEASUREMENT_CONFIGURATION 0x08
#define REG_CONFIGURATION_REGISTER 0x09
#define REG_INTERNAL_STATUS 0x0A
#define REG_FIFO_STATUS 0x0B
#define REG_RESET 0x0C
#define REG_ID 0x0D
#define REG_C0 0x10
#define REG_C1 0x11
#define REG_C00 0x13
#define REG_C10 0x15
#define REG_C01 0x18
#define REG_C11 0x1A
#define REG_C20 0x1C
#define REG_C21 0x1E
#define REG_C30 0x20

// Measurement modes
#define MODE_IDLE 0
#define MODE_PRESSURE 1
#define MODE_TEMPERATURE 2
#define MODE_CONTINUE_PRESSURE 5
#define MODE_CONTINUE_TEMPERATURE 6
#define MODE_CONTINUE_PRESSURE_AND_TEMPERATURE 7

// Oversampling rates
#define RATE_1 0
#define RATE_2 1
#define RATE_4 2
#define RATE_8 3
#define RATE_16 4
#define RATE_32 5
#define RATE_64 6
#define RATE_128 7

// Scale factors for different oversampling rates
static constexpr int32_t SCALE_FACTORS[] = {
    524288,  // RATE_1
    1572864, // RATE_2
    3670016, // RATE_4
    7864320, // RATE_8
    253952,  // RATE_16
    516096,  // RATE_32
    1040384, // RATE_64
    2088960  // RATE_128
};

// Temperature source
#define TEMPERATURE_EXTERNAL 1

bool SPL06_001::begin() {
  bool success;
  _i2c->begin();

  success = reset();
  if (!success) {
    Serial.println("Failed to reset SPL06-001");
    return false;
  }
  delay(10);

  success = set_mode(MODE_CONTINUE_PRESSURE_AND_TEMPERATURE);
  if (!success) {
    Serial.println("Failed to set mode");
    return false;
  }
  success = set_pressure_configuration(RATE_128, RATE_128);
  if (!success) {
    Serial.println("Failed to set pressure configuration");
    return false;
  }
  success =
      set_temperature_configuration(TEMPERATURE_EXTERNAL, RATE_128, RATE_128);
  if (!success) {
    Serial.println("Failed to set temperature configuration");
    return false;
  }

  // Read sensor ID
  _read_id();

  // Wait for sensor readiness
  if (!_wait_ready()) {
    return false;
  }

  // Read calibration coefficients
  _read_calibration_coefficients();

  return true;
}

bool SPL06_001::set_mode(uint8_t mode) {
  return _i2c->write_byte_data(REG_MEASUREMENT_CONFIGURATION, mode);
}
bool SPL06_001::set_pressure_configuration(uint8_t measure_rate,
                                           uint8_t precision) {
  bool success;
  uint8_t config;

  success = _i2c->read_byte_data(REG_PRESSURE_CONFIGURATION, &config);
  if (!success) {
    return false;
  }
  if (measure_rate != 255) {
    config &= ~(0b111 << 4);               // Clear pressure rate bits
    config |= (measure_rate & 0b111) << 4; // Set new pressure rate
  }
  if (precision != 255) {
    _pressure_scale_factor = SCALE_FACTORS[precision];
    config &= ~(0b111 << 0);       // Clear pressure precision bits
    config |= (precision & 0b111); // Set new pressure precision

    if (precision > 3) {
      uint8_t cfg_reg;
      success = _i2c->read_byte_data(REG_CONFIGURATION_REGISTER, &cfg_reg);
      if (!success) {
        return false;
      }
      cfg_reg |= (1 << 2);
      success = _i2c->write_byte_data(REG_CONFIGURATION_REGISTER, cfg_reg);
      if (!success) {
        return false;
      }
    }
  }
  return _i2c->write_byte_data(REG_PRESSURE_CONFIGURATION, config);
}
bool SPL06_001::set_temperature_configuration(uint8_t external,
                                              uint8_t measure_rate,
                                              uint8_t precision) {
  bool success;
  uint8_t config;

  success = _i2c->read_byte_data(REG_TEMPERATURE_CONFIGURATION, &config);
  if (!success) {
    return false;
  }

  if (external != 255) {
    config &= ~(0b1 << 7);     // Clear temperature external bit
    config |= (external << 7); // Set new temperature external bit
  }
  if (measure_rate != 255) {
    config &= ~(0b111 << 4);               // Clear temperature rate bits
    config |= (measure_rate & 0b111) << 4; // Set new temperature rate
  }
  if (precision != 255) {
    _temperature_scale_factor = SCALE_FACTORS[precision];
    config &= ~(0b111 << 0);       // Clear temperature precision bits
    config |= (precision & 0b111); // Set new temperature precision
    if (precision > 3) {
      uint8_t cfg_reg;
      success = _i2c->read_byte_data(REG_CONFIGURATION_REGISTER, &cfg_reg);
      if (!success) {
        return false;
      }
      cfg_reg |= (1 << 3);
      success = _i2c->write_byte_data(REG_CONFIGURATION_REGISTER, cfg_reg);
      if (!success) {
        return false;
      }
    }
  }
  return _i2c->write_byte_data(REG_TEMPERATURE_CONFIGURATION, config);
}

bool SPL06_001::set_pressure_rate(uint8_t rate) {
  return set_pressure_configuration(rate, 255);
}
bool SPL06_001::set_pressure_precision(uint8_t precision) {
  return set_pressure_configuration(255, precision);
}
bool SPL06_001::set_temperature_external(bool enable) {
  return set_temperature_configuration(enable, 255, 255);
}
bool SPL06_001::set_temperature_rate(uint8_t rate) {
  return set_temperature_configuration(255, rate, 255);
}
bool SPL06_001::set_temperature_precision(uint8_t precision) {
  return set_temperature_configuration(255, 255, precision);
}

bool SPL06_001::reset() { return _i2c->write_byte_data(REG_RESET, 0b1001); }

bool SPL06_001::_wait_ready(uint32_t timeout) {
  uint32_t start_time = millis();

  while (millis() - start_time < timeout) {
    bool success;
    uint8_t data;
    success = _i2c->read_byte_data(REG_MEASUREMENT_CONFIGURATION, &data);
    if (!success) {
      return false;
    }
    // When sensor is ready, these bits should be 1
    data = data >> 4 & 0x0F;

    if (data == 0x0F) {
      return true;
    }

    delay(10);
  }

  return false;
}

bool SPL06_001::_read_id() {
  uint8_t id;
  bool success;
  success = _i2c->read_byte_data(REG_ID, &id);
  if (!success) {
    return false;
  }
  _product_id = (id >> 4) & 0x0F;
  _version = id & 0x0F;
  return true;
}

bool SPL06_001::_read_calibration_coefficients() {
  uint8_t data[18];
  bool success;

  success = _i2c->read_i2c_block_data(REG_C0, data, 18);
  if (!success) {
    return false;
  }

  // Read all calibration coefficients
  _calib_coeffs.c0 = static_cast<int16_t>(
      twos_complement((data[0] << 4) | (data[1] >> 4), 12));
  _calib_coeffs.c1 = static_cast<int16_t>(
      twos_complement(((data[1] & 0x0F) << 8) | data[2], 12));

  uint32_t c00_val =
      ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
  _calib_coeffs.c00 = static_cast<int32_t>(twos_complement(c00_val, 20));

  uint32_t c10_val =
      (((uint32_t)data[5] & 0x0F) << 16) | ((uint32_t)data[6] << 8) | data[7];
  _calib_coeffs.c10 = static_cast<int32_t>(twos_complement(c10_val, 20));

  uint32_t c01_val = ((uint32_t)data[8] << 8) | data[9];
  _calib_coeffs.c01 = static_cast<int16_t>(twos_complement(c01_val, 16));

  uint32_t c11_val = ((uint32_t)data[10] << 8) | data[11];
  _calib_coeffs.c11 = static_cast<int16_t>(twos_complement(c11_val, 16));

  uint32_t c20_val = ((uint32_t)data[12] << 8) | data[13];
  _calib_coeffs.c20 = static_cast<int16_t>(twos_complement(c20_val, 16));

  uint32_t c21_val = ((uint32_t)data[14] << 8) | data[15];
  _calib_coeffs.c21 = static_cast<int16_t>(twos_complement(c21_val, 16));

  uint32_t c30_val = ((uint32_t)data[16] << 8) | data[17];
  _calib_coeffs.c30 = static_cast<int16_t>(twos_complement(c30_val, 16));
}

bool SPL06_001::_read_raw_temperature_data_into(int32_t *temperature) {
  uint8_t data[3];
  bool success;
  success = _i2c->read_i2c_block_data(REG_TEMPERATURE, data, 3);
  if (!success) {
    return false;
  }
  // Read 24-bit temperature data (big-endian format)
  uint32_t value =
      ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  *temperature = static_cast<int32_t>(twos_complement(value, 24));
  return true;
}

bool SPL06_001::_read_raw_pressure_data_into(int32_t *pressure) {
  uint8_t data[3];
  bool success;
  success = _i2c->read_i2c_block_data(REG_PRESSURE, data, 3);
  if (!success) {
    return false;
  }
  // Read 24-bit pressure data (big-endian format)
  uint32_t value =
      ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  *pressure = static_cast<int32_t>(twos_complement(value, 24));
  return true;
}

bool SPL06_001::_read_temperature(int32_t t_raw) {
  // Read raw temperature
  if (t_raw == 0) {
    bool success;
    success = _read_raw_temperature_data_into(&t_raw);
    if (!success) {
      return false;
    }
  }
  // Calculate scaled temperature
  float t_raw_sc = static_cast<float>(t_raw) / _temperature_scale_factor;

  // Apply calibration using coefficients
  float c0 = static_cast<float>(_calib_coeffs.c0);
  float c1 = static_cast<float>(_calib_coeffs.c1);

  // Temperature compensation formula from datasheet
  _temperature = (c0 * 0.5f) + (c1 * t_raw_sc);

  return true;
}

bool SPL06_001::_read_pressure(int32_t t_raw, int32_t p_raw) {
  // Read raw values
  if (t_raw == 0) {
    bool success;
    success = _read_raw_temperature_data_into(&t_raw);
    if (!success) {
      return false;
    }
  }
  if (p_raw == 0) {
    bool success;
    success = _read_raw_pressure_data_into(&p_raw);
    if (!success) {
      return false;
    }
  }

  // Calculate scaled values
  float t_raw_sc = static_cast<float>(t_raw) / _temperature_scale_factor;
  float p_raw_sc = static_cast<float>(p_raw) / _pressure_scale_factor;

  // Get calibration coefficients
  float c00 = static_cast<float>(_calib_coeffs.c00);
  float c10 = static_cast<float>(_calib_coeffs.c10);
  float c20 = static_cast<float>(_calib_coeffs.c20);
  float c30 = static_cast<float>(_calib_coeffs.c30);
  float c01 = static_cast<float>(_calib_coeffs.c01);
  float c11 = static_cast<float>(_calib_coeffs.c11);
  float c21 = static_cast<float>(_calib_coeffs.c21);

  // Apply pressure compensation formula from datasheet
  _pressure = c00 + p_raw_sc * (c10 + p_raw_sc * (c20 + p_raw_sc * c30));
  _pressure += t_raw_sc * c01 + t_raw_sc * p_raw_sc * (c11 + p_raw_sc * c21);
  _pressure /= 100.0f; // Convert to hPa

  return true;
}

bool SPL06_001::_read() {
  int32_t t_raw = 0;
  int32_t p_raw = 0;

  bool success;

  success = _read_raw_temperature_data_into(&t_raw);
  if (!success) {
    return false;
  }
  success = _read_raw_pressure_data_into(&p_raw);
  if (!success) {
    return false;
  }

  success = _read_temperature(t_raw);
  if (!success) {
    return false;
  }
  success = _read_pressure(t_raw, p_raw);
  if (!success) {
    return false;
  }
  return true;
}
