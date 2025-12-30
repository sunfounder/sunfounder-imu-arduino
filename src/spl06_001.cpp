#include "spl06_001.hpp"
#include <Wire.h>
#include <math.h>

SPL06_001::SPL06_001(uint8_t address, TwoWire *wire) {
    _wire = wire;
    
    if (address == 0) {
        _address = find_i2c_address(wire);
    } else {
        _address = address;
    }
    
    _i2c = new I2C(_address, _wire);
    _sea_level_pressure = P0;
    _offset = 0.0f;
}

SPL06_001::~SPL06_001() {
    delete _i2c;
}

uint8_t SPL06_001::find_i2c_address(TwoWire *wire) {
    for (size_t i = 0; i < I2C_ADDRESS_COUNT; i++) {
        if (I2C::is_device_connected(I2C_ADDRESSES[i], wire)) {
            return I2C_ADDRESSES[i];
        }
    }
    return I2C_ADDRESSES[0]; // Default to first address if none found
}

bool SPL06_001::init(
    uint8_t mode,
    uint8_t pressure_rate,
    uint8_t pressure_precision,
    uint8_t temperature_rate,
    uint8_t temperature_precision) {
    
    // Reset the sensor
    reset();
    delay(10);
    
    // Set mode
    _mode = mode;
    _i2c->write_byte_data(REG_MEASUREMENT_CONFIGURATION, _mode);
    
    // Configure shift settings based on precision
    uint8_t cfg_reg = 0;
    if (pressure_precision > 3) {
        cfg_reg |= (1 << 2);
    }
    if (temperature_precision > 3) {
        cfg_reg |= (1 << 3);
    }
    _i2c->write_byte_data(REG_CONFIGURATION_REGISTER, cfg_reg);
    
    // Set pressure configuration
    set_pressure_configuration(pressure_rate, pressure_precision);
    
    // Set temperature configuration
    set_temperature_configuration(temperature_rate, temperature_precision);
    
    // Read sensor ID
    _read_id();
    
    // Wait for sensor readiness
    if (!wait_ready()) {
        return false;
    }
    
    // Read calibration coefficients
    _read_calibration_coefficients();
    
    return true;
}

void SPL06_001::reset() {
    _i2c->write_byte_data(REG_RESET, 0b1001);
}

void SPL06_001::set_pressure_configuration(uint8_t rate, uint8_t precision) {
    _pressure_rate = rate;
    _pressure_precision = precision;
    _pressure_scale_factor = static_cast<float>(SCALE_FACTORS[precision]);
    
    // Update pressure configuration
    uint8_t pressure_config = (_pressure_rate << 4) | _pressure_precision;
    _i2c->write_byte_data(REG_PRESSURE_CONFIGURATION, pressure_config);
}

void SPL06_001::get_pressure_configuration(uint8_t &rate, uint8_t &precision) {
    uint8_t data = _i2c->read_byte_data(REG_PRESSURE_CONFIGURATION);
    rate = (data >> 4) & 0x07;
    precision = data & 0x07;
}

void SPL06_001::set_temperature_configuration(uint8_t rate, uint8_t precision) {
    uint8_t temperature_external = TEMPERATURE_EXTERNAL;
    _temperature_rate = rate;
    _temperature_precision = precision;
    _temperature_scale_factor = static_cast<float>(SCALE_FACTORS[precision]);
    
    // Update temperature configuration
    uint8_t temperature_config = (temperature_external << 7) | (_temperature_rate << 4) | _temperature_precision;
    _i2c->write_byte_data(REG_TEMPERATURE_CONFIGURATION, temperature_config);
}

void SPL06_001::get_temperature_configuration(uint8_t &external, uint8_t &rate, uint8_t &precision) {
    uint8_t data = _i2c->read_byte_data(REG_TEMPERATURE_CONFIGURATION);
    external = (data >> 7) & 0x01;
    rate = (data >> 4) & 0x07;
    precision = data & 0x07;
}

bool SPL06_001::wait_ready(uint32_t timeout) {
    uint32_t start_time = millis();
    
    while (millis() - start_time < timeout) {
        uint8_t data = _i2c->read_byte_data(REG_MEASUREMENT_CONFIGURATION);
        // When sensor is ready, these bits should be 1
        bool coefficients_ready = (data & 0x80) > 7 != 0;
        bool sensor_ready = (data & 0x40) > 6 != 0;
        bool temperature_ready = (data & 0x20) > 5 != 0;
        bool pressure_ready = (data & 0x10) > 4 != 0;
        
        if (coefficients_ready && sensor_ready && temperature_ready && pressure_ready) {
            return true;
        }
        
        delay(10);
    }
    
    return false;
}

void SPL06_001::_read_id() {
    _id = _i2c->read_byte_data(REG_ID);
}

void SPL06_001::_read_calibration_coefficients() {
    uint8_t data[18];
    
    if (_i2c->read_i2c_block_data(REG_C0, data, 18)) {
        // Read all calibration coefficients
        _calib_coeffs.c0 = static_cast<int16_t>(twos_complement((data[0] << 4) | (data[1] >> 4), 12));
        _calib_coeffs.c1 = static_cast<int16_t>(twos_complement(((data[1] & 0x0F) << 8) | data[2], 12));
        
        uint32_t c00_val = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
        _calib_coeffs.c00 = static_cast<int32_t>(twos_complement(c00_val, 20));
        
        uint32_t c10_val = (((uint32_t)data[5] & 0x0F) << 16) | ((uint32_t)data[6] << 8) | data[7];
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
}

int32_t SPL06_001::_get_raw_temperature() {
    uint8_t data[3];
    if (_i2c->read_i2c_block_data(REG_TEMPERATURE, data, 3)) {
        // Read 24-bit temperature data (big-endian format)
        uint32_t value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        return static_cast<int32_t>(twos_complement(value, 24));
    }
    return 0;
}

int32_t SPL06_001::_get_raw_pressure() {
    uint8_t data[3];
    if (_i2c->read_i2c_block_data(REG_PRESSURE, data, 3)) {
        // Read 24-bit pressure data (big-endian format)
        uint32_t value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        return static_cast<int32_t>(twos_complement(value, 24));
    }
    return 0;
}

float SPL06_001::get_temperature() {
    // Read raw temperature
    int32_t t_raw = _get_raw_temperature();
    
    // Calculate scaled temperature
    float t_raw_sc = static_cast<float>(t_raw) / _temperature_scale_factor;
    
    // Apply calibration using coefficients
    float c0 = static_cast<float>(_calib_coeffs.c0);
    float c1 = static_cast<float>(_calib_coeffs.c1);
    
    // Temperature compensation formula from datasheet
    float temperature = (c0 * 0.5f) + (c1 * t_raw_sc);
    
    return temperature;
}

float SPL06_001::get_pressure() {
    // Read raw values
    int32_t t_raw = _get_raw_temperature();
    int32_t p_raw = _get_raw_pressure();
    
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
    float pressure = c00 + p_raw_sc * (c10 + p_raw_sc * (c20 + p_raw_sc * c30));
    pressure += t_raw_sc * c01 + t_raw_sc * p_raw_sc * (c11 + p_raw_sc * c21);
    pressure += _offset;
    
    return pressure / 100.0f; // Convert to hPa
}

float SPL06_001::get_altitude(float pressure) {
    // Use current pressure if not provided
    if (pressure < 0.0f) {
        pressure = get_pressure();
    }
    
    // Barometric formula for altitude calculation
    float altitude = 44330.0f * (1.0f - pow((pressure / _sea_level_pressure), 0.1903f));
    
    return altitude;
}

void SPL06_001::read(float &temperature, float &pressure, float &altitude) {
    temperature = get_temperature();
    pressure = get_pressure();
    altitude = get_altitude(pressure);
}

void SPL06_001::set_sea_level_pressure(float pressure) {
    _sea_level_pressure = pressure;
}

void SPL06_001::set_offset(float offset) {
    _offset = offset;
}

uint8_t SPL06_001::get_address() const {
    return _address;
}

uint8_t SPL06_001::get_id() const {
    return _id;
}
