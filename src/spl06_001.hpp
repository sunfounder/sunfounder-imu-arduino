#pragma once

#include <Arduino.h>
#include "utils.hpp"

class SPL06_001 {
public:
    // I2C addresses
    static constexpr uint8_t I2C_ADDRESSES[] = {0x76, 0x77};
    static constexpr size_t I2C_ADDRESS_COUNT = sizeof(I2C_ADDRESSES) / sizeof(I2C_ADDRESSES[0]);

    // Register addresses
    static constexpr uint8_t REG_PRESSURE = 0x00;
    static constexpr uint8_t REG_TEMPERATURE = 0x03;
    static constexpr uint8_t REG_PRESSURE_CONFIGURATION = 0x06;
    static constexpr uint8_t REG_TEMPERATURE_CONFIGURATION = 0x07;
    static constexpr uint8_t REG_MEASUREMENT_CONFIGURATION = 0x08;
    static constexpr uint8_t REG_CONFIGURATION_REGISTER = 0x09;
    static constexpr uint8_t REG_INTERNAL_STATUS = 0x0A;
    static constexpr uint8_t REG_FIFO_STATUS = 0x0B;
    static constexpr uint8_t REG_RESET = 0x0C;
    static constexpr uint8_t REG_ID = 0x0D;
    static constexpr uint8_t REG_C0 = 0x10;
    static constexpr uint8_t REG_C1 = 0x11;
    static constexpr uint8_t REG_C00 = 0x13;
    static constexpr uint8_t REG_C10 = 0x15;
    static constexpr uint8_t REG_C01 = 0x18;
    static constexpr uint8_t REG_C11 = 0x1A;
    static constexpr uint8_t REG_C20 = 0x1C;
    static constexpr uint8_t REG_C21 = 0x1E;
    static constexpr uint8_t REG_C30 = 0x20;

    // Measurement modes
    static constexpr uint8_t MODE_IDLE = 0;
    static constexpr uint8_t MODE_PRESSURE = 1;
    static constexpr uint8_t MODE_TEMPERATURE = 2;
    static constexpr uint8_t MODE_CONTINUE_PRESSURE = 5;
    static constexpr uint8_t MODE_CONTINUE_TEMPERATURE = 6;
    static constexpr uint8_t MODE_CONTINUE_PRESSURE_AND_TEMPERATURE = 7;

    // Oversampling rates
    static constexpr uint8_t RATE_1 = 0;
    static constexpr uint8_t RATE_2 = 1;
    static constexpr uint8_t RATE_4 = 2;
    static constexpr uint8_t RATE_8 = 3;
    static constexpr uint8_t RATE_16 = 4;
    static constexpr uint8_t RATE_32 = 5;
    static constexpr uint8_t RATE_64 = 6;
    static constexpr uint8_t RATE_128 = 7;

    // Temperature source
    static constexpr uint8_t TEMPERATURE_EXTERNAL = 1;

    // Constants
    static constexpr float P0 = 1013.25f; // Standard sea level pressure in hPa

    SPL06_001(uint8_t address = 0, TwoWire *wire = &Wire);
    ~SPL06_001();

    bool init(
        uint8_t mode = MODE_CONTINUE_PRESSURE_AND_TEMPERATURE,
        uint8_t pressure_rate = RATE_128,
        uint8_t pressure_precision = RATE_128,
        uint8_t temperature_rate = RATE_128,
        uint8_t temperature_precision = RATE_128);

    void reset();

    // Configuration methods
    void set_pressure_configuration(uint8_t rate, uint8_t precision);
    void get_pressure_configuration(uint8_t &rate, uint8_t &precision);
    void set_temperature_configuration(uint8_t rate, uint8_t precision);
    void get_temperature_configuration(uint8_t &external, uint8_t &rate, uint8_t &precision);

    // Measurement methods
    float get_temperature();
    float get_pressure();
    float get_altitude(float pressure = -1.0f);
    void read(float &temperature, float &pressure, float &altitude);

    // Calibration methods
    void set_sea_level_pressure(float pressure);
    void set_offset(float offset);

    uint8_t get_address() const;
    uint8_t get_id() const;

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

    I2C *_i2c;
    TwoWire *_wire;
    uint8_t _address;
    uint8_t _id;
    uint8_t _mode;
    float _sea_level_pressure;
    float _offset;

    // Configuration parameters
    uint8_t _pressure_rate;
    uint8_t _pressure_precision;
    uint8_t _temperature_rate;
    uint8_t _temperature_precision;
    float _pressure_scale_factor;
    float _temperature_scale_factor;

    CalibrationCoefficients _calib_coeffs;

    // Scale factors for different oversampling rates
    static constexpr int32_t SCALE_FACTORS[] = {
        524288,    // RATE_1
        1572864,   // RATE_2
        3670016,   // RATE_4
        7864320,   // RATE_8
        253952,    // RATE_16
        516096,    // RATE_32
        1040384,   // RATE_64
        2088960    // RATE_128
    };

    bool wait_ready(uint32_t timeout = 2000);
    int32_t _get_raw_temperature();
    int32_t _get_raw_pressure();
    void _read_calibration_coefficients();
    void _read_id();

    static uint8_t find_i2c_address(TwoWire *wire);
};
