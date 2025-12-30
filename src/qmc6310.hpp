#pragma once

#include <Arduino.h>
#include "utils.hpp"

// Data structure
typedef struct {
    float x;
    float y;
    float z;
} MagData;

class QMC6310 {
public:
    // I2C addresses
    static constexpr uint8_t I2C_ADDRESSES[] = {0x1C, 0x3C};
    static constexpr size_t I2C_ADDRESS_COUNT = sizeof(I2C_ADDRESSES) / sizeof(I2C_ADDRESSES[0]);

    // Register addresses
    static constexpr uint8_t REG_DATA_X = 0x01;
    static constexpr uint8_t REG_DATA_Y = 0x03;
    static constexpr uint8_t REG_DATA_Z = 0x05;
    static constexpr uint8_t REG_STATUS = 0x09;
    static constexpr uint8_t REG_CTL_1 = 0x0A;
    static constexpr uint8_t REG_CTL_2 = 0x0B;
    static constexpr uint8_t REG_SIGN = 0x29;

    // Modes
    static constexpr uint8_t MODE_SUSPEND = 0b00 << 0;
    static constexpr uint8_t MODE_NORMAL = 0b01 << 0;
    static constexpr uint8_t MODE_SINGLE = 0b10 << 0;
    static constexpr uint8_t MODE_CONTINUOUS = 0b11 << 0;

    // Output data rates
    static constexpr uint8_t ODR_10_HZ = 0b00 << 2;
    static constexpr uint8_t ODR_50_HZ = 0b01 << 2;
    static constexpr uint8_t ODR_100_HZ = 0b10 << 2;
    static constexpr uint8_t ODR_200_HZ = 0b11 << 2;

    // OSR1 values
    static constexpr uint8_t OSR1_8 = 0b00 << 4;
    static constexpr uint8_t OSR1_4 = 0b01 << 4;
    static constexpr uint8_t OSR1_2 = 0b10 << 4;
    static constexpr uint8_t OSR1_1 = 0b11 << 4;

    // OSR2 values
    static constexpr uint8_t OSR2_1 = 0b00 << 6;
    static constexpr uint8_t OSR2_2 = 0b01 << 6;
    static constexpr uint8_t OSR2_4 = 0b10 << 6;
    static constexpr uint8_t OSR2_8 = 0b11 << 6;

    // Set/Reset modes
    static constexpr uint8_t SET_RESET_ON = 0b00 << 0;
    static constexpr uint8_t SET_ONLY_ON = 0b01 << 0;
    static constexpr uint8_t SET_RESET_OFF = 0b10 << 0;

    // Ranges
    static constexpr uint8_t RANGE_30_GAUSS = 0b00 << 2;
    static constexpr uint8_t RANGE_12_GAUSS = 0b01 << 2;
    static constexpr uint8_t RANGE_8_GAUSS = 0b10 << 2;
    static constexpr uint8_t RANGE_2_GAUSS = 0b11 << 2;

    // Self-test
    static constexpr uint8_t SELF_TEST_ON = 0b1 << 6;
    static constexpr uint8_t SELF_TEST_OFF = 0b0 << 6;

    // Soft reset
    static constexpr uint8_t SOFT_RST_ON = 0b1 << 7;
    static constexpr uint8_t SOFT_RST_OFF = 0b0 << 7;

    // Range values in Gauss
    static constexpr float RANGE_GAUSS[] = {
        30.0f,  // RANGE_30_GAUSS
        12.0f,  // RANGE_12_GAUSS
        8.0f,   // RANGE_8_GAUSS
        2.0f    // RANGE_2_GAUSS
    };

    QMC6310(uint8_t address = 0, TwoWire *wire = &Wire);
    ~QMC6310();

    bool init(
        uint8_t set_reset_mode = SET_RESET_ON,
        uint8_t mode = MODE_NORMAL,
        uint8_t odr = ODR_200_HZ,
        uint8_t osr1 = OSR1_8,
        uint8_t osr2 = OSR2_8,
        uint8_t range = RANGE_8_GAUSS);

    MagData get_magnetometer_data(bool raw = false);
    float get_azimuth(const MagData &data, const char *plane = "xy");
    void read(MagData &mag_data, float &azimuth);

    // Calibration methods
    void set_calibration(float x_offset, float y_offset, float z_offset, 
                        float x_scale, float y_scale, float z_scale);
    void calibrate_prepare();
    void calibrate_step();
    void calibrate_step(const MagData &data);
    void calibrate_finish(float *offsets, float *scales);

    uint8_t get_address() const;

private:
    I2C *_i2c;
    TwoWire *_wire;
    uint8_t _address;
    float _range;
    float _offsets[3] = {0.0f, 0.0f, 0.0f};
    float _scales[3] = {1.0f, 1.0f, 1.0f};

    // Calibration data
    float **_calibrate_data_temp;
    size_t _calibrate_data_count;

    static uint8_t find_i2c_address(TwoWire *wire);
};
