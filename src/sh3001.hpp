#pragma once

#include <Arduino.h>
#include "utils.hpp"

// Data structures
typedef struct {
    float x;
    float y;
    float z;
} AccelData;

typedef struct {
    float x;
    float y;
    float z;
} GyroData;

class SH3001 {
public:
    // I2C addresses
    static constexpr uint8_t I2C_ADDRESSES[] = {0x36, 0x37};
    static constexpr size_t I2C_ADDRESS_COUNT = sizeof(I2C_ADDRESSES) / sizeof(I2C_ADDRESSES[0]);

    // Register addresses
    static constexpr uint8_t REG_ACC_X = 0x00;
    static constexpr uint8_t REG_ACC_Y = 0x02;
    static constexpr uint8_t REG_ACC_Z = 0x04;
    static constexpr uint8_t REG_GYRO_X = 0x06;
    static constexpr uint8_t REG_GYRO_Y = 0x08;
    static constexpr uint8_t REG_GYRO_Z = 0x0A;
    static constexpr uint8_t REG_TEMP_DATA = 0x0C;
    static constexpr uint8_t REG_CHIP_ID = 0x0F;

    static constexpr uint8_t REG_TEMP_CONF0 = 0x20;
    static constexpr uint8_t REG_TEMP_CONF1 = 0x21;
    static constexpr uint8_t REG_ACC_CONF0 = 0x22;
    static constexpr uint8_t REG_ACC_CONF1 = 0x23;
    static constexpr uint8_t REG_ACC_CONF2 = 0x25;
    static constexpr uint8_t REG_ACC_CONF3 = 0x26;
    static constexpr uint8_t REG_GYRO_CONF0 = 0x28;
    static constexpr uint8_t REG_GYRO_CONF1 = 0x29;
    static constexpr uint8_t REG_GYRO_CONF2 = 0x2B;
    static constexpr uint8_t REG_GYRO_CONF3 = 0x8F;
    static constexpr uint8_t REG_GYRO_CONF4 = 0x9F;
    static constexpr uint8_t REG_GYRO_CONF5 = 0xAF;

    // Configuration macros
    // Accelerometer ODR
    static constexpr uint8_t ACC_ODR_1000HZ = 0b0000;
    static constexpr uint8_t ACC_ODR_500HZ = 0b0001;
    static constexpr uint8_t ACC_ODR_250HZ = 0b0010;
    static constexpr uint8_t ACC_ODR_125HZ = 0b0011;
    static constexpr uint8_t ACC_ODR_63HZ = 0b0100;
    static constexpr uint8_t ACC_ODR_31HZ = 0b0101;
    static constexpr uint8_t ACC_ODR_16HZ = 0b0110;
    static constexpr uint8_t ACC_ODR_2000HZ = 0b1000;
    static constexpr uint8_t ACC_ODR_4000HZ = 0b1001;
    static constexpr uint8_t ACC_ODR_8000HZ = 0b1010;

    // Accelerometer range
    static constexpr uint8_t ACC_RANGE_16G = 0b010;
    static constexpr uint8_t ACC_RANGE_8G = 0b011;
    static constexpr uint8_t ACC_RANGE_4G = 0b100;
    static constexpr uint8_t ACC_RANGE_2G = 0b101;

    // Accelerometer low pass filter
    static constexpr uint8_t ACC_ODRX040 = 0b000;
    static constexpr uint8_t ACC_ODRX025 = 0b001;
    static constexpr uint8_t ACC_ODRX011 = 0b010;
    static constexpr uint8_t ACC_ODRX004 = 0b011;

    // Gyroscope ODR
    static constexpr uint8_t GYRO_ODR_1000HZ = 0b0000;
    static constexpr uint8_t GYRO_ODR_500HZ = 0b0001;
    static constexpr uint8_t GYRO_ODR_250HZ = 0b0010;
    static constexpr uint8_t GYRO_ODR_125HZ = 0b0011;
    static constexpr uint8_t GYRO_ODR_63HZ = 0b0100;
    static constexpr uint8_t GYRO_ODR_31HZ = 0b0101;
    static constexpr uint8_t GYRO_ODR_2000HZ = 0b1000;
    static constexpr uint8_t GYRO_ODR_4000HZ = 0b1001;
    static constexpr uint8_t GYRO_ODR_8000HZ = 0b1010;
    static constexpr uint8_t GYRO_ODR_16000HZ = 0b1011;
    static constexpr uint8_t GYRO_ODR_32000HZ = 0b1100;

    // Gyroscope range
    static constexpr uint8_t GYRO_RANGE_125 = 0x02;
    static constexpr uint8_t GYRO_RANGE_250 = 0x03;
    static constexpr uint8_t GYRO_RANGE_500 = 0x04;
    static constexpr uint8_t GYRO_RANGE_1000 = 0x05;
    static constexpr uint8_t GYRO_RANGE_2000 = 0x06;

    // Gyroscope low pass filter
    static constexpr uint8_t GYRO_LPF_00 = 0b00;
    static constexpr uint8_t GYRO_LPF_01 = 0b01;
    static constexpr uint8_t GYRO_LPF_10 = 0b10;
    static constexpr uint8_t GYRO_LPF_11 = 0b11;

    // Temperature ODR
    static constexpr uint8_t TEMP_ODR_500 = 0b00;
    static constexpr uint8_t TEMP_ODR_250 = 0b01;
    static constexpr uint8_t TEMP_ODR_125 = 0b10;
    static constexpr uint8_t TEMP_ODR_63 = 0b11;

    // Constants
    static constexpr uint8_t CHIP_ID = 0x61;
    static constexpr float G = 9.80665f;

    SH3001(uint8_t address = 0, TwoWire *wire = &Wire);
    ~SH3001();

    bool init();

    // Configuration methods
    void set_acceleration_configuration(
        bool low_power_enable = false,
        bool adc_dither_enable = false,
        bool filter_enable = true,
        uint8_t odr = ACC_ODR_1000HZ,
        uint8_t range = ACC_RANGE_16G,
        bool low_pass_filter_enable = false,
        uint8_t low_pass_filter = ACC_ODRX011);

    void set_gyroscope_configuration(
        bool inactive_detect_enable = false,
        bool filter_enable = true,
        uint8_t odr = GYRO_ODR_1000HZ,
        bool low_pass_filter_enable = false,
        uint8_t low_pass_filter = GYRO_LPF_10,
        uint8_t range_x = GYRO_RANGE_2000,
        uint8_t range_y = GYRO_RANGE_2000,
        uint8_t range_z = GYRO_RANGE_2000);

    void set_temperature_configuration(
        bool enable = true,
        uint8_t odr = TEMP_ODR_125);

    // Data reading methods
    float read_temperature();
    AccelData read_accel(bool raw = false);
    GyroData read_gyro(bool raw = false);
    void read_all(AccelData &accel, GyroData &gyro, float &temperature);

    // Calibration methods
    void set_accel_offset(float x, float y, float z);
    void set_gyro_offset(float x, float y, float z);
    float *calibrate_gyro(uint16_t times = 100);
    void calibrate_accel_prepare();
    void calibrate_accel_step();
    float *calibrate_accel_finish();

    uint8_t get_address() const;

private:
    I2C *_i2c;
    TwoWire *_wire;
    uint8_t _address;
    float _accel_range;
    float _gyro_range[3];
    float _acc_offset[3] = {0, 0, 0};
    float _gyro_offset[3] = {0, 0, 0};
    uint16_t _room_temp;

    // Calibration data
    float **_accel_cali_temp;
    size_t _accel_cali_count;

    static uint8_t find_i2c_address(TwoWire *wire);
};
