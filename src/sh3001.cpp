#include "sh3001.hpp"
#include <Wire.h>
#include <memory>

// Accelerometer range mapping
const float ACC_RANGE_MAP[] = {
    2.0f,   // ACC_RANGE_2G (0b101)
    4.0f,   // ACC_RANGE_4G (0b100)
    8.0f,   // ACC_RANGE_8G (0b011)
    16.0f   // ACC_RANGE_16G (0b010)
};

// Gyroscope range mapping
const float GYRO_RANGE_MAP[] = {
    125.0f,  // GYRO_RANGE_125 (0x02)
    250.0f,  // GYRO_RANGE_250 (0x03)
    500.0f,  // GYRO_RANGE_500 (0x04)
    1000.0f, // GYRO_RANGE_1000 (0x05)
    2000.0f  // GYRO_RANGE_2000 (0x06)
};

SH3001::SH3001(uint8_t address, TwoWire *wire) {
    _wire = wire;
    
    if (address == 0) {
        _address = find_i2c_address(wire);
    } else {
        _address = address;
    }
    
    _i2c = new I2C(_address, _wire);
    _accel_cali_temp = nullptr;
    _accel_cali_count = 0;
}

SH3001::~SH3001() {
    delete _i2c;
    
    if (_accel_cali_temp != nullptr) {
        for (size_t i = 0; i < _accel_cali_count; i++) {
            delete[] _accel_cali_temp[i];
        }
        delete[] _accel_cali_temp;
        _accel_cali_temp = nullptr;
        _accel_cali_count = 0;
    }
}

uint8_t SH3001::find_i2c_address(TwoWire *wire) {
    for (size_t i = 0; i < I2C_ADDRESS_COUNT; i++) {
        if (I2C::is_device_connected(I2C_ADDRESSES[i], wire)) {
            return I2C_ADDRESSES[i];
        }
    }
    return I2C_ADDRESSES[0]; // Default to first address if none found
}

bool SH3001::init() {
    uint8_t chip_id = _i2c->read_byte_data(REG_CHIP_ID);
    if (chip_id != CHIP_ID) {
        return false;
    }

    set_acceleration_configuration(
        false,   // low_power_enable
        false,   // adc_dither_enable
        true,    // filter_enable
        ACC_ODR_1000HZ, // odr
        ACC_RANGE_16G,  // range
        false,   // low_pass_filter_enable
        ACC_ODRX011     // low_pass_filter
    );
    
    set_gyroscope_configuration(
        false,           // inactive_detect_enable
        true,            // filter_enable
        GYRO_ODR_1000HZ, // odr
        false,           // low_pass_filter_enable
        GYRO_LPF_10,     // low_pass_filter
        GYRO_RANGE_2000, // range_x
        GYRO_RANGE_2000, // range_y
        GYRO_RANGE_2000  // range_z
    );
    
    set_temperature_configuration(
        true,           // enable
        TEMP_ODR_125    // odr
    );
    
    // Read room temperature reference
    uint16_t temp_data = _i2c->read_word_data(REG_TEMP_CONF0);
    _room_temp = temp_data & 0x0FFF;
    
    return true;
}

void SH3001::set_acceleration_configuration(
    bool low_power_enable,
    bool adc_dither_enable,
    bool filter_enable,
    uint8_t odr,
    uint8_t range,
    bool low_pass_filter_enable,
    uint8_t low_pass_filter) {
    
    // Configure ACC_CONF0
    uint8_t conf0 = _i2c->read_byte_data(REG_ACC_CONF0);
    conf0 &= ~(0x80 | 0x40 | 0x01); // Clear bits
    conf0 |= (low_power_enable << 7) | (adc_dither_enable << 6) | (filter_enable << 0);
    _i2c->write_byte_data(REG_ACC_CONF0, conf0);
    
    // Configure ACC_CONF1 (ODR)
    uint8_t conf1 = _i2c->read_byte_data(REG_ACC_CONF1);
    conf1 &= 0xF0; // Clear ODR bits
    conf1 |= (odr & 0x0F);
    _i2c->write_byte_data(REG_ACC_CONF1, conf1);
    
    // Configure ACC_CONF2 (range)
    uint8_t conf2 = _i2c->read_byte_data(REG_ACC_CONF2);
    conf2 &= 0xF8; // Clear range bits
    conf2 |= (range & 0x07);
    _i2c->write_byte_data(REG_ACC_CONF2, conf2);
    
    // Update accel range based on selected range
    if (range == ACC_RANGE_2G) {
        _accel_range = 2.0f;
    } else if (range == ACC_RANGE_4G) {
        _accel_range = 4.0f;
    } else if (range == ACC_RANGE_8G) {
        _accel_range = 8.0f;
    } else if (range == ACC_RANGE_16G) {
        _accel_range = 16.0f;
    }
    
    // Configure ACC_CONF3 (low pass filter)
    uint8_t conf3 = _i2c->read_byte_data(REG_ACC_CONF3);
    conf3 &= ~(0x08 | 0xE0); // Clear LPFE and LPF bits
    conf3 |= (low_pass_filter_enable << 3) | ((low_pass_filter & 0x07) << 5);
    _i2c->write_byte_data(REG_ACC_CONF3, conf3);
}

void SH3001::set_gyroscope_configuration(
    bool inactive_detect_enable,
    bool filter_enable,
    uint8_t odr,
    bool low_pass_filter_enable,
    uint8_t low_pass_filter,
    uint8_t range_x,
    uint8_t range_y,
    uint8_t range_z) {
    
    // Configure GYRO_CONF0
    uint8_t conf0 = _i2c->read_byte_data(REG_GYRO_CONF0);
    conf0 &= ~(0x10 | 0x01); // Clear bits
    conf0 |= (inactive_detect_enable << 4) | (filter_enable << 0);
    _i2c->write_byte_data(REG_GYRO_CONF0, conf0);
    
    // Configure GYRO_CONF1 (ODR)
    uint8_t conf1 = _i2c->read_byte_data(REG_GYRO_CONF1);
    conf1 &= 0xF0; // Clear ODR bits
    conf1 |= (odr & 0x0F);
    _i2c->write_byte_data(REG_GYRO_CONF1, conf1);
    
    // Configure GYRO_CONF2 (low pass filter)
    uint8_t conf2 = _i2c->read_byte_data(REG_GYRO_CONF2);
    conf2 &= ~(0x10 | 0x0C); // Clear LPFE and LPF bits
    conf2 |= (low_pass_filter_enable << 4) | ((low_pass_filter & 0x03) << 2);
    _i2c->write_byte_data(REG_GYRO_CONF2, conf2);
    
    // Configure GYRO_CONF3 (range X)
    uint8_t conf3 = _i2c->read_byte_data(REG_GYRO_CONF3);
    conf3 &= 0xF8; // Clear range bits
    conf3 |= (range_x & 0x07);
    _i2c->write_byte_data(REG_GYRO_CONF3, conf3);
    _gyro_range[0] = GYRO_RANGE_MAP[range_x - 2]; // 2 is the offset for GYRO_RANGE_125
    
    // Configure GYRO_CONF4 (range Y)
    uint8_t conf4 = _i2c->read_byte_data(REG_GYRO_CONF4);
    conf4 &= 0xF8; // Clear range bits
    conf4 |= (range_y & 0x07);
    _i2c->write_byte_data(REG_GYRO_CONF4, conf4);
    _gyro_range[1] = GYRO_RANGE_MAP[range_y - 2];
    
    // Configure GYRO_CONF5 (range Z)
    uint8_t conf5 = _i2c->read_byte_data(REG_GYRO_CONF5);
    conf5 &= 0xF8; // Clear range bits
    conf5 |= (range_z & 0x07);
    _i2c->write_byte_data(REG_GYRO_CONF5, conf5);
    _gyro_range[2] = GYRO_RANGE_MAP[range_z - 2];
}

void SH3001::set_temperature_configuration(bool enable, uint8_t odr) {
    uint8_t conf0 = _i2c->read_byte_data(REG_TEMP_CONF0);
    conf0 &= ~(0x80 | 0x30); // Clear enable and ODR bits
    conf0 |= (enable << 7) | ((odr & 0x03) << 4);
    _i2c->write_byte_data(REG_TEMP_CONF0, conf0);
}

float SH3001::read_temperature() {
    uint8_t temp_data[2];
    if (_i2c->read_i2c_block_data(REG_TEMP_DATA, temp_data, 2)) {
        // Convert raw data to 12-bit value (big-endian format)
        uint16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
        raw_temp = raw_temp & 0x0FFF; // 12-bit value
        float temperature = (raw_temp - _room_temp) / 16.0f + 25.0f;
        return temperature;
    }
    return 0.0f;
}

AccelData SH3001::read_accel(bool raw) {
    AccelData data = {0.0f, 0.0f, 0.0f};
    uint8_t accel_data[6];
    
    if (_i2c->read_i2c_block_data(REG_ACC_X, accel_data, 6)) {
        // Convert raw data to 16-bit signed values (little-endian format)
        int16_t x_raw = (accel_data[1] << 8) | accel_data[0];
        int16_t y_raw = (accel_data[3] << 8) | accel_data[2];
        int16_t z_raw = (accel_data[5] << 8) | accel_data[4];
        
        if (raw) {
            data.x = static_cast<float>(x_raw);
            data.y = static_cast<float>(y_raw);
            data.z = static_cast<float>(z_raw);
        } else {
            // Apply offset
            x_raw -= static_cast<int16_t>(_acc_offset[0]);
            y_raw -= static_cast<int16_t>(_acc_offset[1]);
            z_raw -= static_cast<int16_t>(_acc_offset[2]);
            
            // Convert to g
            float x_g = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
            float y_g = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
            float z_g = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
            
            // Convert to m/s^2
            data.x = x_g * G;
            data.y = y_g * G;
            data.z = z_g * G;
        }
    }
    
    return data;
}

GyroData SH3001::read_gyro(bool raw) {
    GyroData data = {0.0f, 0.0f, 0.0f};
    uint8_t gyro_data[6];
    
    if (_i2c->read_i2c_block_data(REG_GYRO_X, gyro_data, 6)) {
        // Convert raw data to 16-bit signed values (little-endian format)
        int16_t x_raw = (gyro_data[1] << 8) | gyro_data[0];
        int16_t y_raw = (gyro_data[3] << 8) | gyro_data[2];
        int16_t z_raw = (gyro_data[5] << 8) | gyro_data[4];
        
        if (raw) {
            data.x = static_cast<float>(x_raw);
            data.y = static_cast<float>(y_raw);
            data.z = static_cast<float>(z_raw);
        } else {
            // Apply offset
            x_raw -= static_cast<int16_t>(_gyro_offset[0]);
            y_raw -= static_cast<int16_t>(_gyro_offset[1]);
            z_raw -= static_cast<int16_t>(_gyro_offset[2]);
            
            // Convert to dps
            data.x = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f, -_gyro_range[0], _gyro_range[0]);
            data.y = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f, -_gyro_range[1], _gyro_range[1]);
            data.z = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f, -_gyro_range[2], _gyro_range[2]);
        }
    }
    
    return data;
}

void SH3001::read_all(AccelData &accel, GyroData &gyro, float &temperature) {
    uint8_t all_data[14]; // 6 bytes accel + 6 bytes gyro + 2 bytes temp
    
    if (_i2c->read_i2c_block_data(REG_ACC_X, all_data, 14)) {
        // Read accelerometer data (little-endian format)
        int16_t x_raw = (all_data[1] << 8) | all_data[0];
        int16_t y_raw = (all_data[3] << 8) | all_data[2];
        int16_t z_raw = (all_data[5] << 8) | all_data[4];
        
        // Apply offset and convert to m/s^2
        x_raw -= static_cast<int16_t>(_acc_offset[0]);
        y_raw -= static_cast<int16_t>(_acc_offset[1]);
        z_raw -= static_cast<int16_t>(_acc_offset[2]);
        
        float x_g = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
        float y_g = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
        float z_g = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f, -_accel_range, _accel_range);
        
        accel.x = x_g * G;
        accel.y = y_g * G;
        accel.z = z_g * G;
        
        // Read gyroscope data (little-endian format)
        int16_t gx_raw = (all_data[7] << 8) | all_data[6];
        int16_t gy_raw = (all_data[9] << 8) | all_data[8];
        int16_t gz_raw = (all_data[11] << 8) | all_data[10];
        
        // Apply offset and convert to dps
        gx_raw -= static_cast<int16_t>(_gyro_offset[0]);
        gy_raw -= static_cast<int16_t>(_gyro_offset[1]);
        gz_raw -= static_cast<int16_t>(_gyro_offset[2]);
        
        gyro.x = mapping(static_cast<float>(gx_raw), -32768.0f, 32767.0f, -_gyro_range[0], _gyro_range[0]);
        gyro.y = mapping(static_cast<float>(gy_raw), -32768.0f, 32767.0f, -_gyro_range[1], _gyro_range[1]);
        gyro.z = mapping(static_cast<float>(gz_raw), -32768.0f, 32767.0f, -_gyro_range[2], _gyro_range[2]);
        
        // Read temperature data (big-endian format)
        uint16_t temp_raw = (all_data[12] << 8) | all_data[13];
        temp_raw = temp_raw & 0x0FFF; // 12-bit value
        temperature = (temp_raw - _room_temp) / 16.0f + 25.0f;
    }
}

void SH3001::set_accel_offset(float x, float y, float z) {
    _acc_offset[0] = x;
    _acc_offset[1] = y;
    _acc_offset[2] = z;
}

void SH3001::set_gyro_offset(float x, float y, float z) {
    _gyro_offset[0] = x;
    _gyro_offset[1] = y;
    _gyro_offset[2] = z;
}

float *SH3001::calibrate_gyro(uint16_t times) {
    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;
    
    for (uint16_t i = 0; i < times; i++) {
        GyroData gyro_data = read_gyro(true);
        gx_sum += static_cast<int32_t>(gyro_data.x);
        gy_sum += static_cast<int32_t>(gyro_data.y);
        gz_sum += static_cast<int32_t>(gyro_data.z);
        delay(10);
    }
    
    _gyro_offset[0] = static_cast<float>(gx_sum) / times;
    _gyro_offset[1] = static_cast<float>(gy_sum) / times;
    _gyro_offset[2] = static_cast<float>(gz_sum) / times;
    
    return _gyro_offset;
}

void SH3001::calibrate_accel_prepare() {
    // Free any existing calibration data
    if (_accel_cali_temp != nullptr) {
        for (size_t i = 0; i < _accel_cali_count; i++) {
            delete[] _accel_cali_temp[i];
        }
        delete[] _accel_cali_temp;
        _accel_cali_temp = nullptr;
        _accel_cali_count = 0;
    }
    
    // Allocate initial memory for 100 data points
    _accel_cali_temp = new float*[100];
    for (size_t i = 0; i < 100; i++) {
        _accel_cali_temp[i] = new float[3]();
    }
    _accel_cali_count = 0;
}

void SH3001::calibrate_accel_step() {
    AccelData accel_data = read_accel(true);
    
    // Reallocate memory if needed
    if (_accel_cali_count % 100 == 0 && _accel_cali_count > 0) {
        size_t new_size = _accel_cali_count + 100;
        float **new_data = new float*[new_size];
        
        // Copy existing data
        for (size_t i = 0; i < _accel_cali_count; i++) {
            new_data[i] = _accel_cali_temp[i];
        }
        
        // Allocate new memory
        for (size_t i = _accel_cali_count; i < new_size; i++) {
            new_data[i] = new float[3]();
        }
        
        // Free old memory
        delete[] _accel_cali_temp;
        _accel_cali_temp = new_data;
    }
    
    // Store data
    _accel_cali_temp[_accel_cali_count][0] = accel_data.x;
    _accel_cali_temp[_accel_cali_count][1] = accel_data.y;
    _accel_cali_temp[_accel_cali_count][2] = accel_data.z;
    _accel_cali_count++;
}

float *SH3001::calibrate_accel_finish() {
    if (_accel_cali_count == 0 || _accel_cali_temp == nullptr) {
        return _acc_offset;
    }
    
    // Calculate min and max values
    float x_min = _accel_cali_temp[0][0];
    float x_max = _accel_cali_temp[0][0];
    float y_min = _accel_cali_temp[0][1];
    float y_max = _accel_cali_temp[0][1];
    float z_min = _accel_cali_temp[0][2];
    float z_max = _accel_cali_temp[0][2];
    
    for (size_t i = 1; i < _accel_cali_count; i++) {
        // X axis
        if (_accel_cali_temp[i][0] < x_min) x_min = _accel_cali_temp[i][0];
        if (_accel_cali_temp[i][0] > x_max) x_max = _accel_cali_temp[i][0];
        
        // Y axis
        if (_accel_cali_temp[i][1] < y_min) y_min = _accel_cali_temp[i][1];
        if (_accel_cali_temp[i][1] > y_max) y_max = _accel_cali_temp[i][1];
        
        // Z axis
        if (_accel_cali_temp[i][2] < z_min) z_min = _accel_cali_temp[i][2];
        if (_accel_cali_temp[i][2] > z_max) z_max = _accel_cali_temp[i][2];
    }
    
    // Calculate offsets
    _acc_offset[0] = (x_min + x_max) / 2.0f;
    _acc_offset[1] = (y_min + y_max) / 2.0f;
    _acc_offset[2] = (z_min + z_max) / 2.0f;
    
    return _acc_offset;
}

uint8_t SH3001::get_address() const {
    return _address;
}
