#include "qmc6310.hpp"
#include <Wire.h>
#include <math.h>

QMC6310::QMC6310(uint8_t address, TwoWire *wire) {
    _wire = wire;
    
    if (address == 0) {
        _address = find_i2c_address(wire);
    } else {
        _address = address;
    }
    
    _i2c = new I2C(_address, _wire);
    _calibrate_data_temp = nullptr;
    _calibrate_data_count = 0;
}

QMC6310::~QMC6310() {
    delete _i2c;
    
    if (_calibrate_data_temp != nullptr) {
        for (size_t i = 0; i < _calibrate_data_count; i++) {
            delete[] _calibrate_data_temp[i];
        }
        delete[] _calibrate_data_temp;
        _calibrate_data_temp = nullptr;
        _calibrate_data_count = 0;
    }
}

uint8_t QMC6310::find_i2c_address(TwoWire *wire) {
    for (size_t i = 0; i < I2C_ADDRESS_COUNT; i++) {
        if (I2C::is_device_connected(I2C_ADDRESSES[i], wire)) {
            return I2C_ADDRESSES[i];
        }
    }
    return I2C_ADDRESSES[0]; // Default to first address if none found
}

bool QMC6310::init(
    uint8_t set_reset_mode,
    uint8_t mode,
    uint8_t odr,
    uint8_t osr1,
    uint8_t osr2,
    uint8_t range) {
    
    // Set the range
    size_t range_index = (range >> 2) & 0x03;
    _range = RANGE_GAUSS[range_index];
    
    // Define the sign for X Y and Z axis
    _i2c->write_byte_data(REG_SIGN, 0x06);
    
    // Reset
    _i2c->write_byte_data(REG_CTL_2, SOFT_RST_ON);
    delay(10);
    
    // Set control register 2
    _i2c->write_byte_data(REG_CTL_2, set_reset_mode | range);
    
    // Set control register 1
    _i2c->write_byte_data(REG_CTL_1, mode | odr | osr1 | osr2);
    
    return true;
}

MagData QMC6310::get_magnetometer_data(bool raw) {
    MagData data = {0.0f, 0.0f, 0.0f};
    
    // Read raw data
    uint16_t x_raw = _i2c->read_word_data(REG_DATA_X, true); // true for little-endian
    uint16_t y_raw = _i2c->read_word_data(REG_DATA_Y, true); // true for little-endian
    uint16_t z_raw = _i2c->read_word_data(REG_DATA_Z, true); // true for little-endian
    
    // Convert to signed values
    int16_t x_signed = twos_complement(x_raw, 16);
    int16_t y_signed = twos_complement(y_raw, 16);
    int16_t z_signed = twos_complement(z_raw, 16);
    
    // Convert to Gauss
    float x_gauss = mapping(static_cast<float>(x_signed), -32768.0f, 32767.0f, -_range, _range);
    float y_gauss = mapping(static_cast<float>(y_signed), -32768.0f, 32767.0f, -_range, _range);
    float z_gauss = mapping(static_cast<float>(z_signed), -32768.0f, 32767.0f, -_range, _range);
    
    if (raw) {
        data.x = x_gauss;
        data.y = y_gauss;
        data.z = z_gauss;
    } else {
        // Apply calibration
        data.x = (x_gauss - _offsets[0]) * _scales[0];
        data.y = (y_gauss - _offsets[1]) * _scales[1];
        data.z = (z_gauss - _offsets[2]) * _scales[2];
    }
    
    return data;
}

float QMC6310::get_azimuth(const MagData &data, const char *plane) {
    float azimuth = 0.0f;
    float x = data.x;
    float y = data.y;
    float z = data.z;
    
    if (strcmp(plane, "xy") == 0) {
        azimuth = atan2(x, y) * 180.0f / M_PI;
        azimuth -= 90.0f;
    } else if (strcmp(plane, "yz") == 0) {
        azimuth = atan2(z, y) * 180.0f / M_PI;
    } else if (strcmp(plane, "xz") == 0) {
        azimuth = atan2(z, x) * 180.0f / M_PI;
    }
    
    // Ensure azimuth is in [0, 360) range
    if (azimuth < 0.0f) {
        azimuth += 360.0f;
    }
    
    return azimuth;
}

void QMC6310::read(MagData &mag_data, float &azimuth) {
    mag_data = get_magnetometer_data();
    azimuth = get_azimuth(mag_data);
}

void QMC6310::set_calibration(float x_offset, float y_offset, float z_offset,
                             float x_scale, float y_scale, float z_scale) {
    _offsets[0] = x_offset;
    _offsets[1] = y_offset;
    _offsets[2] = z_offset;
    
    _scales[0] = x_scale;
    _scales[1] = y_scale;
    _scales[2] = z_scale;
}

void QMC6310::calibrate_prepare() {
    // Free any existing calibration data
    if (_calibrate_data_temp != nullptr) {
        for (size_t i = 0; i < _calibrate_data_count; i++) {
            delete[] _calibrate_data_temp[i];
        }
        delete[] _calibrate_data_temp;
        _calibrate_data_temp = nullptr;
        _calibrate_data_count = 0;
    }
    
    // Allocate initial memory for 100 data points
    _calibrate_data_temp = new float*[100];
    for (size_t i = 0; i < 100; i++) {
        _calibrate_data_temp[i] = new float[3]();
    }
    _calibrate_data_count = 0;
}

void QMC6310::calibrate_step() {
    MagData data = get_magnetometer_data(true);
    calibrate_step(data);
}

void QMC6310::calibrate_step(const MagData &data) {
    // Reallocate memory if needed
    if (_calibrate_data_count % 100 == 0 && _calibrate_data_count > 0) {
        size_t new_size = _calibrate_data_count + 100;
        float **new_data = new float*[new_size];
        
        // Copy existing data
        for (size_t i = 0; i < _calibrate_data_count; i++) {
            new_data[i] = _calibrate_data_temp[i];
        }
        
        // Allocate new memory
        for (size_t i = _calibrate_data_count; i < new_size; i++) {
            new_data[i] = new float[3]();
        }
        
        // Free old memory
        delete[] _calibrate_data_temp;
        _calibrate_data_temp = new_data;
    }
    
    // Store data
    _calibrate_data_temp[_calibrate_data_count][0] = data.x;
    _calibrate_data_temp[_calibrate_data_count][1] = data.y;
    _calibrate_data_temp[_calibrate_data_count][2] = data.z;
    _calibrate_data_count++;
}

void QMC6310::calibrate_finish(float *offsets, float *scales) {
    if (_calibrate_data_count == 0 || _calibrate_data_temp == nullptr) {
        return;
    }
    
    // Calculate min and max values
    float x_min = _calibrate_data_temp[0][0];
    float x_max = _calibrate_data_temp[0][0];
    float y_min = _calibrate_data_temp[0][1];
    float y_max = _calibrate_data_temp[0][1];
    float z_min = _calibrate_data_temp[0][2];
    float z_max = _calibrate_data_temp[0][2];
    
    for (size_t i = 1; i < _calibrate_data_count; i++) {
        // X axis
        if (_calibrate_data_temp[i][0] < x_min) x_min = _calibrate_data_temp[i][0];
        if (_calibrate_data_temp[i][0] > x_max) x_max = _calibrate_data_temp[i][0];
        
        // Y axis
        if (_calibrate_data_temp[i][1] < y_min) y_min = _calibrate_data_temp[i][1];
        if (_calibrate_data_temp[i][1] > y_max) y_max = _calibrate_data_temp[i][1];
        
        // Z axis
        if (_calibrate_data_temp[i][2] < z_min) z_min = _calibrate_data_temp[i][2];
        if (_calibrate_data_temp[i][2] > z_max) z_max = _calibrate_data_temp[i][2];
    }
    
    // Calculate offsets and scales
    float calculated_offsets[3] = {
        (x_min + x_max) / 2.0f,
        (y_min + y_max) / 2.0f,
        (z_min + z_max) / 2.0f
    };
    
    float calculated_scales[3] = {
        (x_max - x_min) / 2.0f,
        (y_max - y_min) / 2.0f,
        (z_max - z_min) / 2.0f
    };
    
    // Calculate average scale
    float avg_scale = (calculated_scales[0] + calculated_scales[1] + calculated_scales[2]) / 3.0f;
    
    // Normalize scales
    calculated_scales[0] = avg_scale / calculated_scales[0];
    calculated_scales[1] = avg_scale / calculated_scales[1];
    calculated_scales[2] = avg_scale / calculated_scales[2];
    
    // Update internal offsets and scales
    _offsets[0] = calculated_offsets[0];
    _offsets[1] = calculated_offsets[1];
    _offsets[2] = calculated_offsets[2];
    
    _scales[0] = calculated_scales[0];
    _scales[1] = calculated_scales[1];
    _scales[2] = calculated_scales[2];
    
    // Copy to output parameters if provided
    if (offsets != nullptr) {
        offsets[0] = _offsets[0];
        offsets[1] = _offsets[1];
        offsets[2] = _offsets[2];
    }
    
    if (scales != nullptr) {
        scales[0] = _scales[0];
        scales[1] = _scales[1];
        scales[2] = _scales[2];
    }
}

uint8_t QMC6310::get_address() const {
    return _address;
}
