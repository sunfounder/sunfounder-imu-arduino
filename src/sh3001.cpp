#include "sh3001.hpp"

// Register addresses
#define REG_ACC_X 0x00
#define REG_ACC_Y 0x02
#define REG_ACC_Z 0x04
#define REG_GYRO_X 0x06
#define REG_GYRO_Y 0x08
#define REG_GYRO_Z 0x0A
#define REG_TEMP_DATA 0x0C
#define REG_CHIP_ID 0x0F

#define REG_TEMP_CONF0 0x20
#define REG_TEMP_CONF1 0x21
#define REG_ACC_CONF0 0x22
#define REG_ACC_CONF1 0x23
#define REG_ACC_CONF2 0x25
#define REG_ACC_CONF3 0x26
#define REG_GYRO_CONF0 0x28
#define REG_GYRO_CONF1 0x29
#define REG_GYRO_CONF2 0x2B
#define REG_GYRO_CONF3 0x8F
#define REG_GYRO_CONF4 0x9F
#define REG_GYRO_CONF5 0xAF

// Configuration macros
// Accelerometer ODR
#define ACC_ODR_1000HZ 0b0000
#define ACC_ODR_500HZ 0b0001
#define ACC_ODR_250HZ 0b0010
#define ACC_ODR_125HZ 0b0011
#define ACC_ODR_63HZ 0b0100
#define ACC_ODR_31HZ 0b0101
#define ACC_ODR_16HZ 0b0110
#define ACC_ODR_2000HZ 0b1000
#define ACC_ODR_4000HZ 0b1001
#define ACC_ODR_8000HZ 0b1010

// Accelerometer range
#define ACC_RANGE_16G 0b010
#define ACC_RANGE_8G 0b011
#define ACC_RANGE_4G 0b100
#define ACC_RANGE_2G 0b101

// Accelerometer low pass filter
#define ACC_ODRX040 0b000
#define ACC_ODRX025 0b001
#define ACC_ODRX011 0b010
#define ACC_ODRX004 0b011

// Gyroscope ODR
#define GYRO_ODR_1000HZ 0b0000
#define GYRO_ODR_500HZ 0b0001
#define GYRO_ODR_250HZ 0b0010
#define GYRO_ODR_125HZ 0b0011
#define GYRO_ODR_63HZ 0b0100
#define GYRO_ODR_31HZ 0b0101
#define GYRO_ODR_2000HZ 0b1000
#define GYRO_ODR_4000HZ 0b1001
#define GYRO_ODR_8000HZ 0b1010
#define GYRO_ODR_16000HZ 0b1011
#define GYRO_ODR_32000HZ 0b1100

// Gyroscope range
#define GYRO_RANGE_125 0x02
#define GYRO_RANGE_250 0x03
#define GYRO_RANGE_500 0x04
#define GYRO_RANGE_1000 0x05
#define GYRO_RANGE_2000 0x06

// Gyroscope low pass filter
#define GYRO_LPF_00 0b00
#define GYRO_LPF_01 0b01
#define GYRO_LPF_10 0b10
#define GYRO_LPF_11 0b11

// Temperature ODR
#define TEMP_ODR_500 0b00
#define TEMP_ODR_250 0b01
#define TEMP_ODR_125 0b10
#define TEMP_ODR_63 0b11

// Constants
#define CHIP_ID 0x61

// Accelerometer range mapping
const float ACC_RANGE_MAP[] = {
    0,     // Reserved (0b000)
    0,     // Reserved (0b001)
    16.0f, // ACC_RANGE_16G (0b010)
    8.0f,  // ACC_RANGE_8G (0b011)
    4.0f,  // ACC_RANGE_4G (0b100)
    2.0f   // ACC_RANGE_2G (0b101)
};

// Gyroscope range mapping
const float GYRO_RANGE_MAP[] = {
    0,       // Reserved (0x00)
    0,       // Reserved (0x01)
    125.0f,  // GYRO_RANGE_125 (0x02)
    250.0f,  // GYRO_RANGE_250 (0x03)
    500.0f,  // GYRO_RANGE_500 (0x04)
    1000.0f, // GYRO_RANGE_1000 (0x05)
    2000.0f  // GYRO_RANGE_2000 (0x06)
};

bool SH3001::begin() {
  bool success;
  uint8_t chip_id;

  _i2c->begin();
  success = _i2c->read_byte_data(REG_CHIP_ID, &chip_id);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" chip ID read failed");
    return false;
  }
  if (chip_id != CHIP_ID) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" chip ID expected 0x");
    Serial.print(CHIP_ID, HEX);
    Serial.print(" got 0x");
    Serial.println(chip_id, HEX);
    return false;
  }

  success = set_accel_low_power(false);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel low power set failed");
    return false;
  }
  success = set_accel_adc_dither_enable(false);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel adc dither enable set failed");
    return false;
  }
  success = set_accel_filter_enable(true);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel filter enable set failed");
    return false;
  }
  success = set_accel_odr(ACC_ODR_1000HZ);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel odr set failed");
    return false;
  }
  success = set_accel_range(ACC_RANGE_16G);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel range set failed");
    return false;
  }
  success = set_accel_low_pass_filter(false, ACC_ODRX011);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel low pass filter set failed");
    return false;
  }

  success = set_gyro_inactive_detect_enable(false);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro inactive detect enable set failed");
    return false;
  }
  success = set_gyro_filter_enable(true);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro filter enable set failed");
    return false;
  }
  success = set_gyro_odr(GYRO_ODR_1000HZ);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro odr set failed");
    return false;
  }
  success = set_gyro_low_pass_filter(false, GYRO_LPF_10);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro low pass filter set failed");
    return false;
  }
  success = set_gyro_x_range(GYRO_RANGE_2000);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro x range set failed");
    return false;
  }
  success = set_gyro_y_range(GYRO_RANGE_2000);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro y range set failed");
    return false;
  }
  success = set_gyro_z_range(GYRO_RANGE_2000);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro z range set failed");
    return false;
  }

  success = set_temperature_enable(true);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" temperature enable set failed");
    return false;
  }
  success = set_temperature_odr(TEMP_ODR_125);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" temperature odr set failed");
    return false;
  }
  uint16_t temp_data;
  success = _i2c->read_word_data(REG_TEMP_CONF0, &temp_data);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" temperature reference read failed");
    return false;
  }
  _room_temp = temp_data & 0x0FFF;

  return true;
}

bool SH3001::set_accel_low_power(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_ACC_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf0 read failed");
    return false;
  }
  conf0 &= ~(0x80); // Clear low power bit
  conf0 |= (enable << 7);
  return _i2c->write_byte_data(REG_ACC_CONF0, conf0);
}
bool SH3001::set_accel_adc_dither_enable(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_ACC_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf0 read failed");
    return false;
  }
  conf0 &= ~(0x40); // Clear adc dither bit
  conf0 |= (enable << 6);
  return _i2c->write_byte_data(REG_ACC_CONF0, conf0);
}
bool SH3001::set_accel_filter_enable(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_ACC_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf0 read failed");
    return false;
  }
  conf0 &= ~(0x01); // Clear filter bit
  conf0 |= (enable << 0);
  return _i2c->write_byte_data(REG_ACC_CONF0, conf0);
}
bool SH3001::set_accel_odr(uint8_t odr) {
  bool success;
  uint8_t conf1;
  success = _i2c->read_byte_data(REG_ACC_CONF1, &conf1);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf1 read failed");
    return false;
  }
  conf1 &= 0xF0; // Clear ODR bits
  conf1 |= (odr & 0x0F);
  return _i2c->write_byte_data(REG_ACC_CONF1, conf1);
}
bool SH3001::set_accel_range(uint8_t range) {
  bool success;
  uint8_t conf2;
  success = _i2c->read_byte_data(REG_ACC_CONF2, &conf2);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf2 read failed");
    return false;
  }
  conf2 &= 0xF8; // Clear range bits
  conf2 |= (range & 0x07);
  _accel_range = ACC_RANGE_MAP[range];
  success = _i2c->write_byte_data(REG_ACC_CONF2, conf2);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf2 write failed");
    return false;
  }
  return true;
}
bool SH3001::set_accel_low_pass_filter(bool enable, uint8_t filter) {
  bool success;
  uint8_t conf3;
  success = _i2c->read_byte_data(REG_ACC_CONF3, &conf3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf3 read failed");
    return false;
  }
  conf3 &= ~(0xE8); // Clear low pass filter enable bit and filter bits
  conf3 |= (enable << 3);
  conf3 |= ((filter & 0x07) << 5);
  success = _i2c->write_byte_data(REG_ACC_CONF3, conf3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" accel conf3 write failed");
    return false;
  }
  return true;
}
bool SH3001::set_gyro_inactive_detect_enable(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_GYRO_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf0 read failed");
    return false;
  }
  conf0 &= ~(0x10); // Clear inactive detect bit
  conf0 |= (enable << 4);
  return _i2c->write_byte_data(REG_GYRO_CONF0, conf0);
}
bool SH3001::set_gyro_filter_enable(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_GYRO_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf0 read failed");
    return false;
  }
  conf0 &= ~(0x01); // Clear filter bit
  conf0 |= (enable << 0);
  return _i2c->write_byte_data(REG_GYRO_CONF0, conf0);
}
bool SH3001::set_gyro_odr(uint8_t odr) {
  bool success;
  uint8_t conf1;
  success = _i2c->read_byte_data(REG_GYRO_CONF1, &conf1);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf1 read failed");
    return false;
  }
  conf1 &= 0xF0; // Clear ODR bits
  conf1 |= (odr & 0x0F);
  return _i2c->write_byte_data(REG_GYRO_CONF1, conf1);
}
bool SH3001::set_gyro_low_pass_filter(bool enable, uint8_t filter) {
  bool success;
  uint8_t conf2;
  success = _i2c->read_byte_data(REG_GYRO_CONF2, &conf2);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf2 read failed");
    return false;
  }
  conf2 &= ~(0x1C); // Clear low pass filter enable bit and filter bits
  conf2 |= (enable << 4);
  conf2 |= ((filter & 0x03) << 2);
  return _i2c->write_byte_data(REG_GYRO_CONF2, conf2);
}
bool SH3001::set_gyro_x_range(uint8_t range) {
  bool success;
  uint8_t conf3;
  success = _i2c->read_byte_data(REG_GYRO_CONF3, &conf3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf3 read failed");
    return false;
  }
  conf3 &= 0xF8; // Clear range bits
  conf3 |= (range & 0x07);
  _gyro_range[0] = GYRO_RANGE_MAP[range];
  success = _i2c->write_byte_data(REG_GYRO_CONF3, conf3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf3 write failed");
    return false;
  }
  return true;
}
bool SH3001::set_gyro_y_range(uint8_t range) {
  bool success;
  uint8_t conf4;
  success = _i2c->read_byte_data(REG_GYRO_CONF4, &conf4);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf4 read failed");
    return false;
  }
  conf4 &= 0xF8; // Clear range bits
  conf4 |= (range & 0x07);
  _gyro_range[1] = GYRO_RANGE_MAP[range];
  success = _i2c->write_byte_data(REG_GYRO_CONF4, conf4);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf4 write failed");
    return false;
  }
  return true;
}
bool SH3001::set_gyro_z_range(uint8_t range) {
  bool success;
  uint8_t conf5;
  success = _i2c->read_byte_data(REG_GYRO_CONF5, &conf5);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf5 read failed");
    return false;
  }
  conf5 &= 0xF8; // Clear range bits
  conf5 |= (range & 0x07);
  _gyro_range[2] = GYRO_RANGE_MAP[range];
  success = _i2c->write_byte_data(REG_GYRO_CONF5, conf5);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" gyro conf5 write failed");
    return false;
  }
  return true;
}
bool SH3001::set_temperature_enable(bool enable) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_TEMP_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" temp conf0 read failed");
    return false;
  }
  conf0 &= ~(0x80); // Clear enable bit
  conf0 |= (enable << 7);
  return _i2c->write_byte_data(REG_TEMP_CONF0, conf0);
}

bool SH3001::set_temperature_odr(uint8_t odr) {
  bool success;
  uint8_t conf0;
  success = _i2c->read_byte_data(REG_TEMP_CONF0, &conf0);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" temp conf0 read failed");
    return false;
  }
  conf0 &= ~(0x30); // Clear ODR bits
  conf0 |= ((odr & 0x03) << 4);
  return _i2c->write_byte_data(REG_TEMP_CONF0, conf0);
}

bool SH3001::_read_temperature() {
  uint8_t temp_data[2];
  bool success = _i2c->read_i2c_block_data(REG_TEMP_DATA, temp_data, 2);
  if (!success) {
    return false;
  }
  // Convert raw data to 12-bit value (big-endian format)
  uint16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
  raw_temp = raw_temp & 0x0FFF; // 12-bit value
  _temperature = (raw_temp - _room_temp) / 16.0f + 25.0f;

  return true;
}

bool SH3001::_read_accel() {
  uint8_t accel_data[6];

  if (!_i2c->read_i2c_block_data(REG_ACC_X, accel_data, 6)) {
    return false;
  }
  // Convert raw data to 16-bit signed values (little-endian format)
  int16_t x_raw = (accel_data[1] << 8) | accel_data[0];
  int16_t y_raw = (accel_data[3] << 8) | accel_data[2];
  int16_t z_raw = (accel_data[5] << 8) | accel_data[4];

  // Convert to g first
  float x_g = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);
  float y_g = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);
  float z_g = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);

  // Set data values
  _accel_data.x = x_g;
  _accel_data.y = y_g;
  _accel_data.z = z_g;

  return true;
}

bool SH3001::_read_gyro() {
  uint8_t gyro_data[6];

  bool success = _i2c->read_i2c_block_data(REG_GYRO_X, gyro_data, 6);
  if (!success) {
    return false;
  }

  // Convert raw data to 16-bit signed values (little-endian format)
  int16_t x_raw = (gyro_data[1] << 8) | gyro_data[0];
  int16_t y_raw = (gyro_data[3] << 8) | gyro_data[2];
  int16_t z_raw = (gyro_data[5] << 8) | gyro_data[4];

  // Convert to dps first
  float x_dps = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f,
                        -_gyro_range[0], _gyro_range[0]);
  float y_dps = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f,
                        -_gyro_range[1], _gyro_range[1]);
  float z_dps = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f,
                        -_gyro_range[2], _gyro_range[2]);

  _gyro_data.x = static_cast<float>(x_dps);
  _gyro_data.y = static_cast<float>(y_dps);
  _gyro_data.z = static_cast<float>(z_dps);

  return true;
}

bool SH3001::_read_all() {
  uint8_t all_data[14]; // 6 bytes accel + 6 bytes gyro + 2 bytes temp

  bool success = _i2c->read_i2c_block_data(REG_ACC_X, all_data, 14);
  if (!success) {
    Serial.println("[Error] Failed to read all data");
    return false;
  }
  // Read accelerometer data (little-endian format)
  int16_t x_raw = (all_data[1] << 8) | all_data[0];
  int16_t y_raw = (all_data[3] << 8) | all_data[2];
  int16_t z_raw = (all_data[5] << 8) | all_data[4];

  // Convert to g
  float x_g = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);
  float y_g = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);
  float z_g = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f,
                      -_accel_range, _accel_range);

  // Set accelerometer data
  _accel_data.x = x_g;
  _accel_data.y = y_g;
  _accel_data.z = z_g;

  // Read gyroscope data (little-endian format)
  int16_t gx_raw = (all_data[7] << 8) | all_data[6];
  int16_t gy_raw = (all_data[9] << 8) | all_data[8];
  int16_t gz_raw = (all_data[11] << 8) | all_data[10];

  // Convert to dps
  float gx_dps = mapping(static_cast<float>(gx_raw), -32768.0f, 32767.0f,
                         -_gyro_range[0], _gyro_range[0]);
  float gy_dps = mapping(static_cast<float>(gy_raw), -32768.0f, 32767.0f,
                         -_gyro_range[1], _gyro_range[1]);
  float gz_dps = mapping(static_cast<float>(gz_raw), -32768.0f, 32767.0f,
                         -_gyro_range[2], _gyro_range[2]);

  // Set gyroscope data
  _gyro_data.x = gx_dps;
  _gyro_data.y = gy_dps;
  _gyro_data.z = gz_dps;

  // Read temperature data (big-endian format)
  uint16_t temp_raw = (all_data[12] << 8) | all_data[13];
  temp_raw = temp_raw & 0x0FFF; // 12-bit value
  _temperature = (temp_raw - _room_temp) / 16.0f + 25.0f;

  return true; // Data read successfully
}
