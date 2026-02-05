#include "qmc6309.hpp"
#include <Wire.h>

// I2C addresses
#define I2C_ADDRESSES [2]{0x7C, 0x0C}
#define I2C_ADDRESS_COUNT 2

// Register addresses
#define REG_CHIP_ID 0x00
#define REG_DATA_X 0x01
#define REG_DATA_Y 0x03
#define REG_DATA_Z 0x05
#define REG_STATUS 0x09
#define REG_CTL_1 0x0A
#define REG_CTL_2 0x0B
#define REG_CTL_3 0x0C
#define REG_TEST_X 0x13
#define REG_TEST_Y 0x14
#define REG_TEST_Z 0x15

// Modes
#define MODE_SUSPEND 0b00
#define MODE_NORMAL 0b01
#define MODE_SINGLE 0b10
#define MODE_CONTINUOUS 0b11

// Output data rates
#define ODR_1_HZ 0b000
#define ODR_10_HZ 0b001
#define ODR_50_HZ 0b010
#define ODR_100_HZ 0b011
#define ODR_200_HZ 0b100

// OSR1 values
#define OSR1_8 0b00
#define OSR1_4 0b01
#define OSR1_2 0b10
#define OSR1_1 0b11

// OSR2 values
#define OSR2_1 0b000
#define OSR2_2 0b001
#define OSR2_4 0b010
#define OSR2_8 0b011
#define OSR2_16 0b100

// Set/Reset modes
#define SET_RESET_ON 0b00
#define SET_ONLY_ON 0b01
#define SET_RESET_OFF 0b10

// Ranges
#define RANGE_32_GAUSS 0b00
#define RANGE_16_GAUSS 0b01
#define RANGE_8_GAUSS 0b10

// Self-test
#define SELF_TEST_ON 0b1
#define SELF_TEST_OFF 0b0

// Soft reset
#define SOFT_RST_ON 0b1
#define SOFT_RST_OFF 0b0

// Range values in Gauss
static constexpr float RANGE_GAUSS[] = {
    32.0f, // RANGE_32_GAUSS
    16.0f, // RANGE_16_GAUSS
    8.0f,  // RANGE_8_GAUSS
};

bool QMC6309::begin() {
  bool success;

  _i2c->begin();

  success = reset();
  if (!success) {
    Serial.print("[Error] reset failed");
    return false;
  }

  success = set_control_register_2(SET_RESET_ON, ODR_200_HZ, RANGE_8_GAUSS);
  if (!success) {
    Serial.print("[Error] set_control_register_2 failed");
    return false;
  }

  success = set_control_register_1(MODE_NORMAL, OSR1_8, OSR2_8);
  if (!success) {
    Serial.print("[Error] set_control_register_1 failed");
    return false;
  }
}

bool QMC6309::set_control_register_1(uint8_t mode, uint8_t osr1, uint8_t osr2) {
  bool success;
  uint8_t cr1;

  success = _i2c->read_byte_data(REG_CTL_1, &cr1);
  if (!success) {
    Serial.print("[Error] ctl1 read failed");
    return false;
  }
  if (mode != 255) {
    cr1 &= ~(0b11 << 0);       // Clear mode bits
    cr1 |= (mode & 0b11) << 0; // Set new mode
  }
  if (osr1 != 255) {
    cr1 &= ~(0b111 << 3);       // Clear OSR1 bits
    cr1 |= (osr1 & 0b111) << 3; // Set new OSR1
  }
  if (osr2 != 255) {
    cr1 &= ~(0b111 << 5);       // Clear OSR2 bits
    cr1 |= (osr2 & 0b111) << 5; // Set new OSR2
  }
  success = _i2c->write_byte_data(REG_CTL_1, cr1);
  if (!success) {
    Serial.print("[Error] ctl1 write failed");
    return false;
  }
  return true;
}

bool QMC6309::set_control_register_2(uint8_t set_reset_mode, uint8_t odr,
                                     uint8_t range) {
  bool success;
  uint8_t cr2;
  success = _i2c->read_byte_data(REG_CTL_2, &cr2);
  if (!success) {
    Serial.print("[Error] ctl2 read failed");
    return false;
  }
  if (set_reset_mode != 255) {
    cr2 &= ~(0b11 << 0);                 // Clear set/reset mode bits
    cr2 |= (set_reset_mode & 0b11) << 0; // Set new set/reset mode
  }
  if (range != 255) {
    size_t range_index = (range >> 2) & 0x03;
    _range = RANGE_GAUSS[range_index];
    cr2 &= ~(0b11 << 2);        // Clear range bits
    cr2 |= (range & 0b11) << 2; // Set new range
  }
  if (odr != 255) {
    cr2 &= ~(0b111 << 4);      // Clear ODR bits
    cr2 |= (odr & 0b111) << 4; // Set new ODR
  }
  success = _i2c->write_byte_data(REG_CTL_2, cr2);
  if (!success) {
    Serial.print("[Error] ctl2 write failed");
    return false;
  }
  return true;
}
bool QMC6309::set_control_register_3(uint8_t self_test) {
  bool success;
  uint8_t cr3;
  success = _i2c->read_byte_data(REG_CTL_3, &cr3);
  if (!success) {
    Serial.print("[Error] ctl3 read failed");
    return false;
  }
  if (self_test != 255) {
    cr3 &= ~(0b1 << 7);            // Clear self-test bit
    cr3 |= (self_test & 0b1) << 7; // Set new self-test
  }
  success = _i2c->write_byte_data(REG_CTL_3, cr3);
  if (!success) {
    Serial.print("[Error] ctl3 write failed");
    return false;
  }
  return true;
}

bool QMC6309::set_reset_mode(uint8_t set_reset_mode) {
  return set_control_register_2(set_reset_mode, 255, 255);
}
bool QMC6309::set_mode(uint8_t mode) {
  return set_control_register_1(mode, 255, 255);
}
bool QMC6309::set_odr(uint8_t odr) {
  return set_control_register_1(255, 255, odr);
}
bool QMC6309::set_osr1(uint8_t osr1) {
  return set_control_register_1(255, osr1, 255);
}
bool QMC6309::set_osr2(uint8_t osr2) {
  return set_control_register_1(255, 255, osr2);
}
bool QMC6309::set_range(uint8_t range) {
  return set_control_register_2(255, 255, range);
}
bool QMC6309::self_test() {
  set_control_register_1(MODE_SUSPEND);
  set_control_register_1(MODE_CONTINUOUS);
  delay(20);
  set_control_register_3(SELF_TEST_ON);
  delay(150);
  set_control_register_3(SELF_TEST_OFF);
  qmc6309_status_t status = read_status();
  if (!status.self_test_ready) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" self-test failed");
    return false;
  }
  uint8_t datas[3];
  bool success;
  success = _i2c->read_i2c_block_data(REG_TEST_X, datas, 3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" self-test data read failed");
    return false;
  }
  int8_t x = (int8_t)datas[0];
  int8_t y = (int8_t)datas[1];
  int8_t z = (int8_t)datas[2];
  if (x > -50 && x < -1 && y > -50 && y < -1 && z > -50 && z < -1) {
    return true;
  } else {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" self-test data failed");
    return false;
  }
}
qmc6309_status_t QMC6309::read_status() {
  qmc6309_status_t status;
  uint8_t data;
  bool success;

  success = _i2c->read_byte_data(REG_STATUS, &data);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" status read failed");
    return status;
  }

  status.nvm_load_done = (data & (0b1 << 4)) != 0;
  status.nvm_ready = (data & (0b1 << 3)) != 0;
  status.self_test_ready = (data & (0b1 << 2)) != 0;
  status.overflow = (data & (0b1 << 1)) != 0;
  status.data_ready = (data & (0b1 << 0)) != 0;

  return status;
}

bool QMC6309::reset() {
  bool success;
  success = _i2c->write_byte_data(REG_CTL_2, SOFT_RST_ON);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" soft reset write failed");
    return false;
  }
  delay(100);
  success = _i2c->write_byte_data(REG_CTL_2, SOFT_RST_OFF);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" soft reset write failed");
    return false;
  }
  return success;
}

bool QMC6309::_read() {
  uint8_t data[6];
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;

  bool success;

  success = _i2c->read_i2c_block_data(REG_DATA_X, data, 6);
  if (!success) {
    return false;
  }

  // Read raw data
  x_raw = (int16_t)(data[1] << 8 | data[0]);
  y_raw = (int16_t)(data[3] << 8 | data[2]);
  z_raw = (int16_t)(data[5] << 8 | data[4]);

  // Convert to Gauss
  _data.x = mapping((float)x_raw, -32768.0f, 32767.0f, -_range, _range);
  _data.y = mapping((float)y_raw, -32768.0f, 32767.0f, -_range, _range);
  _data.z = mapping((float)z_raw, -32768.0f, 32767.0f, -_range, _range);

  return true;
}
