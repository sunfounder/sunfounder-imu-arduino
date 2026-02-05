#include "qmc6310.hpp"
#include <Wire.h>

// I2C addresses
#define I2C_ADDRESSES [2]{0x1C, 0x3C}
#define I2C_ADDRESS_COUNT 2

// Register addresses
#define REG_DATA_X 0x01
#define REG_DATA_Y 0x03
#define REG_DATA_Z 0x05
#define REG_STATUS 0x09
#define REG_CTL_1 0x0A
#define REG_CTL_2 0x0B
#define REG_SIGN 0x29

// Modes
#define MODE_SUSPEND 0b00
#define MODE_NORMAL 0b01
#define MODE_SINGLE 0b10
#define MODE_CONTINUOUS 0b11

// Output data rates
#define ODR_10_HZ 0b00
#define ODR_50_HZ 0b01
#define ODR_100_HZ 0b10
#define ODR_200_HZ 0b11

// OSR1 values
#define OSR1_8 0b00
#define OSR1_4 0b01
#define OSR1_2 0b10
#define OSR1_1 0b11

// OSR2 values
#define OSR2_1 0b00
#define OSR2_2 0b01
#define OSR2_4 0b10
#define OSR2_8 0b11

// Set/Reset modes
#define SET_RESET_ON 0b00
#define SET_ONLY_ON 0b01
#define SET_RESET_OFF 0b10

// Ranges
#define RANGE_30_GAUSS 0b00
#define RANGE_12_GAUSS 0b01
#define RANGE_8_GAUSS 0b10
#define RANGE_2_GAUSS 0b11

// Self-test
#define SELF_TEST_ON 0b1
#define SELF_TEST_OFF 0b0

// Soft reset
#define SOFT_RST_ON 0b1
#define SOFT_RST_OFF 0b0

// Range values in Gauss
static constexpr float RANGE_GAUSS[] = {
    30.0f, // RANGE_30_GAUSS
    12.0f, // RANGE_12_GAUSS
    8.0f,  // RANGE_8_GAUSS
    2.0f   // RANGE_2_GAUSS
};

bool QMC6310::begin() {
  bool success;

  _i2c->begin();

  success = set_sign(true, true, false);
  if (!success) {
    Serial.print("[Error] set_sign failed");
    return false;
  }

  success = set_control_register_1(OSR2_8, OSR1_8, ODR_200_HZ, MODE_NORMAL);
  if (!success) {
    Serial.print("[Error] set_control_register_1 failed");
    return false;
  }

  success = set_control_register_2(SOFT_RST_OFF, SELF_TEST_OFF, RANGE_8_GAUSS,
                                   SET_RESET_ON);
  if (!success) {
    Serial.print("[Error] set_control_register_2 failed");
    return false;
  }
}

bool QMC6310::set_control_register_1(uint8_t osr2, uint8_t osr1, uint8_t odr,
                                     uint8_t mode) {
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
  if (odr != 255) {
    cr1 &= ~(0b11 << 2);      // Clear ODR bits
    cr1 |= (odr & 0b11) << 2; // Set new ODR
  }
  if (osr1 != 255) {
    cr1 &= ~(0b11 << 4);       // Clear OSR1 bits
    cr1 |= (osr1 & 0b11) << 4; // Set new OSR1
  }
  if (osr2 != 255) {
    cr1 &= ~(0b11 << 6);       // Clear OSR2 bits
    cr1 |= (osr2 & 0b11) << 6; // Set new OSR2
  }
  success = _i2c->write_byte_data(REG_CTL_1, cr1);
  if (!success) {
    Serial.print("[Error] ctl1 write failed");
    return false;
  }
  return true;
}

bool QMC6310::set_control_register_2(uint8_t soft_reset, uint8_t self_test,
                                     uint8_t range, uint8_t set_reset_mode) {
  bool success;
  uint8_t cr2;
  success = _i2c->read_byte_data(REG_CTL_2, &cr2);
  if (!success) {
    Serial.print("[Error] ctl2 read failed");
    return false;
  }
  if (soft_reset != 255) {
    cr2 &= ~(0b1 << 7);             // Clear soft reset bit
    cr2 |= (soft_reset & 0b1) << 7; // Set new soft reset
  }
  if (self_test != 255) {
    cr2 &= ~(0b1 << 6);            // Clear self-test bit
    cr2 |= (self_test & 0b1) << 6; // Set new self-test
  }
  if (range != 255) {
    size_t range_index = (range >> 2) & 0x03;
    _range = RANGE_GAUSS[range_index];
    cr2 &= ~(0b11 << 2);        // Clear range bits
    cr2 |= (range & 0b11) << 2; // Set new range
  }
  if (set_reset_mode != 255) {
    cr2 &= ~(0b11 << 0);                 // Clear set/reset mode bits
    cr2 |= (set_reset_mode & 0b11) << 0; // Set new set/reset mode
  }
  success = _i2c->write_byte_data(REG_CTL_2, cr2);
  if (!success) {
    Serial.print("[Error] ctl2 write failed");
    return false;
  }
  return true;
}

bool QMC6310::set_reset_mode(uint8_t set_reset_mode) {
  return set_control_register_2(255, 255, 255, set_reset_mode);
}
bool QMC6310::set_mode(uint8_t mode) {
  return set_control_register_1(255, 255, 255, mode);
}
bool QMC6310::set_odr(uint8_t odr) {
  return set_control_register_1(255, 255, odr, 255);
}
bool QMC6310::set_osr1(uint8_t osr1) {
  return set_control_register_1(255, osr1, 255, 255);
}
bool QMC6310::set_osr2(uint8_t osr2) {
  return set_control_register_1(255, 255, osr2, 255);
}
bool QMC6310::set_range(uint8_t range) {
  return set_control_register_2(255, 255, range, 255);
}
bool QMC6310::set_sign(bool x, bool y, bool z) {
  uint8_t sign = 0x00;
  sign |= (x ? 0b01 : 0b00) << 0;
  sign |= (y ? 0b01 : 0b00) << 1;
  sign |= (z ? 0b01 : 0b00) << 2;
  bool success;
  success = _i2c->write_byte_data(REG_SIGN, sign);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" sign write failed");
    return false;
  }
  return true;
}

bool QMC6310::reset() {
  bool success;
  success = _i2c->write_byte_data(REG_CTL_2, SOFT_RST_ON);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" soft reset write failed");
    return false;
  }
  delay(10);
  return success;
}

bool QMC6310::_read() {
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
  x_raw = ((int16_t)data[1] << 8) | data[0];
  y_raw = ((int16_t)data[3] << 8) | data[2];
  z_raw = ((int16_t)data[5] << 8) | data[4];

  // Convert to Gauss
  _data.x =
      mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f, -_range, _range);
  _data.y =
      mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f, -_range, _range);
  _data.z =
      mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f, -_range, _range);

  return true;
}
