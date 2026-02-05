#include "qmi8658b.hpp"

// Register addresses
// General Purpose Registers
#define REG_WHO_AM_I 0x00
#define REG_REVISION_ID 0x01

// Setup and Control Registers
#define REG_CTRL1 0x02 // SPI Interface and Sensor Enable
#define REG_CTRL2 0x03 // Accelerometer: DataRate, Full Scale, Self - Test
#define REG_CTRL3 0x04 // Gyroscope: DataRate, Full Scale, Self - Test
// #define REG_CTRL4 0x05 // Reserved
#define REG_CTRL5 0x06 // Low pass filter setting
// #define REG_CTRL6 0x07 // Reserved
#define REG_CTRL7 0x08 // Enable Sensors
#define REG_CTRL8 0x09 // Motion Detection Control
#define REG_CTRL9 0x0A // Host Commands

// Host Controlled Calibration Registers
#define REG_CAL1_L 0x0B // CAL1_L – lower 8 bits. CAL1_H – upper 8 bits.
#define REG_CAL1_H 0x0C
#define REG_CAL2_L 0x0D // CAL2_L – lower 8 bits. CAL2_H – upper 8 bits.
#define REG_CAL2_H 0x0E
#define REG_CAL3_L 0x0F // CAL3_L – lower 8 bits. CAL3_H – upper 8 bits.
#define REG_CAL3_H 0x10
#define REG_CAL4_L 0x11 // CAL4_L – lower 8 bits. CAL4_H – upper 8 bits.
#define REG_CAL4_H 0x12

// FIFO Registers
#define REG_FIFO_WTM_TH 0x13   // FIFO watermark level, in ODRs
#define REG_FIFO_CTRL 0x14     // FIFO Setup
#define REG_FIFO_SMPL_CNT 0x15 // FIFO sample count LSBs
#define REG_FIFO_STATUS 0x16   // FIFO Status
#define REG_FIFO_DATA 0x17     // FIFO Data

// Status Registers
#define REG_STATUSINT 0x2D // Sensor Data Availability with Locking mechanism
#define REG_STATUS0 0x2E   // Output Data Availability
#define REG_STATUS1                                                            \
  0x2F // Miscellaneous Status: Any Motion, No Motion, Significant Motion, Tap

// Timestamp Register
#define REG_TIMESTAMP_LOW 0x30  // Sample Time Stamp - lower 8 bits
#define REG_TIMESTAMP_MID 0x31  // Sample Time Stamp - middle 8 bits
#define REG_TIMESTAMP_HIGH 0x32 // Sample Time Stamp - upper 8 bits

// Data Output Registers (16 bits 2's Complement Except COD Sensor Data)
#define REG_TEMP_L 0x33 // Temperature Output Data - lower 8 bits
#define REG_TEMP_H 0x34 // Temperature Output Data - upper 8 bits

// Accelerometer Registers
#define REG_AX_L 0x35 // X-axis Acceleration - lower 8 bits
#define REG_AX_H 0x36 // X-axis Acceleration - upper 8 bits
#define REG_AY_L 0x37 // Y-axis Acceleration - lower 8 bits
#define REG_AY_H 0x38 // Y-axis Acceleration - upper 8 bits
#define REG_AZ_L 0x39 // Z-axis Acceleration - lower 8 bits
#define REG_AZ_H 0x3A // Z-axis Acceleration - upper 8 bits

// Gyroscope Registers
#define REG_GX_L 0x3B // X-axis Angular Rate - lower 8 bits
#define REG_GX_H 0x3C // X-axis Angular Rate - upper 8 bits
#define REG_GY_L 0x3D // Y-axis Angular Rate - lower 8 bits
#define REG_GY_H 0x3E // Y-axis Angular Rate - upper 8 bits
#define REG_GZ_L 0x3F // Z-axis Angular Rate - lower 8 bits
#define REG_GZ_H 0x40 // Z-axis Angular Rate - upper 8 bits

// COD Indication and General Purpose Registers
#define REG_COD_STATUS 0x46 // Calibration-On-Demand status register
#define REG_dQW_L 0x49      // General purpose register
#define REG_dQW_H 0x4A      // General purpose register
#define REG_dQX_L 0x4B      // General purpose register
#define REG_dQX_H 0x4C      // Reserved
#define REG_dQY_L 0x4D      // General purpose register
#define REG_dQY_H 0x4E      // Reserved
#define REG_dQZ_L 0x4F      // Reserved
#define REG_dQZ_H 0x50      // Reserved
#define REG_dVX_L 0x51      // General purpose register
#define REG_dVX_H 0x52      // General purpose register
#define REG_dVY_L 0x53      // General purpose register
#define REG_dVY_H 0x54      // General purpose register
#define REG_dVZ_L 0x55      // General purpose register
#define REG_dVZ_H 0x56      // General purpose register

// Activity Detection Output Registers
#define REG_ACT_STATUS 0x58 // Activity Status Register
#define REG_TAP_STATUS 0x59 // Axis, direction, number of detected Tap

// Reset Register
#define REG_RESET 0x60 // Soft Reset Register

// Configuration macros
// General
#define WHO_AM_I 0x05    // Device identifier 0x05
#define REVISION_ID 0x7C // Device Revision ID

// Accelerometer Output Data Rate (ODR) options
// Normal 6DOF
#define ACC_ODR_7174_4HZ 0b0000
#define ACC_ODR_3587_2HZ 0b0001
#define ACC_ODR_1793_6HZ 0b0010
#define ACC_ODR_896_8HZ 0b0011
#define ACC_ODR_448_4HZ 0b0100
#define ACC_ODR_224_2HZ 0b0101
#define ACC_ODR_112_1HZ 0b0110
#define ACC_ODR_56_05HZ 0b0111
#define ACC_ODR_28_025HZ 0b1000
// Accel only
#define ACC_ODR_1000HZ 0b0011
#define ACC_ODR_500HZ 0b0100
#define ACC_ODR_250HZ 0b0101
#define ACC_ODR_125HZ 0b0110
#define ACC_ODR_62_5HZ 0b0111
#define ACC_ODR_31_25HZ 0b1000
// Low Power
#define ACC_ODR_128HZ 0b1100
#define ACC_ODR_21HZ 0b1101
#define ACC_ODR_11HZ 0b1110
#define ACC_ODR_3HZ 0b1111

// Accelerometer Full Scale options
#define ACCEL_RANGE_2G 0b00  // ±2g
#define ACCEL_RANGE_4G 0b01  // ±4g
#define ACCEL_RANGE_8G 0b10  // ±8g
#define ACCEL_RANGE_16G 0b11 // ±16g

// Gyroscope Output Data Rate (ODR) options
#define GYRO_ODR_7174_4HZ 0b0000 // 7174.4 Hz, Normal, 100 %
#define GYRO_ODR_3587_2HZ 0b0001 // 3587.2 Hz, Normal, 100 %
#define GYRO_ODR_1793_6HZ 0b0010 // 1793.6 Hz, Normal, 100 %
#define GYRO_ODR_896_8HZ 0b0011  // 896.8 Hz, Normal, 100 %
#define GYRO_ODR_448_4HZ 0b0100  // 448.4 Hz, Normal, 100 %
#define GYRO_ODR_224_2HZ 0b0101  // 224.2 Hz, Normal, 100 %
#define GYRO_ODR_112_1HZ 0b0110  // 112.1 Hz, Normal, 100 %
#define GYRO_ODR_56_05HZ 0b0111  // 56.05 Hz, Normal, 100 %
#define GYRO_ODR_28_025HZ 0b1000 // 28.025 Hz, Normal, 100 %
// ODR values 0x09 to 0x0F are not available

// Gyroscope Full Scale options
#define GYRO_RANGE_16DPS 0b000   // ±16 dps
#define GYRO_RANGE_32DPS 0b001   // ±32 dps
#define GYRO_RANGE_64DPS 0b010   // ±64 dps
#define GYRO_RANGE_128DPS 0b011  // ±128 dps
#define GYRO_RANGE_256DPS 0b100  // ±256 dps
#define GYRO_RANGE_512DPS 0b101  // ±512 dps
#define GYRO_RANGE_1024DPS 0b110 // ±1024 dps
#define GYRO_RANGE_2048DPS 0b111 // ±2048 dps

// Low-Pass Filter Mode options
#define LPF_MODE_2_66 0b00  // 2.66 % of ODR
#define LPF_MODE_3_63 0b01  // 3.63 % of ODR
#define LPF_MODE_5_39 0b10  // 5.39 % of ODR
#define LPF_MODE_13_37 0b11 // 13.37 % of ODR

// SOFTWARE_RESET
#define SOFTWARE_RESET 0xB0
#define SOFTWARE_RESET_OK 0x80

// MODES
#define MODE_POWER_ON_DEFAULT 0x00
#define MODE_LOW_POWER 0x01
#define MODE_POWER_DOWN 0x02
#define MODE_NORMAL_ACCEL_ONLY 0x03
#define MODE_LOW_POWER_ACCEL_ONLY 0x04
#define MODE_SNOOZE_GYRO 0x05
#define MODE_GYRO_ONLY 0x06
#define MODE_ACCEL_GYRO 0x07
#define MODE_ACCEL_SNOOZE_GYRO 0x08
#define MODE_SOFTWARE_RESET 0x09
#define MODE_NO_POWER 0x0A

// Accelerometer range mapping
const float ACCEL_RANGE_MAP[] = {
    2.0f,  // ACC_RANGE_2G (0b00)
    4.0f,  // ACC_RANGE_4G (0b01)
    8.0f,  // ACC_RANGE_8G (0b10)
    16.0f, // ACC_RANGE_16G (0b11)
};

// Gyroscope range mapping
const float GYRO_RANGE_MAP[] = {
    16.0f,   // GYRO_RANGE_16DPS (0b000)
    32.0f,   // GYRO_RANGE_32DPS (0b001)
    64.0f,   // GYRO_RANGE_64DPS (0b010)
    128.0f,  // GYRO_RANGE_128DPS (0b011)
    256.0f,  // GYRO_RANGE_256DPS (0b100)
    512.0f,  // GYRO_RANGE_512DPS (0b101)
    1024.0f, // GYRO_RANGE_1024DPS (0b110)
    2048.0f, // GYRO_RANGE_2048DPS (0b111)
};

bool QMI8658B::begin() {
  bool success;
  uint8_t chip_id;

  _i2c->begin();
  success = _i2c->read_byte_data(REG_WHO_AM_I, &chip_id);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.println(" chip ID read failed");
    return false;
  }
  if (chip_id != WHO_AM_I) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" chip ID expected 0x");
    Serial.print(WHO_AM_I, HEX);
    Serial.print(" got 0x");
    Serial.println(chip_id, HEX);
    return false;
  }

  set_ctrl7(-1, // SyncSample
            -1, // Data Ready
            0,  // Gyroscope Snooze Mode
            1,  // Enable gyro
            1   // Enable accel
  );
  delay(15);
  set_ctrl1(-1, // sim
            1,  // Address auto increment
            -1, // Big endian
            -1, // int2 Hi Z
            -1, // int1 Hi Z
            -1, // FIFO interrupt is mapped to INT2 pin
            -1  // Enable internal high-speed oscillator
  );
  set_ctrl2(0,               // Accel self-test disabled
            ACCEL_RANGE_4G,  // Range
            ACC_ODR_7174_4HZ // ODR
  );
  set_ctrl3(
      0,                 // Gyro self-test disabled
      GYRO_RANGE_512DPS, // Range
      GYRO_ODR_1793_6HZ  // High ODR (>1793.6Hz) causes significant data jitter
                         // due to I2C bottleneck and register tearing
  );
  return true;
}

bool QMI8658B::set_mode(uint8_t mode) {
  bool success;
  success = reset();
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.println(" mode reset failed");
    return false;
  }
  switch (mode) {
  case MODE_LOW_POWER:
    success = set_ctrl1(255, 255, 255, 255, 255, 255, false);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" enable sensor failed");
      return false;
    }
    success = set_ctrl7(255, 255, 255, false, false);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" disable accel gyro failed");
      return false;
    }
    break;
  case MODE_POWER_DOWN:
    success = set_ctrl1(255, 255, 255, 255, 255, 255, true);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" disable sensor failed");
      return false;
    }
    success = set_ctrl7(255, 255, 255, false, false);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" disable accel gyro failed");
      return false;
    }
    break;
  case MODE_NORMAL_ACCEL_ONLY:
    success = set_ctrl7(255, 255, 255, false, true);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" enable accel failed");
      return false;
    }
    break;
  case MODE_ACCEL_GYRO:
    success = set_ctrl7(255, 255, false, true, true);
    if (!success) {
      Serial.print("[Error] ");
      Serial.print(_chip_name);
      Serial.println(" enable accel gyro failed");
      return false;
    }
    break;
  default:
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.println(" unknown mode");
    return false;
  }
}
bool QMI8658B::reset() {
  bool success;
  success = _i2c->write_byte_data(REG_RESET, SOFTWARE_RESET);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" reset failed");
    return false;
  }
  delay(15);
  uint8_t status = 0;
  success = _i2c->read_byte_data(REG_dQY_L, &status);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" status read failed");
    return false;
  }
  return status == SOFTWARE_RESET_OK;
}
bool QMI8658B::set_ctrl1(uint8_t sim, uint8_t addr_ai, uint8_t be,
                         uint8_t int2_en, uint8_t int1_en, uint8_t fifo_int_sel,
                         uint8_t sensor_disable) {
  bool success;
  uint8_t ctrl1;
  success = _i2c->read_byte_data(REG_CTRL1, &ctrl1);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl1 read failed");
    return false;
  }
  if (sim != 255) {
    ctrl1 &= ~(0b1 << 7);      // Clear sim bits
    ctrl1 |= (sim & 0b1) << 7; // Set new sim
  }
  if (addr_ai != 255) {
    ctrl1 &= ~(0b1 << 6);          // Clear addr_ai bits
    ctrl1 |= (addr_ai & 0b1) << 6; // Set new addr_ai
  }
  if (be != 255) {
    ctrl1 &= ~(0b1 << 5);     // Clear be bits
    ctrl1 |= (be & 0b1) << 5; // Set new be
  }
  if (int2_en != 255) {
    ctrl1 &= ~(0b1 << 4);          // Clear int2_en bits
    ctrl1 |= (int2_en & 0b1) << 4; // Set new int2_en
  }
  if (int1_en != 255) {
    ctrl1 &= ~(0b1 << 3);          // Clear int1_en bits
    ctrl1 |= (int1_en & 0b1) << 3; // Set new int1_en
  }
  if (fifo_int_sel != 255) {
    ctrl1 &= ~(0b1 << 2);               // Clear fifo_int_sel bits
    ctrl1 |= (fifo_int_sel & 0b1) << 2; // Set new fifo_int_sel
  }
  if (sensor_disable != 255) {
    ctrl1 &= ~(0b1 << 0);                 // Clear sensor_disable bits
    ctrl1 |= (sensor_disable & 0b1) << 0; // Set new sensor_disable
  }
  success = _i2c->write_byte_data(REG_CTRL1, ctrl1);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl1 write failed");
    return false;
  }
  return true;
}
bool QMI8658B::set_ctrl2(uint8_t acc_self_test, uint8_t acc_fs,
                         uint8_t acc_odr) {
  bool success;
  uint8_t ctrl2;
  success = _i2c->read_byte_data(REG_CTRL2, &ctrl2);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl2 read failed");
    return false;
  }

  if (acc_odr != 255) {
    ctrl2 &= ~(0b1111 << 0);          // Clear acc_odr bits
    ctrl2 |= (acc_odr & 0b1111) << 0; // Set new acc_odr
  }
  if (acc_fs != 255) {
    ctrl2 &= ~(0b111 << 4);         // Clear acc_fs bits
    ctrl2 |= (acc_fs & 0b111) << 4; // Set new acc_fs
    _accel_range = ACCEL_RANGE_MAP[acc_fs];
  }
  if (acc_self_test != 255) {
    ctrl2 &= ~(0b1 << 7);                // Clear acc_self_test bits
    ctrl2 |= (acc_self_test & 0b1) << 7; // Set new acc_self_test
  }
  success = _i2c->write_byte_data(REG_CTRL2, ctrl2);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl2 write failed");
    return false;
  }
  return true;
}
bool QMI8658B::set_ctrl3(uint8_t gyro_self_test, uint8_t gyro_fs,
                         uint8_t gyro_odr) {
  bool success;
  uint8_t ctrl3;
  success = _i2c->read_byte_data(REG_CTRL3, &ctrl3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl3 read failed");
    return false;
  }
  if (gyro_self_test != 255) {
    ctrl3 &= ~(0b1 << 7);                 // Clear gyro_self_test bits
    ctrl3 |= (gyro_self_test & 0b1) << 7; // Set new gyro_self_test
  }
  if (gyro_fs != 255) {
    ctrl3 &= ~(0b111 << 4);          // Clear gyro_fs bits
    ctrl3 |= (gyro_fs & 0b111) << 4; // Set new gyro_fs
    _gyro_range = GYRO_RANGE_MAP[gyro_fs];
  }
  if (gyro_odr != 255) {
    ctrl3 &= ~(0b1111 << 0);           // Clear gyro_odr bits
    ctrl3 |= (gyro_odr & 0b1111) << 0; // Set new gyro_odr
  }
  success = _i2c->write_byte_data(REG_CTRL3, ctrl3);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl3 write failed");
    return false;
  }
  return true;
}
bool QMI8658B::set_ctrl5(uint8_t gyro_lpf_mode, uint8_t gyro_lpf_en,
                         uint8_t accel_lpf_mode, uint8_t accel_lpf_en) {
  bool success;
  uint8_t ctrl5;
  success = _i2c->read_byte_data(REG_CTRL5, &ctrl5);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl5 read failed");
    return false;
  }
  if (gyro_lpf_mode != 255) {
    ctrl5 &= ~(0b11 << 5);                // Clear gyro_lpf_mode bits
    ctrl5 |= (gyro_lpf_mode & 0b11) << 5; // Set new gyro_lpf_mode
  }
  if (gyro_lpf_en != 255) {
    ctrl5 &= ~(0b1 << 4);              // Clear gyro_lpf_en bits
    ctrl5 |= (gyro_lpf_en & 0b1) << 4; // Set new gyro_lpf_en
  }
  if (accel_lpf_mode != 255) {
    ctrl5 &= ~(0b11 << 1);                 // Clear accel_lpf_mode bits
    ctrl5 |= (accel_lpf_mode & 0b11) << 1; // Set new accel_lpf_mode
  }
  if (accel_lpf_en != 255) {
    ctrl5 &= ~(0b1 << 0);               // Clear accel_lpf_en bits
    ctrl5 |= (accel_lpf_en & 0b1) << 0; // Set new accel_lpf_en
  }
  success = _i2c->write_byte_data(REG_CTRL5, ctrl5);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl5 write failed");
    return false;
  }
  return true;
}

bool QMI8658B::set_ctrl7(uint8_t sync_sample, uint8_t drdy_dis,
                         uint8_t gyro_snooze, uint8_t gyro_en,
                         uint8_t accel_en) {
  bool success;
  uint8_t ctrl7;
  success = _i2c->read_byte_data(REG_CTRL7, &ctrl7);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl7 read failed");
    return false;
  }
  if (sync_sample != 255) {
    ctrl7 &= ~(0b1 << 7);              // Clear sync_sample bits
    ctrl7 |= (sync_sample & 0b1) << 7; // Set new sync_sample
  }
  if (drdy_dis != 255) {
    ctrl7 &= ~(0b1 << 5);           // Clear drdy_dis bits
    ctrl7 |= (drdy_dis & 0b1) << 5; // Set new drdy_dis
  }
  if (gyro_snooze != 255) {
    ctrl7 &= ~(0b1 << 4);              // Clear gyro_snooze bits
    ctrl7 |= (gyro_snooze & 0b1) << 4; // Set new gyro_snooze
  }
  if (gyro_en != 255) {
    ctrl7 &= ~(0b1 << 1);          // Clear gyro_en bits
    ctrl7 |= (gyro_en & 0b1) << 1; // Set new gyro_en
  }
  if (accel_en != 255) {
    ctrl7 &= ~(0b1 << 0);           // Clear accel_en bits
    ctrl7 |= (accel_en & 0b1) << 0; // Set new accel_en
  }
  success = _i2c->write_byte_data(REG_CTRL7, ctrl7);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl7 write failed");
    return false;
  }
}

bool QMI8658B::set_ctrl8(uint8_t ctrl9_handshake_type, uint8_t activity_int_sel,
                         uint8_t sig_motion_en, uint8_t no_motion_en,
                         uint8_t any_motion_en, uint8_t tap_en) {
  bool success;
  uint8_t ctrl8;
  success = _i2c->read_byte_data(REG_CTRL8, &ctrl8);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl8 read failed");
    return false;
  }
  if (ctrl9_handshake_type != 255) {
    ctrl8 &= ~(0b1 << 7); // Clear ctrl9_handshake_type bits
    ctrl8 |= (ctrl9_handshake_type & 0b1) << 7; // Set new ctrl9_handshake_type
  }
  if (activity_int_sel != 255) {
    ctrl8 &= ~(0b1 << 6);                   // Clear activity_int_sel bits
    ctrl8 |= (activity_int_sel & 0b1) << 6; // Set new activity_int_sel
  }
  if (sig_motion_en != 255) {
    ctrl8 &= ~(0b1 << 3);                // Clear sig_motion_en bits
    ctrl8 |= (sig_motion_en & 0b1) << 3; // Set new sig_motion_en
  }
  if (no_motion_en != 255) {
    ctrl8 &= ~(0b1 << 2);               // Clear no_motion_en bits
    ctrl8 |= (no_motion_en & 0b1) << 2; // Set new no_motion_en
  }
  if (any_motion_en != 255) {
    ctrl8 &= ~(0b1 << 1);                // Clear any_motion_en bits
    ctrl8 |= (any_motion_en & 0b1) << 1; // Set new any_motion_en
  }
  if (tap_en != 255) {
    ctrl8 &= ~(0b1 << 0);         // Clear tap_en bits
    ctrl8 |= (tap_en & 0b1) << 0; // Set new tap_en
  }
  success = _i2c->write_byte_data(REG_CTRL8, ctrl8);
  if (!success) {
    Serial.print("[Error] ");
    Serial.print(_chip_name);
    Serial.print(" ctrl8 write failed");
    return false;
  }
}

bool QMI8658B::set_accel_range(uint8_t range) {
  return set_ctrl2(255, range, 255);
}

bool QMI8658B::set_gyro_range(uint8_t range) {
  return set_ctrl3(255, range, 255);
}

bool QMI8658B::_read_temperature() {
  uint16_t temp_data;
  bool success = _i2c->read_word_data(REG_TEMP_L, &temp_data);
  if (!success) {
    return false;
  }
  int16_t temp_raw = (int16_t)(temp_data);
  _temperature = temp_raw / 256.0;

  return true;
}

bool QMI8658B::_read_accel() {
  uint8_t accel_data[6];

  if (!_i2c->read_i2c_block_data(REG_AX_L, accel_data, 6)) {
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

bool QMI8658B::_read_gyro() {
  uint8_t gyro_data[6];

  bool success = _i2c->read_i2c_block_data(REG_GX_L, gyro_data, 6);
  if (!success) {
    return false;
  }

  // Convert raw data to 16-bit signed values (little-endian format)
  int16_t x_raw = (gyro_data[1] << 8) | gyro_data[0];
  int16_t y_raw = (gyro_data[3] << 8) | gyro_data[2];
  int16_t z_raw = (gyro_data[5] << 8) | gyro_data[4];

  // Convert to dps first
  float x_dps = mapping(static_cast<float>(x_raw), -32768.0f, 32767.0f,
                        -_gyro_range, _gyro_range);
  float y_dps = mapping(static_cast<float>(y_raw), -32768.0f, 32767.0f,
                        -_gyro_range, _gyro_range);
  float z_dps = mapping(static_cast<float>(z_raw), -32768.0f, 32767.0f,
                        -_gyro_range, _gyro_range);

  _gyro_data.x = x_dps;
  _gyro_data.y = y_dps;
  _gyro_data.z = z_dps;

  return true;
}

bool QMI8658B::_read_all() {
  uint8_t all_data[14]; // 6 bytes accel + 6 bytes gyro + 2 bytes temp

  bool success = _i2c->read_i2c_block_data(REG_TEMP_L, all_data, 14);
  if (!success) {
    Serial.println("[Error] Failed to read all data");
    return false;
  }

  // Read accelerometer data (little-endian format)
  int16_t ax_raw = ((uint16_t)all_data[3] << 8) | all_data[2];
  int16_t ay_raw = ((uint16_t)all_data[5] << 8) | all_data[4];
  int16_t az_raw = ((uint16_t)all_data[7] << 8) | all_data[6];

  // Convert to g
  float ax_g = mapping(static_cast<float>(ax_raw), -32768.0f, 32767.0f,
                       -_accel_range, _accel_range);
  float ay_g = mapping(static_cast<float>(ay_raw), -32768.0f, 32767.0f,
                       -_accel_range, _accel_range);
  float az_g = mapping(static_cast<float>(az_raw), -32768.0f, 32767.0f,
                       -_accel_range, _accel_range);

  // Set accelerometer data
  _accel_data.x = ax_g;
  _accel_data.y = ay_g;
  _accel_data.z = az_g;

  // Read gyroscope data (little-endian format)
  int16_t gx_raw = ((uint16_t)all_data[9] << 8) | all_data[8];
  int16_t gy_raw = ((uint16_t)all_data[11] << 8) | all_data[10];
  int16_t gz_raw = ((uint16_t)all_data[13] << 8) | all_data[12];

  // Convert to dps
  float gx_dps = mapping(static_cast<float>(gx_raw), -32768.0f, 32767.0f,
                         -_gyro_range, _gyro_range);
  float gy_dps = mapping(static_cast<float>(gy_raw), -32768.0f, 32767.0f,
                         -_gyro_range, _gyro_range);
  float gz_dps = mapping(static_cast<float>(gz_raw), -32768.0f, 32767.0f,
                         -_gyro_range, _gyro_range);

  // Set gyroscope data
  _gyro_data.x = gx_dps;
  _gyro_data.y = gy_dps;
  _gyro_data.z = gz_dps;

  // Read temperature data (big-endian format)
  int16_t temp_raw = (all_data[1] << 8) | all_data[0];
  _temperature = temp_raw / 256.0;

  return true; // Data read successfully
}
