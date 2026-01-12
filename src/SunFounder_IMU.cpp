#include "SunFounder_IMU.hpp"

#include "qmc6310.hpp"
#include "sh3001.hpp"
#include "spl06_001.hpp"

SunFounder_IMU::SunFounder_IMU(TwoWire *wire) : _wire(wire) {}

bool SunFounder_IMU::begin() {
  uint8_t addresses[128];
  uint8_t address;
  uint8_t count;
  bool success = true;

  _wire->begin();
  for (uint8_t i = 0; i < 5; i++) {
    count = I2C::scan(_wire, addresses);
    if (count != 0) {
      break;
    }
    delay(1000);
  }
  motion_sensor = get_motion_sensor(addresses, count);
  if (motion_sensor == nullptr) {
    Serial.println("[Warning] No motion sensor found");
  } else {
    success = motion_sensor->begin();
    if (!success) {
      Serial.println("[Warning] Failed to begin motion sensor");
      motion_sensor = nullptr;
    }
  }

  magnetometer = get_magnetometer(addresses, count);
  if (magnetometer == nullptr) {
    Serial.println("[Warning] No magnetometer found");
  } else {
    success = magnetometer->begin();
    if (!success) {
      Serial.println("[Warning] Failed to begin magnetometer");
      magnetometer = nullptr;
    }
  }

  barometer = get_barometer(addresses, count);
  if (barometer == nullptr) {
    Serial.println("[Warning] No barometer found");
  } else {
    success = barometer->begin();
    if (!success) {
      Serial.println("[Warning] Failed to begin barometer");
      barometer = nullptr;
    }
  }
}

MotionSensor *SunFounder_IMU::get_motion_sensor(uint8_t *addresses,
                                                uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (addresses[i] == 0x36 || addresses[i] == 0x37) {
      return new SH3001(_wire, addresses[i]);
    }
  }
  return nullptr;
}

Magnetometer *SunFounder_IMU::get_magnetometer(uint8_t *addresses,
                                               uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (addresses[i] == 0x1C) {
      return new QMC6310(_wire, addresses[i]);
    }
  }
  return nullptr;
}

Barometer *SunFounder_IMU::get_barometer(uint8_t *addresses, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (addresses[i] == 0x76 || addresses[i] == 0x77) {
      return new SPL06_001(_wire, addresses[i]);
    }
  }
  return nullptr;
}

bool SunFounder_IMU::read(bool raw) {
  bool status = true;
  if (motion_sensor != nullptr) {
    status &= motion_sensor->read(raw);
  }
  if (magnetometer != nullptr) {
    status &= magnetometer->read(raw);
  }
  if (barometer != nullptr) {
    status &= barometer->read(raw);
  }
  return status;
}

Vector3f SunFounder_IMU::get_accel() {
  if (motion_sensor == nullptr) {
    Serial.println("[Warning] No motion sensor found");
    return Vector3f(0, 0, 0);
  }
  return motion_sensor->get_accel_data();
}

Vector3f SunFounder_IMU::get_gyro() {
  if (motion_sensor == nullptr) {
    Serial.println("[Warning] No motion sensor found");
    return Vector3f(0, 0, 0);
  }
  return motion_sensor->get_gyro_data();
}

Vector3f SunFounder_IMU::get_magnetometer() {
  if (magnetometer == nullptr) {
    Serial.println("[Warning] No magnetometer found");
    return Vector3f(0, 0, 0);
  }
  return magnetometer->get_data();
}

float SunFounder_IMU::get_temperature() {
  if (barometer != nullptr) {
    return barometer->get_temperature();
  } else if (motion_sensor != nullptr) {
    return motion_sensor->get_temperature();
  } else {
    Serial.println("[Warning] No temperature sensor found");
    return 0.0f;
  }
}

float SunFounder_IMU::get_azimuth() {
  if (magnetometer == nullptr) {
    Serial.println("[Warning] No magnetometer found");
    return 0.0f;
  }
  return magnetometer->get_azimuth();
}

float SunFounder_IMU::get_pressure() {
  if (barometer == nullptr) {
    Serial.println("[Warning] No barometer found");
    return 0.0f;
  }
  return barometer->get_pressure();
}

float SunFounder_IMU::get_altitude() {
  if (barometer == nullptr) {
    Serial.println("[Warning] No barometer found");
    return 0.0f;
  }
  return barometer->get_altitude();
}

void SunFounder_IMU::set_accel_bias(const float bias[3]) {
  if (motion_sensor != nullptr) {
    motion_sensor->set_accel_bias(bias);
  }
}

void SunFounder_IMU::set_accel_scale(const float scale[3]) {
  if (motion_sensor != nullptr) {
    motion_sensor->set_accel_scale(scale);
  }
}

void SunFounder_IMU::set_gyro_bias(const float bias[3]) {
  if (motion_sensor != nullptr) {
    motion_sensor->set_gyro_bias(bias);
  }
}

void SunFounder_IMU::set_gyro_scale(const float scale[3]) {
  if (motion_sensor != nullptr) {
    motion_sensor->set_gyro_scale(scale);
  }
}

void SunFounder_IMU::set_magnetometer_bias(const float bias[3]) {
  if (magnetometer != nullptr) {
    magnetometer->set_bias(bias);
  }
}

void SunFounder_IMU::set_magnetometer_scale(const float scale[3]) {
  if (magnetometer != nullptr) {
    magnetometer->set_scale(scale);
  }
}

void SunFounder_IMU::set_barometer_pressure_offset(const float offset) {
  if (barometer != nullptr) {
    barometer->set_offset(offset);
  }
}

void SunFounder_IMU::set_barometer_sealevel_pressure(
    const float sealevel_pressure) {
  if (barometer != nullptr) {
    barometer->set_sealevel_pressure(sealevel_pressure);
  }
}

void SunFounder_IMU::set_orientation(uint8_t up, uint8_t front) {
  if (magnetometer != nullptr) {
    magnetometer->set_orientation(up, front);
  }
}