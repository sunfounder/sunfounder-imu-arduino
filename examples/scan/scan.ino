#include "SunFounder_IMU.hpp"

#include "Wire.h"

// For Arduino UNO R4, use Wire1 as I2C bus
// #define Wire Wire1
// For ESP32, you may need to set the I2C pins
#define SDA 43
#define SCL 44

SunFounder_IMU imu(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  // For ESP32, you may need to set the I2C pins
  Wire.begin(SDA, SCL);
  // Wire.begin();
}

void loop() {
  Serial.println("IMU Init");
  imu.begin();

  Serial.println("IMU Inited");
  if (imu.is_motion_sensor_found()) {
    Serial.println("Motion sensor found: " + imu.get_motion_sensor_name());
  } else {
    Serial.println("No motion sensor found");
  }
  if (imu.is_magnetometer_found()) {
    Serial.println("Magnetometer found: " + imu.get_magnetometer_name());
  } else {
    Serial.println("No magnetometer found");
  }
  if (imu.is_barometer_found()) {
    Serial.println("Barometer found: " + imu.get_barometer_name());
  } else {
    Serial.println("No barometer found");
  }
  delay(5000);
}
