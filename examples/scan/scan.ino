#include "SunFounder_IMU.hpp"

#include "Wire.h"

SunFounder_IMU imu(&Wire1);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  imu.begin();

  if (imu.is_motion_sensor_found()) {
    Serial.println("Motion sensor found: " + imu.get_motion_sensor_name());
  }
  if (imu.is_magnetometer_found()) {
    Serial.println("Magnetometer found: " + imu.get_magnetometer_name());
  }
  if (imu.is_barometer_found()) {
    Serial.println("Barometer found: " + imu.get_barometer_name());
  }
}

void loop() {}
