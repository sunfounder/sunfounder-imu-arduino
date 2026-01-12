#include "SunFounder_IMU.hpp"
#include "calibration_data.h"

#include "Wire.h"

SunFounder_IMU imu(&Wire1);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  imu.begin();
  imu.set_accel_bias(ACCEL_BIAS);
  imu.set_accel_scale(ACCEL_SCALE);
  imu.set_gyro_bias(GYRO_BIAS);
  imu.set_gyro_scale(GYRO_SCALE);
  imu.set_magnetometer_bias(MAG_BIAS);
  imu.set_magnetometer_scale(MAG_SCALE);

  // Change these if necessary
  // imu.set_gravity(GRAVITY);
  // imu.set_barometer_pressure_offset(BARO_PRESSURE_OFFSET);
  // imu.set_barometer_sealevel_pressure(BARO_SEALEVEL_PRESSURE);
}

void loop() {
  imu.read();
  Vector3f accel = imu.get_accel();
  Vector3f gyro = imu.get_gyro();
  Vector3f magnetometer = imu.get_magnetometer();
  float temperature = imu.get_temperature();
  float azimuth = imu.get_azimuth();
  float pressure = imu.get_pressure();
  float altitude = imu.get_altitude();

  Serial.print("Accel: ");
  Serial.print(accel.x);
  Serial.print(", ");
  Serial.print(accel.y);
  Serial.print(", ");
  Serial.println(accel.z);
  Serial.print("Gyro: ");
  Serial.print(gyro.x);
  Serial.print(", ");
  Serial.print(gyro.y);
  Serial.print(", ");
  Serial.println(gyro.z);
  Serial.print("Magnetometer: ");
  Serial.print(magnetometer.x);
  Serial.print(", ");
  Serial.print(magnetometer.y);
  Serial.print(", ");
  Serial.println(magnetometer.z);
  Serial.print("Azimuth: ");
  Serial.println(azimuth);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Altitude: ");
  Serial.println(altitude);

  delay(1000);
}
