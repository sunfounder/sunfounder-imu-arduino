#include "SunFounder_IMU.hpp"
#include "Wire.h"

SunFounder_IMU imu(&Wire1);

const char *FACES[] = {"Z face up",   "Z face down", "X face up",
                       "X face down", "Y face up",   "Y face down"};

const int FACES_COUNT = 6;
const int TIMES = 50;
float accel_datas[TIMES][3];
float gyro_datas[TIMES][3];
float magnetometer_datas[TIMES][3];
float accel_means[FACES_COUNT][3];
float gyro_means[FACES_COUNT][3];
float magnetometer_means[FACES_COUNT][3];

float get_max_from_list(float *list, int size) {
  float max = list[0];
  for (int i = 1; i < size; i++) {
    if (list[i] > max) {
      max = list[i];
    }
  }
  return max;
}

float get_min_from_list(float *list, int size) {
  float min = list[0];
  for (int i = 1; i < size; i++) {
    if (list[i] < min) {
      min = list[i];
    }
  }
  return min;
}

void calculate_calibration_data(float *bias, float *scale,
                                float means_data[][3]) {
  float x_dats[6] = {means_data[0][0], means_data[1][0], means_data[2][0],
                     means_data[3][0], means_data[4][0], means_data[5][0]};
  float y_dats[6] = {means_data[0][1], means_data[1][1], means_data[2][1],
                     means_data[3][1], means_data[4][1], means_data[5][1]};
  float z_dats[6] = {means_data[0][2], means_data[1][2], means_data[2][2],
                     means_data[3][2], means_data[4][2], means_data[5][2]};

  float x_max = get_max_from_list(x_dats, 6);
  float x_min = get_min_from_list(x_dats, 6);
  float y_max = get_max_from_list(y_dats, 6);
  float y_min = get_min_from_list(y_dats, 6);
  float z_max = get_max_from_list(z_dats, 6);
  float z_min = get_min_from_list(z_dats, 6);

  bias[0] = (x_max + x_min) / 2.0f;
  bias[1] = (y_max + y_min) / 2.0f;
  bias[2] = (z_max + z_min) / 2.0f;

  // Check if the range is too small to avoid division by zero
  const float MIN_RANGE = 0.001f;       // Minimum range threshold
  const float MAX_SCALE_VALUE = 100.0f; // Maximum scale value limit

  // Calculate ranges and scale factors
  float x_range = max(fabs(x_max - x_min), MIN_RANGE);
  float y_range = max(fabs(y_max - y_min), MIN_RANGE);
  float z_range = max(fabs(z_max - z_min), MIN_RANGE);

  // Calculate scale factors
  scale[0] = 2.0f / x_range;
  scale[1] = 2.0f / y_range;
  scale[2] = 2.0f / z_range;

  // Constrain scale values to avoid overflow
  scale[0] = constrain(scale[0], -MAX_SCALE_VALUE, MAX_SCALE_VALUE);
  scale[1] = constrain(scale[1], -MAX_SCALE_VALUE, MAX_SCALE_VALUE);
  scale[2] = constrain(scale[2], -MAX_SCALE_VALUE, MAX_SCALE_VALUE);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  imu.begin();
  Serial.println("IMU initialized successfully.");
  Serial.println("Starting calibration process...");

  // Calibration process for each face
  for (int i = 0; i < FACES_COUNT; i++) {
    Serial.println("\n\n=================");
    Serial.print("*  ");
    Serial.println(FACES[i]);
    Serial.println("=================");
    Serial.print("Set the device down flat ");
    Serial.print(FACES[i]);
    Serial.println(", and DO NOT move it until this process is complete.");
    Serial.println("\nWhen you are ready, send any character to continue.");

    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    while (Serial.available()) {
      Serial.read(); // Clear the buffer
    }

    // Read data for this face
    Serial.print("Reading data: ");
    for (int j = 0; j < TIMES; j++) {
      imu.read(true);
      accel_datas[j][0] = imu.get_accel().x;
      accel_datas[j][1] = imu.get_accel().y;
      accel_datas[j][2] = imu.get_accel().z;
      gyro_datas[j][0] = imu.get_gyro().x;
      gyro_datas[j][1] = imu.get_gyro().y;
      gyro_datas[j][2] = imu.get_gyro().z;
      magnetometer_datas[j][0] = imu.get_magnetometer().x;
      magnetometer_datas[j][1] = imu.get_magnetometer().y;
      magnetometer_datas[j][2] = imu.get_magnetometer().z;

      // Print progress
      Serial.print("=");
      // Serial.print("Accel: ");
      // Serial.print(accel_datas[j][0]);
      // Serial.print(" ");
      // Serial.print(accel_datas[j][1]);
      // Serial.print(" ");
      // Serial.print(accel_datas[j][2]);
      // Serial.print(" | Gyro: ");
      // Serial.print(gyro_datas[j][0]);
      // Serial.print(" ");
      // Serial.print(gyro_datas[j][1]);
      // Serial.print(" ");
      // Serial.print(gyro_datas[j][2]);
      // Serial.print(" | Mag: ");
      // Serial.print(magnetometer_datas[j][0]);
      // Serial.print(" ");
      // Serial.print(magnetometer_datas[j][1]);
      // Serial.print(" ");
      // Serial.println(magnetometer_datas[j][2]);
      delay(10);
    }

    // Calculate means for this face
    for (int j = 0; j < 3; j++) {
      accel_means[i][j] = 0;
      gyro_means[i][j] = 0;
      magnetometer_means[i][j] = 0;
      double accel_mean_temp = 0;
      double gyro_mean_temp = 0;
      double magnetometer_mean_temp = 0;
      for (int k = 0; k < TIMES; k++) {
        accel_mean_temp += accel_datas[k][j];
        gyro_mean_temp += gyro_datas[k][j];
        magnetometer_mean_temp += magnetometer_datas[k][j];
      }
      accel_means[i][j] = accel_mean_temp / TIMES;
      gyro_means[i][j] = gyro_mean_temp / TIMES;
      magnetometer_means[i][j] = magnetometer_mean_temp / TIMES;
      // Print means
      // Serial.print("  ");
      // Serial.print(accel_means[i][j]);
      // Serial.print("  ");
      // Serial.print(gyro_means[i][j]);
      // Serial.print("  ");
      // Serial.println(magnetometer_means[i][j]);
    }
  }

  // Calculate calibration data
  float accel_bias[3];
  float accel_scale[3];
  float gyro_bias[3];
  float gyro_scale[3];
  float magnetometer_bias[3];
  float magnetometer_scale[3];
  calculate_calibration_data(accel_bias, accel_scale, accel_means);
  calculate_calibration_data(gyro_bias, gyro_scale, gyro_means);
  calculate_calibration_data(magnetometer_bias, magnetometer_scale,
                             magnetometer_means);

  Serial.println("\nCalibration complete!");
  Serial.println(
      "Copy the following calibration data to your calibration_data.h:");
  Serial.print("const float ACCEL_BIAS[3] = {");
  Serial.print(accel_bias[0]);
  Serial.print(", ");
  Serial.print(accel_bias[1]);
  Serial.print(", ");
  Serial.print(accel_bias[2]);
  Serial.println("};");

  Serial.print("const float ACCEL_SCALE[3] = {");
  Serial.print(accel_scale[0]);
  Serial.print(", ");
  Serial.print(accel_scale[1]);
  Serial.print(", ");
  Serial.print(accel_scale[2]);
  Serial.println("};");

  Serial.print("const float GYRO_BIAS[3] = {");
  Serial.print(gyro_bias[0]);
  Serial.print(", ");
  Serial.print(gyro_bias[1]);
  Serial.print(", ");
  Serial.print(gyro_bias[2]);
  Serial.println("};");

  Serial.print("const float GYRO_SCALE[3] = {");
  Serial.print(gyro_scale[0]);
  Serial.print(", ");
  Serial.print(gyro_scale[1]);
  Serial.print(", ");
  Serial.print(gyro_scale[2]);
  Serial.println("};");

  Serial.print("const float MAG_BIAS[3] = {");
  Serial.print(magnetometer_bias[0]);
  Serial.print(", ");
  Serial.print(magnetometer_bias[1]);
  Serial.print(", ");
  Serial.print(magnetometer_bias[2]);
  Serial.println("};");

  Serial.print("const float MAG_SCALE[3] = {");
  Serial.print(magnetometer_scale[0]);
  Serial.print(", ");
  Serial.print(magnetometer_scale[1]);
  Serial.print(", ");
  Serial.print(magnetometer_scale[2]);
  Serial.println("};");
}

void loop() {
  // Nothing to do after calibration
  delay(1000);
}