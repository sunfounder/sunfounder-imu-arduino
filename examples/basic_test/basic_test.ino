#include <Wire.h>
#include <SunFounder_IMU.hpp>

SunFounder_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Serial.println("SunFounder IMU Basic Test");
    Serial.println("Initializing...");

    // Initialize the IMU
    if (imu.begin()) {
        Serial.println("IMU initialized successfully!");
        Serial.println("Reading data every 1000ms...");
    } else {
        Serial.println("Failed to initialize IMU!");
        while (1) {
            delay(1000);
        }
    }
}

void loop() {
    AccelData accel;
    GyroData gyro;
    float temperature;

    // Read acceleration, gyro, and temperature
    if (imu.read_accel(accel) && imu.read_gyro(gyro) && imu.read_temperature(temperature)) {
        Serial.print("Accel (m/s²): ");
        Serial.print(accel.x, 2);
        Serial.print(", ");
        Serial.print(accel.y, 2);
        Serial.print(", ");
        Serial.print(accel.z, 2);

        Serial.print(" | Gyro (dps): ");
        Serial.print(gyro.x, 2);
        Serial.print(", ");
        Serial.print(gyro.y, 2);
        Serial.print(", ");
        Serial.print(gyro.z, 2);

        Serial.print(" | Temp (°C): ");
        Serial.print(temperature, 2);
        Serial.println();
    } else {
        Serial.println("Failed to read data!");
    }

    delay(1000);
}
