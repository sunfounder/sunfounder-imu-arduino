#include <Wire.h>
#include <SunFounder_IMU.hpp>

// For Arduino Uno R4, you need to use Wire1 instead of Wire
SunFounder_IMU imu = SunFounder_IMU(&Wire1);

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Serial.println("SunFounder IMU Calibration Test");
    Serial.println("Initializing...");
    
    // Initialize I2C buses
    Wire.begin();
    Wire1.begin();
    
    // Initialize the IMU
    if (imu.begin()) {
        Serial.println("IMU initialized successfully!");
        
        // Check which sensors are available
        Serial.print("Accelerometer/Gyroscope: ");
        Serial.println(imu.is_accel_gyro_available() ? "Available" : "Not available");
        Serial.print("  Chip: ");
        Serial.println(imu.get_accel_gyro_chip_name());
        
        Serial.print("Magnetometer: ");
        Serial.println(imu.is_magnetometer_available() ? "Available" : "Not available");
        Serial.print("  Chip: ");
        Serial.println(imu.get_magnetometer_chip_name());
        
        Serial.print("Barometer/Altimeter: ");
        Serial.println(imu.is_barometer_available() ? "Available" : "Not available");
        Serial.print("  Chip: ");
        Serial.println(imu.get_barometer_chip_name());
        
        // Calibration menu
        Serial.println("\n--- Calibration Menu ---");
        Serial.println("1. Calibrate Accelerometer/Gyroscope");
        Serial.println("2. Calibrate Magnetometer");
        Serial.println("3. Calibrate All Sensors");
        Serial.println("4. Skip Calibration");
        Serial.println("Please enter your choice (1-4):");
        
        // Wait for user input
        while (!Serial.available()) {
            delay(100);
        }
        
        char choice = Serial.read();
        Serial.print("\nSelected: ");
        Serial.println(choice);
        
        switch (choice) {
            case '1':
                // Calibrate Accelerometer/Gyroscope
                imu.calibrate_accel_gyro();
                break;
            case '2':
                // Calibrate magnetometer
                imu.calibrate_magnetometer();
                break;
            case '3':
                // Calibrate all sensors
                imu.calibrate_all();
                break;
            case '4':
                // Skip calibration
                Serial.println("Calibration skipped.");
                break;
            default:
                Serial.println("Invalid choice. Calibration skipped.");
                break;
        }
        
        Serial.println("\nCalibration process completed. Starting data reading...");
    } else {
        Serial.println("Failed to initialize IMU!");
        while (1) {
            delay(1000);
        }
    }
}

void loop() {
    IMUData imu_data;

    // Read all sensor data using the simplified API
    if (imu.read(imu_data)) {
        Serial.print("\nAcceleration (m/s²): ");
        Serial.print(imu_data.accel.x, 3);
        Serial.print(", ");
        Serial.print(imu_data.accel.y, 3);
        Serial.print(", ");
        Serial.print(imu_data.accel.z, 3);

        Serial.print("\nGyroscope (dps): ");
        Serial.print(imu_data.gyro.x, 3);
        Serial.print(", ");
        Serial.print(imu_data.gyro.y, 3);
        Serial.print(", ");
        Serial.print(imu_data.gyro.z, 3);

        Serial.print("\nMagnetic (Gauss): ");
        Serial.print(imu_data.mag.x, 3);
        Serial.print(", ");
        Serial.print(imu_data.mag.y, 3);
        Serial.print(", ");
        Serial.print(imu_data.mag.z, 3);

        Serial.print("\nAzimuth (°): ");
        Serial.print(imu_data.azimuth, 2);

        Serial.print("\nTemperature (°C): ");
        Serial.print(imu_data.temperature, 2);

        Serial.print("\nPressure (hPa): ");
        Serial.print(imu_data.pressure, 2);

        Serial.print("\nAltitude (m): ");
        Serial.print(imu_data.altitude, 2);
        Serial.println();
    } else {
        Serial.println("Failed to read sensor data!");
    }

    delay(1000);
}