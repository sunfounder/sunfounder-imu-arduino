#pragma once

#include <Arduino.h>
#include "sh3001.hpp"
#include "qmc6310.hpp"
#include "spl06_001.hpp"

// Data structure to hold all IMU data
struct IMUData {
    AccelData accel;
    GyroData gyro;
    MagData mag;
    float temperature;
    float pressure;
    float altitude;
    float azimuth;
};

class SunFounder_IMU {
public:
    SunFounder_IMU(TwoWire *wire = &Wire);
    ~SunFounder_IMU();

    // Initialization methods
    bool begin();
    bool init_sh3001();
    bool init_qmc6310();
    bool init_spl06_001();

    // Sensor access methods
    SH3001* get_sh3001();
    QMC6310* get_qmc6310();
    SPL06_001* get_spl06_001();

    // Data reading methods
    bool read_all(
        AccelData &accel,
        GyroData &gyro,
        MagData &mag,
        float &temperature,
        float &pressure,
        float &altitude,
        float &azimuth
    );
    
    bool read(IMUData &data);

    bool read_accel(AccelData &accel);
    bool read_gyro(GyroData &gyro);
    bool read_mag(MagData &mag);
    bool read_mag_with_azimuth(MagData &mag, float &azimuth);
    bool read_pressure(float &pressure);
    bool read_temperature(float &temperature);
    bool read_altitude(float &altitude);

    // Calibration methods
    void calibrate_gyro(uint16_t times = 100);
    void calibrate_accel_prepare();
    void calibrate_accel_step();
    void calibrate_accel_finish();
    void calibrate_mag_prepare();
    void calibrate_mag_step();
    void calibrate_mag_finish(float *offsets = nullptr, float *scales = nullptr);
    
    // Unified calibration methods
    bool calibrate_accel_gyro(uint16_t gyro_times = 100, uint16_t accel_steps = 100);
    bool calibrate_magnetometer(uint16_t steps = 200);
    bool calibrate_all(uint16_t gyro_times = 100, uint16_t accel_steps = 100, uint16_t mag_steps = 200);

    // Status methods
    bool is_accel_gyro_available() const;
    bool is_magnetometer_available() const;
    bool is_barometer_available() const;
    
    // Chip information methods
    const char* get_accel_gyro_chip_name() const;
    const char* get_magnetometer_chip_name() const;
    const char* get_barometer_chip_name() const;

private:
    TwoWire *_wire;
    SH3001 *_sh3001;
    QMC6310 *_qmc6310;
    SPL06_001 *_spl06_001;

    bool _sh3001_available;
    bool _qmc6310_available;
    bool _spl06_001_available;
};
