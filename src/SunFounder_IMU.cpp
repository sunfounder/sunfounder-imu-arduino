#include "SunFounder_IMU.hpp"

SunFounder_IMU::SunFounder_IMU(TwoWire *wire) {
    _wire = wire;
    _sh3001 = nullptr;
    _qmc6310 = nullptr;
    _spl06_001 = nullptr;
    _sh3001_available = false;
    _qmc6310_available = false;
    _spl06_001_available = false;
}

SunFounder_IMU::~SunFounder_IMU() {
    delete _sh3001;
    delete _qmc6310;
    delete _spl06_001;
    _sh3001 = nullptr;
    _qmc6310 = nullptr;
    _spl06_001 = nullptr;
}

bool SunFounder_IMU::begin() {
    // Initialize each sensor independently
    init_sh3001();
    init_qmc6310();
    init_spl06_001();
    
    // Return true if at least one sensor is available
    return _sh3001_available || _qmc6310_available || _spl06_001_available;
}

bool SunFounder_IMU::init_sh3001() {
    if (_sh3001 == nullptr) {
        _sh3001 = new SH3001(0, _wire);
    }
    
    _sh3001_available = _sh3001->init();
    return _sh3001_available;
}

bool SunFounder_IMU::init_qmc6310() {
    if (_qmc6310 == nullptr) {
        _qmc6310 = new QMC6310(0, _wire);
    }
    
    _qmc6310_available = _qmc6310->init();
    return _qmc6310_available;
}

bool SunFounder_IMU::init_spl06_001() {
    if (_spl06_001 == nullptr) {
        _spl06_001 = new SPL06_001(0, _wire);
    }
    
    _spl06_001_available = _spl06_001->init();
    return _spl06_001_available;
}

SH3001* SunFounder_IMU::get_sh3001() {
    return _sh3001;
}

QMC6310* SunFounder_IMU::get_qmc6310() {
    return _qmc6310;
}

SPL06_001* SunFounder_IMU::get_spl06_001() {
    return _spl06_001;
}

bool SunFounder_IMU::read_all(
    AccelData &accel,
    GyroData &gyro,
    MagData &mag,
    float &temperature,
    float &pressure,
    float &altitude,
    float &azimuth
) {
    // Initialize with zeros
    accel = {0.0f, 0.0f, 0.0f};
    gyro = {0.0f, 0.0f, 0.0f};
    mag = {0.0f, 0.0f, 0.0f};
    temperature = 0.0f;
    pressure = 0.0f;
    altitude = 0.0f;
    azimuth = 0.0f;
    
    // Read from SH3001 (accel, gyro)
    if (_sh3001_available) {
        _sh3001->read_all(accel, gyro, temperature);
    }
    
    // Read from QMC6310 (magnetic and azimuth)
    if (_qmc6310_available) {
        _qmc6310->read(mag, azimuth);
    }
    
    // Read from SPL06_001 (pressure, altitude, and temperature - override SH3001 temperature)
    if (_spl06_001_available) {
        pressure = _spl06_001->get_pressure();
        altitude = _spl06_001->get_altitude(pressure);
        temperature = _spl06_001->get_temperature(); // Use temperature from barometer instead of SH3001
    }
    
    // Return true if at least one sensor is available
    return _sh3001_available || _qmc6310_available || _spl06_001_available;
}

bool SunFounder_IMU::read(IMUData &data) {
    // Initialize with zeros
    data.accel = {0.0f, 0.0f, 0.0f};
    data.gyro = {0.0f, 0.0f, 0.0f};
    data.mag = {0.0f, 0.0f, 0.0f};
    data.temperature = 0.0f;
    data.pressure = 0.0f;
    data.altitude = 0.0f;
    data.azimuth = 0.0f;
    
    // Read from SH3001 (accel, gyro)
    if (_sh3001_available) {
        _sh3001->read_all(data.accel, data.gyro, data.temperature);
    }
    
    // Read from QMC6310 (magnetic and azimuth)
    if (_qmc6310_available) {
        _qmc6310->read(data.mag, data.azimuth);
    }
    
    // Read from SPL06_001 (pressure, altitude, and temperature - override SH3001 temperature)
    if (_spl06_001_available) {
        data.pressure = _spl06_001->get_pressure();
        data.altitude = _spl06_001->get_altitude(data.pressure);
        data.temperature = _spl06_001->get_temperature(); // Use temperature from barometer instead of SH3001
    }
    
    // Return true if at least one sensor is available
    return _sh3001_available || _qmc6310_available || _spl06_001_available;
}

bool SunFounder_IMU::read_accel(AccelData &accel) {
    if (!_sh3001_available) {
        accel = {0.0f, 0.0f, 0.0f};
        return false;
    }
    
    accel = _sh3001->read_accel();
    return true;
}

bool SunFounder_IMU::read_gyro(GyroData &gyro) {
    if (!_sh3001_available) {
        gyro = {0.0f, 0.0f, 0.0f};
        return false;
    }
    
    gyro = _sh3001->read_gyro();
    return true;
}

bool SunFounder_IMU::read_mag(MagData &mag) {
    if (!_qmc6310_available) {
        mag = {0.0f, 0.0f, 0.0f};
        return false;
    }
    
    mag = _qmc6310->get_magnetometer_data();
    return true;
}

bool SunFounder_IMU::read_mag_with_azimuth(MagData &mag, float &azimuth) {
    if (!_qmc6310_available) {
        mag = {0.0f, 0.0f, 0.0f};
        azimuth = 0.0f;
        return false;
    }
    
    _qmc6310->read(mag, azimuth);
    return true;
}

bool SunFounder_IMU::read_pressure(float &pressure) {
    if (!_spl06_001_available) {
        pressure = 0.0f;
        return false;
    }
    
    pressure = _spl06_001->get_pressure();
    return true;
}

bool SunFounder_IMU::read_temperature(float &temperature) {
    if (!_sh3001_available) {
        temperature = 0.0f;
        return false;
    }
    
    temperature = _sh3001->read_temperature();
    return true;
}

bool SunFounder_IMU::read_altitude(float &altitude) {
    if (!_spl06_001_available) {
        altitude = 0.0f;
        return false;
    }
    
    altitude = _spl06_001->get_altitude();
    return true;
}

void SunFounder_IMU::calibrate_gyro(uint16_t times) {
    if (_sh3001_available) {
        _sh3001->calibrate_gyro(times);
    }
}

void SunFounder_IMU::calibrate_accel_prepare() {
    if (_sh3001_available) {
        _sh3001->calibrate_accel_prepare();
    }
}

void SunFounder_IMU::calibrate_accel_step() {
    if (_sh3001_available) {
        _sh3001->calibrate_accel_step();
    }
}

void SunFounder_IMU::calibrate_accel_finish() {
    if (_sh3001_available) {
        _sh3001->calibrate_accel_finish();
    }
}

void SunFounder_IMU::calibrate_mag_prepare() {
    if (_qmc6310_available) {
        _qmc6310->calibrate_prepare();
    }
}

void SunFounder_IMU::calibrate_mag_step() {
    if (_qmc6310_available) {
        _qmc6310->calibrate_step();
    }
}

void SunFounder_IMU::calibrate_mag_finish(float *offsets, float *scales) {
    if (_qmc6310_available) {
        _qmc6310->calibrate_finish(offsets, scales);
    }
}

bool SunFounder_IMU::calibrate_accel_gyro(uint16_t gyro_times, uint16_t accel_steps) {
    if (!_sh3001_available) {
        return false;
    }
    
    // Calibrate gyroscope first (stationary calibration)
    Serial.println("Calibrating gyroscope...");
    Serial.println("Please keep the IMU stationary during gyro calibration.");
    delay(2000);
    _sh3001->calibrate_gyro(gyro_times);
    
    // Calibrate accelerometer (multiple positions)
    Serial.println("\nCalibrating accelerometer...");
    Serial.println("Please place the IMU in different positions during calibration.");
    Serial.println("Calibration will take about 20 seconds...");
    
    _sh3001->calibrate_accel_prepare();
    
    for (uint16_t i = 0; i < accel_steps; i++) {
        _sh3001->calibrate_accel_step();
        delay(100);
        
        // Print progress every 10 steps
        if (i % 10 == 0) {
            Serial.print(".");
        }
    }
    
    _sh3001->calibrate_accel_finish();
    Serial.println("\nAccelerometer and gyroscope calibration completed!");
    
    return true;
}

bool SunFounder_IMU::calibrate_magnetometer(uint16_t steps) {
    if (!_qmc6310_available) {
        return false;
    }
    
    Serial.println("Calibrating QMC6310 magnetometer...");
    Serial.println("Please rotate the IMU in all directions during calibration.");
    Serial.println("Calibration will take about 20 seconds...");
    
    _qmc6310->calibrate_prepare();
    
    for (uint16_t i = 0; i < steps; i++) {
        _qmc6310->calibrate_step();
        delay(100);
        
        // Print progress every 10 steps
        if (i % 10 == 0) {
            Serial.print(".");
        }
    }
    
    _qmc6310->calibrate_finish(nullptr, nullptr);
    Serial.println("\nMagnetometer calibration completed!");
    
    return true;
}

bool SunFounder_IMU::calibrate_all(uint16_t gyro_times, uint16_t accel_steps, uint16_t mag_steps) {
    bool success = true;
    
    Serial.println("Starting full IMU calibration...");
    
    // Calibrate Accelerometer/Gyroscope
    if (!calibrate_accel_gyro(gyro_times, accel_steps)) {
        success = false;
        Serial.println("Accelerometer/Gyroscope calibration failed!");
    }
    
    // Calibrate magnetometer
    if (!calibrate_magnetometer(mag_steps)) {
        success = false;
        Serial.println("Magnetometer calibration failed!");
    }
    
    if (success) {
        Serial.println("\nAll calibrations completed successfully!");
    } else {
        Serial.println("\nSome calibrations failed!");
    }
    
    return success;
}

bool SunFounder_IMU::is_accel_gyro_available() const {
    return _sh3001_available;
}

bool SunFounder_IMU::is_magnetometer_available() const {
    return _qmc6310_available;
}

bool SunFounder_IMU::is_barometer_available() const {
    return _spl06_001_available;
}

const char* SunFounder_IMU::get_accel_gyro_chip_name() const {
    return _sh3001_available ? "SH3001" : "Not available";
}

const char* SunFounder_IMU::get_magnetometer_chip_name() const {
    return _qmc6310_available ? "QMC6310" : "Not available";
}

const char* SunFounder_IMU::get_barometer_chip_name() const {
    return _spl06_001_available ? "SPL06-001" : "Not available";
}
