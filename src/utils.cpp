#include "utils.hpp"

I2C::I2C(uint8_t address, TwoWire *wire, uint8_t retry) {
    _wire = wire;
    _address = address;
    _retry = retry;
}

I2C::~I2C() {
}

bool I2C::write_byte(uint8_t data) {
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(data);
        if (_wire->endTransmission() == 0) {
            return true;
        }
        delay(1);
    }
    return false;
}

bool I2C::write_byte_data(uint8_t reg, uint8_t data) {
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write(data);
        if (_wire->endTransmission() == 0) {
            return true;
        }
        delay(1);
    }
    return false;
}

bool I2C::write_word_data(uint8_t reg, uint16_t data, bool lsb) {
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (lsb) {
            _wire->write(static_cast<uint8_t>(data & 0xFF));
            _wire->write(static_cast<uint8_t>((data >> 8) & 0xFF));
        } else {
            _wire->write(static_cast<uint8_t>((data >> 8) & 0xFF));
            _wire->write(static_cast<uint8_t>(data & 0xFF));
        }
        if (_wire->endTransmission() == 0) {
            return true;
        }
        delay(1);
    }
    return false;
}

bool I2C::write_i2c_block_data(uint8_t reg, const uint8_t *data, uint8_t length) {
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        for (uint8_t j = 0; j < length; j++) {
            _wire->write(data[j]);
        }
        if (_wire->endTransmission() == 0) {
            return true;
        }
        delay(1);
    }
    return false;
}

uint8_t I2C::read_byte() {
    uint8_t data = 0;
    for (uint8_t i = 0; i < _retry; i++) {
        if (_wire->requestFrom(_address, static_cast<uint8_t>(1)) == 1) {
            data = _wire->read();
            return data;
        }
        delay(1);
    }
    return data;
}

uint8_t I2C::read_byte_data(uint8_t reg) {
    uint8_t data = 0;
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (_wire->endTransmission(false) == 0) {
            if (_wire->requestFrom(_address, static_cast<uint8_t>(1)) == 1) {
                data = _wire->read();
                return data;
            }
        }
        delay(1);
    }
    return data;
}

uint16_t I2C::read_word_data(uint8_t reg, bool lsb) {
    uint16_t data = 0;
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (_wire->endTransmission(false) == 0) {
            if (_wire->requestFrom(_address, static_cast<uint8_t>(2)) == 2) {
                if (lsb) {
                    uint8_t low = _wire->read();
                    uint8_t high = _wire->read();
                    data = (high << 8) | low;
                } else {
                    uint8_t high = _wire->read();
                    uint8_t low = _wire->read();
                    data = (high << 8) | low;
                }
                return data;
            }
        }
        delay(1);
    }
    return data;
}

bool I2C::read_i2c_block_data(uint8_t reg, uint8_t *data, uint8_t length) {
    for (uint8_t i = 0; i < _retry; i++) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (_wire->endTransmission(false) == 0) {
            uint8_t bytes_read = _wire->requestFrom(_address, length);
            if (bytes_read == length) {
                for (uint8_t j = 0; j < length; j++) {
                    data[j] = _wire->read();
                }
                return true;
            }
        }
        delay(1);
    }
    return false;
}

bool I2C::is_ready() {
    return is_device_connected(_address, _wire);
}

void I2C::scan(TwoWire *wire) {
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++) {
        wire->beginTransmission(address);
        error = wire->endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.println("Scan complete");
    }
}

bool I2C::is_device_connected(uint8_t address, TwoWire *wire) {
    wire->beginTransmission(address);
    byte error = wire->endTransmission();
    return (error == 0);
}

// Utility functions
int32_t twos_complement(uint32_t value, uint8_t bits) {
    if (value & (1 << (bits - 1))) {
        value -= (1 << bits);
    }
    return static_cast<int32_t>(value);
}

float mapping(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
