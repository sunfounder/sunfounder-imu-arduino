#pragma once

#include <Arduino.h>
#include <Wire.h>

class I2C {
private:
    TwoWire *_wire;
    uint8_t _address;
    uint8_t _retry;

public:
    I2C(uint8_t address, TwoWire *wire = &Wire, uint8_t retry = 5);
    ~I2C();

    bool write_byte(uint8_t data);
    bool write_byte_data(uint8_t reg, uint8_t data);
    bool write_word_data(uint8_t reg, uint16_t data, bool lsb = false);
    bool write_i2c_block_data(uint8_t reg, const uint8_t *data, uint8_t length);
    bool write_i2c_block_data(uint8_t reg, const uint8_t *data, size_t length) {
        return write_i2c_block_data(reg, data, static_cast<uint8_t>(length));
    }

    uint8_t read_byte();
    uint8_t read_byte_data(uint8_t reg);
    uint16_t read_word_data(uint8_t reg, bool lsb = false);
    bool read_i2c_block_data(uint8_t reg, uint8_t *data, uint8_t length);

    bool is_ready();
    static void scan(TwoWire *wire = &Wire);
    static bool is_device_connected(uint8_t address, TwoWire *wire = &Wire);
};

// Utility functions
int32_t twos_complement(uint32_t value, uint8_t bits);
float mapping(float value, float in_min, float in_max, float out_min, float out_max);
