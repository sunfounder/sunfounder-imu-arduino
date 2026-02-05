#include "utils.hpp"

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
