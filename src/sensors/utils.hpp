#pragma once

#include <Arduino.h>

struct Vector3f {
  float x;
  float y;
  float z;

  Vector3f() : x(0), y(0), z(0) {}
  Vector3f(float x_val, float y_val, float z_val)
      : x(x_val), y(y_val), z(z_val) {}
};

#define VECTOR_X_PLUS 0
#define VECTOR_X_MINUS 1
#define VECTOR_Y_PLUS 2
#define VECTOR_Y_MINUS 3
#define VECTOR_Z_PLUS 4
#define VECTOR_Z_MINUS 5

// Utility functions
int32_t twos_complement(uint32_t value, uint8_t bits);
float mapping(float value, float in_min, float in_max, float out_min,
              float out_max);
