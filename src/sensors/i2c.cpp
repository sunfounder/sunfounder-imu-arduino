#include "i2c.hpp"

// #define I2C_DEBUG

I2C::I2C(TwoWire *wire, uint8_t address) : _wire(wire), _address(address) {}

I2C::~I2C() {}

void I2C::begin(uint8_t retry) {
  _retry = retry;
  _wire->begin();
}

bool I2C::write_byte(uint8_t data) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-w] 0x%02X, 0x%02X", _address, data);
  Serial.println(msg);
#endif
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
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-w] 0x%02X, 0x%02X, 0x%02X", _address, reg, data);
  Serial.println(msg);
#endif
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

bool I2C::write_word_data(uint8_t reg, uint16_t data) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-w] 0x%02X, 0x%02X, 0x%04X", _address, reg, data);
  Serial.println(msg);
#endif
  for (uint8_t i = 0; i < _retry; i++) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(static_cast<uint8_t>((data >> 8) & 0xFF));
    _wire->write(static_cast<uint8_t>(data & 0xFF));
    if (_wire->endTransmission() == 0) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool I2C::write_i2c_block_data(uint8_t reg, const uint8_t *data,
                               uint8_t length) {
#ifdef I2C_DEBUG
  char msg[255];
  sprintf(msg, "[i2c-w] 0x%02X, 0x%02X, ", _address, reg);
  for (uint8_t j = 0; j < length; j++) {
    sprintf(msg, "%s, 0x%02X", msg, data[j]);
  }
  Serial.println(msg);
#endif
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

bool I2C::read_byte(uint8_t *data) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-r] 0x%02X", _address);
#endif
  for (uint8_t i = 0; i < _retry; i++) {
    if (_wire->requestFrom(_address, static_cast<uint8_t>(1)) == 1) {
      *data = _wire->read();
#ifdef I2C_DEBUG
      sprintf(msg, "%s, 0x%02X", msg, *data);
      Serial.println(msg);
#endif
      return true;
    }
    delay(1);
  }
#ifdef I2C_DEBUG
  sprintf(msg, "%s Failed!", msg);
  Serial.println(msg);
#endif
  return false;
}

bool I2C::read_byte_data(uint8_t reg, uint8_t *data) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-r] 0x%02X, 0x%02X", _address, reg);
#endif
  for (uint8_t i = 0; i < _retry; i++) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) == 0) {
      if (_wire->requestFrom(_address, static_cast<uint8_t>(1)) == 1) {
        *data = _wire->read();
#ifdef I2C_DEBUG
        sprintf(msg, "%s, 0x%02X", msg, *data);
        Serial.println(msg);
#endif
        return true;
      }
    }
    delay(1);
  }
#ifdef I2C_DEBUG
  sprintf(msg, "%s Failed!", msg);
  Serial.println(msg);
#endif
  return false;
}

bool I2C::read_word_data(uint8_t reg, uint16_t *data) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-r] 0x%02X, 0x%02X", _address, reg);
#endif
  for (uint8_t i = 0; i < _retry; i++) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) == 0) {
      if (_wire->requestFrom(_address, static_cast<uint8_t>(2)) == 2) {
        uint8_t low = _wire->read();
        uint8_t high = _wire->read();
        *data = (high << 8) | low;
#ifdef I2C_DEBUG
        sprintf(msg, "%s, 0x%04X", msg, *data);
        Serial.println(msg);
#endif
        return true;
      }
    }
    delay(1);
  }
  return false;
#ifdef I2C_DEBUG
  sprintf(msg, "%s Failed!", msg);
  Serial.println(msg);
#endif
}

bool I2C::read_i2c_block_data(uint8_t reg, uint8_t *data, uint8_t length) {
#ifdef I2C_DEBUG
  char msg[200];
  sprintf(msg, "[i2c-r] 0x%02X, 0x%02X", _address, reg);
#endif
  for (uint8_t i = 0; i < _retry; i++) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
      Serial.print("[Error] Failed to read register 0x");
      Serial.println(reg, HEX);
      continue;
    }
    uint8_t bytes_read = _wire->requestFrom(_address, length);
    if (bytes_read != length) {
      Serial.print("[Error] Length error on register 0x");
      Serial.print(reg, HEX);
      Serial.print(" expect ");
      Serial.print(length);
      Serial.print(" bytes, reads ");
      Serial.println(bytes_read);
      continue;
    }
    for (uint8_t j = 0; j < length; j++) {
      data[j] = _wire->read();
#ifdef I2C_DEBUG
      sprintf(msg, "%s, 0x%02X", msg, data[j]);
#endif
    }
#ifdef I2C_DEBUG
    Serial.println(msg);
#endif
    return true;
  }
#ifdef I2C_DEBUG
  sprintf(msg, "%s Failed!", msg);
  Serial.println(msg);
#endif
  return false;
}

uint8_t I2C::scan(TwoWire *wire, uint8_t *addresses) {
#ifdef I2C_DEBUG
  Serial.println("scan:");
#endif
  byte error, address;
  uint8_t count = 0;

  for (address = 1; address < 127; address++) {
    wire->beginTransmission(address);
    error = wire->endTransmission();

    if (error == 0) {
      addresses[count] = address;
      count++;
#ifdef I2C_DEBUG
      Serial.print("Found address: 0x");
      Serial.println(address, HEX);
#endif
    }
  }
#ifdef I2C_DEBUG
  Serial.print("Total found: ");
  Serial.println(count);
#endif
  return count;
}
