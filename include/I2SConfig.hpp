#pragma once

#include <Arduino.h>

#define I2C_ADDRESS_ESP32 8
#define I2C_ADDRESS_ARDUINO 9

struct I2COutput {
    // FRONT_LED, BACK_LED, HEAD_LED 
    // all in (red, green, blue, brightness) format
    uint8_t data[12];
};

struct I2CInput {
    uint8_t rotary;
    bool btnSine:1;
    bool btnSineAsync:1;
    bool btnFlash:1;
    bool btnRotate:1;
    bool btnPanic:1;
    bool swOn:1;
};
