#include <Arduino.h>
#include <Wire.h>
#include <I2SConfig.hpp>
#include "../../astrobulles_esp/include/Config.hpp"
#include <Button.h>

I2COutput rxData;
I2CInput txData;

uint32_t rotary = 0;
bool oneBit(unsigned v) {return (v && !(v & (v - 1)));};
const int rotaryLen = (sizeof(PINS_MEGA_ROTARY)/sizeof(*PINS_MEGA_ROTARY));
const uint32_t rotaryMask = ((1 << (rotaryLen))-1);
uint8_t rotaryStable = 0;

Button BtnSine(PIN_MEGA_BTN_SINE);
Button BtnSineAsync(PIN_MEGA_BTN_SINE_ASYNC);
Button BtnFlash(PIN_MEGA_BTN_FLASH);
Button BtnRotate(PIN_MEGA_BTN_ROTATE);
Button BtnPanic(PIN_MEGA_BTN_PANIC);


void receiveEvent(int numBytesReceived) 
{
    Serial.println("received i2c");
    Wire.readBytes( (byte*) &rxData, numBytesReceived);
    analogWrite(PIN_MEGA_FRONT_LED_R, rxData.data[0]*rxData.data[3]/0xff);
    analogWrite(PIN_MEGA_FRONT_LED_G, rxData.data[1]*rxData.data[3]/0xff);
    analogWrite(PIN_MEGA_FRONT_LED_B, rxData.data[2]*rxData.data[3]/0xff);
    analogWrite(PIN_MEGA_BACK_LED_R, rxData.data[4]*rxData.data[7]/0xff);
    analogWrite(PIN_MEGA_BACK_LED_G, rxData.data[5]*rxData.data[7]/0xff);
    analogWrite(PIN_MEGA_BACK_LED_B, rxData.data[6]*rxData.data[7]/0xff);
    analogWrite(PIN_MEGA_HEAD_LED_R, rxData.data[8]*rxData.data[11]/0xff);
    analogWrite(PIN_MEGA_HEAD_LED_G, rxData.data[9]*rxData.data[11]/0xff);
    analogWrite(PIN_MEGA_HEAD_LED_B, rxData.data[10]*rxData.data[11]/0xff);
}

void requestEvent() 
{
    Serial.println("requested i2c");
    txData.rotary = rotaryStable;
    txData.btnFlash = BtnFlash.pressed();
    txData.btnRotate = BtnRotate.pressed();
    txData.btnSine = BtnSine.pressed();
    txData.btnSineAsync = BtnSineAsync.pressed();
    txData.btnPanic = BtnPanic.pressed();
    txData.swOn = digitalRead(PIN_MEGA_ON_SWITCH);
    Wire.write((byte*) &txData, sizeof(txData));
}

void setup() 
{
    Serial.begin(115200);
    Wire.begin(I2C_ADDRESS_ARDUINO);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // initPins
    pinMode(PIN_MEGA_FRONT_LED_R, OUTPUT);
    pinMode(PIN_MEGA_FRONT_LED_G, OUTPUT);
    pinMode(PIN_MEGA_FRONT_LED_B, OUTPUT);
    pinMode(PIN_MEGA_BACK_LED_R, OUTPUT);
    pinMode(PIN_MEGA_BACK_LED_G, OUTPUT);
    pinMode(PIN_MEGA_BACK_LED_B, OUTPUT);
    pinMode(PIN_MEGA_HEAD_LED_R, OUTPUT);
    pinMode(PIN_MEGA_HEAD_LED_G, OUTPUT);
    pinMode(PIN_MEGA_HEAD_LED_B, OUTPUT);
    for (int pin : PINS_MEGA_ROTARY) {
        pinMode(pin, INPUT_PULLUP);
    }
    
    BtnSine.begin();
    BtnSineAsync.begin();
    BtnFlash.begin();
    BtnRotate.begin();
    BtnPanic.begin();

    pinMode(PIN_MEGA_ON_SWITCH, INPUT_PULLUP);
}

void loop() 
{
    for (int pin : PINS_MEGA_ROTARY) {
        rotary = (rotary << 1) + !digitalRead(pin);
    }
    if (oneBit(rotary &= rotaryMask) && ((rotary & rotaryMask) != ((rotary >> rotaryLen)  & rotaryMask))) {
        int i = 0;
        while (rotary >>= 1) {
            i++;
        }
        rotaryStable = i;
    }; 
}
