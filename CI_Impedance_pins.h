#pragma once

#include <stdint.h>

class ImpedanceSensingPins
{
public:
    ImpedanceSensingPins(void) {};

    void init(void)
    {
        pinMode(aux1, OUTPUT);
        pinMode(aux2, OUTPUT);
        pinMode(buttonPin, INPUT_PULLUP); // button is pulled high when open, grounded when pressed
        pinMode(ref200_EN, OUTPUT);
        digitalWriteFast(ref200_EN, LOW);
        pinMode(short_EA, OUTPUT);
        digitalWriteFast(short_EA, HIGH);
    }

    // MUX
    const uint8_t A0 = 15;
    const uint8_t A1 = 16;
    const uint8_t A2 = 17;
    const uint8_t A3 = 18;
    const uint8_t CSA = 19;
    const uint8_t CSB = 20;
    const uint8_t WR = 21;
    const uint8_t EN = 22;

    // MUX switch connections
    const uint8_t sw_EA2 = 14; // pins 38/47  
    const uint8_t sw_EA3 = 15; // pins 39/46
    const uint8_t sw_EA5 = 16; // pins 40/45
    const uint8_t sw_nc = 12;  // pins 1/36; unconnected
    //const uint8_t swB_gnd = 6;   // pin 30; connected to ground

    // pulse control pins
    const uint8_t ref200_EN = 12; // HIGH to connect REF200
    const uint8_t short_EA = 10;  // HIGH -> shorts DA-DB

    const int buttonPin = 3; // momentary tactile button

    const int aux1 = 23;
    const int aux2 = 13;
};
