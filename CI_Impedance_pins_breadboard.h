#pragma once

#include <stdint.h>
#include <array>

class ImpedanceSensingPins
{
public:
    ImpedanceSensingPins(void) {};

    void init(void)
    {
        // pulse control
        pinMode(ref200_EN, OUTPUT);
        digitalWriteFast(ref200_EN, LOW);

        pinMode(short_EA, OUTPUT);
        digitalWriteFast(short_EA, HIGH);
        

        // ADCs
        //pinMode(adc_batt, INPUT);
        pinMode(adc_EA, INPUT);


        //pinMode(test_batt, OUTPUT);
        //digitalWriteFast(test_batt, LOW);

        //pinMode(led_low_batt, OUTPUT);
        //digitalWriteFast(led_low_batt, LOW);  

        //pinMode(led_status, OUTPUT);
        //digitalWriteFast(led_status, LOW);

        //pinMode(test_batt, OUTPUT);
        //digitalWriteFast(test_batt, LOW);

        pinMode(aux1, OUTPUT);
        digitalWriteFast(aux1, LOW);

        pinMode(aux2, OUTPUT);
        digitalWriteFast(aux2, LOW);

        pinMode(buttonPin, INPUT_PULLUP); // button is pulled high when open, grounded when pressed


    }

    // MUX
    struct MuxPins {
        const uint8_t A0 = 15;
        const uint8_t A1 = 16;
        const uint8_t A2 = 17;
        const uint8_t A3 = 18;
        const uint8_t CSA = 19;
        const uint8_t CSB = 20;
        const uint8_t WR = 21;
        const uint8_t EN = 22;
    } mux;
    const std::array<uint8_t, 8> mux_pins = {{ mux.A0, mux.A1, mux.A2, mux.A3, mux.CSA, mux.CSB, mux.WR, mux.EN }};

    // EA <-> MUX switch connections (NOTE: EA[1] = EA1; EA[0] is not connected)
    std::array<uint8_t, 13> EA = { { 13, 1, 14, 15, 1, 16, 1, 1, 1, 1, 1, 1, 1 } };
    
    //const uint8_t sw_EA2 = 14; // pins 38/47  
    //const uint8_t sw_EA3 = 15; // pins 39/46
    //const uint8_t sw_EA5 = 16; // pins 40/45
    //const uint8_t sw_nc = 12;  // pins 1/36; unconnected
    //const uint8_t swB_gnd = 6;   // pin 30; connected to ground

    // pulse control
    const uint8_t ref200_EN = 12; // HIGH to connect REF200
    const uint8_t short_EA = 10;  // HIGH -> shorts DA-DB

    // OLED display (SPI)
    //const uint8_t oled_cs = 10;
    //const uint8_t oled_dc = 14;
    //const uint8_t oled_reset = 15;

    // ADC
    //const uint8_t adc_batt = A0; // pin 14
    const uint8_t adc_EA = A4;   // pin 18

    //const uint8_t test_batt = 16; // HIGH to enable battery monitor circuitry
    //const uint8_t led_low_batt = 19;
    //const uint8_t led_status = 20;
    const uint8_t buttonPin = 3; // momentary tactile button
    const uint8_t aux1 = 23;
    const uint8_t aux2 = 13;
};
