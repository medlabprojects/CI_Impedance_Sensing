#pragma once

#include <stdint.h>
#include <array>
#include <SPI.h>
#include <SFE_MicroOLED.h>

class ImpedanceSensingPins
{
public:
    ImpedanceSensingPins(void) 
        //: oled(oled_reset, oled_dc, oled_cs)
    {};

    void init(void)
    {
        // pulse control
        pinMode(ref200_EN, OUTPUT);
        digitalWriteFast(ref200_EN, LOW);

        pinMode(short_EA, OUTPUT);
        digitalWriteFast(short_EA, HIGH);


        // ADCs
        pinMode(adc_batt, INPUT);
        pinMode(adc_EA, INPUT);


        pinMode(test_batt, OUTPUT);
        digitalWriteFast(test_batt, LOW);

        pinMode(led_low_batt, OUTPUT);
        digitalWriteFast(led_low_batt, LOW);

        pinMode(led_status, OUTPUT);
        digitalWriteFast(led_status, LOW);

        pinMode(test_batt, OUTPUT);
        digitalWriteFast(test_batt, LOW);

        pinMode(aux1, OUTPUT);
        digitalWriteFast(aux1, LOW);

        pinMode(aux2, OUTPUT);
        digitalWriteFast(aux2, LOW);

        pinMode(buttonPin, INPUT_PULLUP); // button is pulled high when open, grounded when pressed

        //oled.begin();    // Initialize the OLED
        //oled.clear(ALL); // Clear the display's internal memory
        //oled.setFontType(0);
        //oled.setCursor(0, 16);
        //oled.println("Impedance");
        //oled.print(" Sensing");
        //oled.display();  // Display what's in the buffer (splashscreen)
    }

    // MUX
    struct MuxPins {
        const uint8_t A0 = 7;
        const uint8_t A1 = 6;
        const uint8_t A2 = 5;
        const uint8_t A3 = 4;
        const uint8_t CSA = 3;
        const uint8_t CSB = 2;
        const uint8_t WR = 1;
        const uint8_t EN = 0;
    } mux;
    const std::array<uint8_t, 8> mux_pins = { { mux.A0, mux.A1, mux.A2, mux.A3, mux.CSA, mux.CSB, mux.WR, mux.EN } };

    // EA <-> MUX switch connections (NOTE: EA[1] = EA1; EA[0] is not connected)
    std::array<uint8_t, 13> EA = { { 16, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 } };

    // pulse control
    const uint8_t ref200_EN = 9; // HIGH to connect REF200
    const uint8_t short_EA = 8;  // HIGH -> shorts DA-DB

    // OLED display (SPI)
    //MicroOLED oled;
    const uint8_t oled_cs = 10;
    const uint8_t oled_dc = 14;
    const uint8_t oled_reset = 15;


    // ADC
    const uint8_t adc_batt = A3; // pin 14
    const uint8_t adc_EA = A4;   // pin 18

    const uint8_t test_batt = 16; // HIGH to enable battery monitor circuitry
    const uint8_t led_low_batt = 19;
    const uint8_t led_status = 20;
    const uint8_t buttonPin = 21; // momentary tactile button
    const uint8_t aux1 = 22;
    const uint8_t aux2 = 23;
};
