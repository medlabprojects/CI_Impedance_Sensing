#include <ADC.h>
#include "RingBuffer.h"
#include <IntervalTimer.h>
#include <Eigen.h>
#include <Eigen/Dense>
//#include <SFE_MicroOLED.h>
#include "ADG726.h"
#include "CI_Impedance_pins.h"
#include "CI_Impedance_fsm.h"

using namespace Eigen;

ImpedanceSensingPins pins;

ADG726 mux({ pins.A0, pins.A1, pins.A2, pins.A3, pins.CSA, pins.CSB, pins.WR, pins.EN });

//const uint8_t size_EA = 3;
//const uint8_t EA[size_EA] = {sw_EA2, sw_EA3, sw_EA5};
const uint8_t size_EA = 2;
const uint8_t EA[size_EA] = { pins.sw_EA2, pins.sw_EA3 };
uint8_t ch_index = 0;                  // currently selected channels to measure
uint8_t ch_anode = EA[ch_index];       // anode channel during initial positive pulse
uint8_t ch_cathode = EA[ch_index + 1]; // cathode channel during inital positive pulse

IntervalTimer timerPulse;
//const int pulse_time = 100; // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulse_time
const int pulse_time = 55;       // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulse_time
const int interPulseDelay = 0;   // multiples of pulse_time between pulse trains ([us] = pulse_time * interPulseDelay)
volatile bool timerFlag = false;
volatile int timerCount = 0;

IntervalTimer timerAdc;    // timer for triggering ADC samples
const float adcTime = 8.5; // [us] time between each adc trigger
const int adcDelay = 0;    // number of adc samples to discard before saving (while waiting to current pulse to stabalize)
const int nSamples = 5;    // number of ADC samples to take during positive pulse
const int nPulses = 32;    // number of pulse trains which will be sampled and averaged together for each single output measurement
//const float filterSigma = 2.5; // samples more than this many std deviations from the mean will be filtered out
//const float filterVariance = filterSigma * filterSigma; // variance is actually used since it is faster to compute 

ADC *adc = new ADC(); // ADC object
RingBuffer *adcRingBuffer = new RingBuffer; // buffer to capture adc samples (changed size in .h to 16 elements)
//MatrixXi adcRaw(nSamples, nPulses); // stores raw (integer) values from adc
MatrixXf adcRaw(nSamples, nPulses); // stores raw (integer) values from adc; float because .mean() method is later called
volatile int adcCount = 0; // current sample number (row index of adcRaw)
volatile int pulseCount = 0; // current pulse number (column index of adcRaw)
bool adcFlag = false;
double adc2Voltage = 0.0;

MatrixXd Alinfit(nSamples, 2); // linear regression matrix for line fitting
double resistance = 0.0; // resistive component of measured impedance
double capacitance = 0.0;// capacitive component of measured impedance

volatile bool runFlag = false;

char cmd = 0;

//********************************************************************
// STATE MACHINE
//********************************************************************

//***********************
// ****** powerUp *******

fsmState powerUp(void) {
    stateCurrent = statePowerUp;

    // initialize serial port
    Serial.begin(115200);
    delay(1000);
    Serial.println("State: PowerUp");

    // initialize pins
    pins.init();
    mux.init();
    
    // 
    attachInterrupt(pins.buttonPin, button_isr, FALLING);

    // set up ADC
    pinMode(A0, INPUT);
    //  pinMode(A10, INPUT); // Diff Channel 0 +
    //  pinMode(A11, INPUT); // Diff Channel 0 -
    adc->setAveraging(0); // no averaging; take single samples
    adc->setResolution(16); // resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // sets ADCK to highest speed within spec for all resolutions
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // HIGH_SPEED adds +6 ADCK; MED_SPEED adds +10 ADCK
                                                           //  adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // use 1.2V internal reference
    adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0); // use 3.3V internal reference
    adc2Voltage = 3.3 / 65536.0; // conversion factor for 16-bit differential adc values
                                 //  adc2Voltage = 3.3/32768.0; // conversion factor for 16-bit differential adc values
                                 //  adc2Voltage = 1.2/32768.0; // conversion factor for 16-bit adc values
                                 //  adc2Voltage = 3.3/adc->getPGA()/adc->getMaxValue(); // conversion factor for adc values
                                 //  adc->adc0->analogReadDifferential(A10,A11); // call once to setup -> differential
    adc->adc0->analogRead(A0); // call once to setup -> single-ended
    adc->enableInterrupts(ADC_0);

    Serial.print("adc2Voltage = ");
    Serial.println(adc2Voltage, 8);

    Serial.print("adc_max_val = ");
    Serial.println(adc->adc0->getMaxValue());

    // set up linear regression matrix
    Alinfit.col(0) = VectorXd::Ones(nSamples);
    Alinfit.col(1) = VectorXd::LinSpaced(nSamples, 1 + adcDelay, nSamples + adcDelay);
    //  print_mtxf(Alinfit);

    Serial.println("\nInitialization Complete");

    return stateNotRunning;
}

//*************************
// ****** notRunning ******

fsmState notRunning(void) {
    stateCurrent = stateNotRunning;

    // ensure EA is disconnected
    disableREF200();
    mux.selectA(pins.sw_nc);
    mux.selectB(pins.sw_nc);
    mux.enable();

    // stop IntervalTimers
    timerPulse.end();
    timerAdc.end();

    // reset counts
    pulseCount = 0;
    timerCount = 0;

    Serial.println("\nState: NotRunning\n");

    Serial.print("+100 uA pulse for ");
    Serial.print(pulse_time);
    Serial.print(" us, followed immediately by -100 uA pulse for ");
    Serial.print(pulse_time);
    Serial.println(" us");
    Serial.print("Inter-pulse delay is ");
    Serial.print(interPulseDelay * pulse_time);
    Serial.println(" us\n");
    Serial.println("\nType 's' or press button to start pulses");

    while ((cmd != 's') && (cmd != 't') && (!runFlag)) {
        if (Serial.available() > 0) {
            cmd = Serial.read();
        }
    }

    if (cmd == 't') {
        //ch_anode = sw_test1;
        //ch_cathode = sw_test2;
    }

    runFlag = true;

    Serial.println("\nStarting Pulses");

    return statePulsePositive;
}

//*****************************
// ****** pulsePositive *******

fsmState pulsePositive(void) {
    stateCurrent = statePulsePositive;

    // reset adcCount
    adcCount = 0;

    // begin pulse IntervalTimer
    timerCount = 0;
    timerPulse.begin(timerPulse_isr, pulse_time);

    // stop shorting
    delayMicroseconds(pulse_time - 10); // wait until just before start of pulse
    digitalWriteFast(pins.short_EA, LOW);

    // wait for first pulse to sync up timing
    while (timerCount == 0);

    // start triggering ADC
    // takes around (1.6 + adcTime + 2.4) [us] to set up IntervalTimer,  wait for first trigger, and start first adc measurement
    timerAdc.begin(timerAdc_isr, adcTime);

    // start positive pulse (current from ch_anode to ch_cathode)
    enableREF200(); // turn on current source

    return statePulseNegative;
}

//*****************************
// ****** pulseNegative *******

fsmState pulseNegative(void) {
    stateCurrent = statePulseNegative;

    // wait for positive pulse to finish
    while (timerCount < 2) {
        // store adc samples
        if (!adcRingBuffer->isEmpty()) {
            adcCount++; // increment count

            if (adcCount >= (nSamples + adcDelay)) {
                timerAdc.end(); // halt triggering
            }

            noInterrupts(); // ensure ring buffer isn't modified during read
                            //      int adcLast = adcRingBuffer->read(); // read sample from buffer
            int adcLast = adcRingBuffer->read(); // read sample from buffer
            interrupts();

            if (adcCount > adcDelay) {
                //        digitalWriteFast(pins.aux2, HIGH);
                adcRaw(adcCount - adcDelay - 1, pulseCount) = adcLast; // store
                                                                       //        digitalWriteFast(pins.aux2, LOW);
            }
        }
    }

    // start negative pulse (current from ch_cathode to ch_anode)
    disableREF200();
    mux.disable(); // avoids temporary short
    mux.selectA(ch_cathode);
    mux.selectB(ch_anode);
    mux.enable();
    enableREF200();

    // ensure ADC triggering has been stopped
    timerAdc.end();

    // ensure buffer is cleared
    while (!adcRingBuffer->isEmpty()) {
        //    digitalWriteFast(pins.aux2, HIGH);
        adcRingBuffer->read();
        //    digitalWriteFast(pins.aux2, LOW);
    }

    // increment pulse count
    pulseCount++;

    return stateInterPulse;
}

//*****************************
// ****** interPulse **********

fsmState interPulse(void) {
    stateCurrent = stateInterPulse;

    // wait for negative pulse to finish
    while (timerCount < 3);

    // disconnect from current source
    disableREF200();

    // short electrodes to bring anode/cathode to ground potential
    delayMicroseconds(10);
    mux.selectA(ch_anode);
    mux.selectB(ch_cathode);
    delayMicroseconds(5);
    digitalWriteFast(pins.short_EA, HIGH);

    // check whether this is the last pulse
    fsmState next = (pulseCount < nPulses ? statePulsePositive : stateComputeZ);

    // wait for interPulseDelay to finish
    while (timerCount < (3 + interPulseDelay));

    return next;
}

//****************************
// ******** computeZ *********

fsmState computeZ(void) {
    stateCurrent = stateComputeZ;

    digitalWriteFast(pins.aux1, HIGH);

    // stop IntervalTimer
    timerPulse.end();

    // reset pulse count
    pulseCount = 0;

    // increment to next channel pair
    ch_index++;
    if (ch_index >= (size_EA - 1)) {
        ch_index = 0; // 'roll-over' back to first pair
    }
    ch_anode = EA[ch_index];
    ch_cathode = EA[ch_index + 1];

    // average together samples taken at same time during pulses (i.e. rows of adcRaw) and convert to voltage
    VectorXd adcMean(nSamples);
    adcMean = adc2Voltage * adcRaw.rowwise().mean().cast<double>();

    // stop ADC triggering and empty buffer
    timerAdc.end();
    while (!adcRingBuffer->isEmpty()) {
        adcRingBuffer->read();
    }

    // fit least-squares line to data
    Vector2d fit;
    fit = Alinfit.colPivHouseholderQr().solve(adcMean);

    // compute resistive component (via intercept of fit)
    // I = 100E-6 [A] ==> 1/I = 1E4 [A]
    resistance = fit(0) * 1.0E4;

    // compute capacitive component (via slope of fit)
    // C[nF] = (0.1[ma] * adcTime[us/sample]) / (slope[V/sample])
    capacitance = 0.1 * (double)adcTime / fit(1);


    // send results
    Serial.print(resistance);
    if(ch_index == 0){
    Serial.println(); // finished scan, back to first pair
    }
    else{
    Serial.print(",");
    }

    //Serial.print(adcRaw(0, 0));

    //  Serial.print(static_cast<uint32_t>(adcRaw(0,0)));
    //  Serial.print(resistance);
    //  Serial.print(",");
    //  if ((capacitance > 500)||(capacitance<0)) {
    //    // too large to cause enough voltage rise over the pulse length
    //    Serial.print("0");
    //  }
    //  else {
    //    Serial.print(capacitance,4);
    //  }
    //for (int ii = 0; ii<nSamples; ii++) {
    //    Serial.print(",");
    //    Serial.print(1000.0*adcMean(ii), 1);
    //}
    //Serial.println();


    digitalWriteFast(pins.aux1, LOW);

    // begin next pulse train
    return statePulsePositive;
}

//********************************
// ****** StepStateMachine *******

fsmState stepStateMachine(fsmState next_state)
{
    switch (next_state)
    {
    case statePowerUp:
        return powerUp();

    case statePulsePositive:
        return pulsePositive();

    case statePulseNegative:
        return pulseNegative();

    case stateInterPulse:
        return interPulse();

    case stateNotRunning:
        return notRunning();

    case stateComputeZ:
        return computeZ();

    default:
        Serial.println("Error: Unrecognized State");
        while (1) {
            digitalWriteFast(pins.aux2, HIGH);
            delay(500);
            digitalWriteFast(pins.aux2, LOW);
            delay(500);
        }
    }
}

//************************************
//**** Interrupt Service Routines ****
//************************************

void timerPulse_isr() { // IntervalTimer     
    timerCount++;       // increment counter
}

void timerAdc_isr() {
    digitalWriteFast(pins.aux2, HIGH);
    //  adc->adc0->startSingleDifferential(A10,A11);
    adc->adc0->startSingleRead(A0);
    digitalWriteFast(pins.aux2, LOW);
}

// called as soon as adc measurement is ready
void adc0_isr() {
    digitalWriteFast(pins.aux1, HIGH);

    //  adcRingBuffer->write(adc->adc0->readSingle()); // for differential

    // single-ended (uint16_t cast is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!)
    adcRingBuffer->write((uint16_t)adc->adc0->readSingle());

    digitalWriteFast(pins.aux1, LOW);
}

void button_isr() {
    static unsigned long debounceTime = 0;
    unsigned long currentTime = millis();

    // If interrupts come faster than N ms, assume it's a bounce and ignore
    if (currentTime - debounceTime > 250) {
        runFlag = !runFlag;

        if (!runFlag) {
            WRITE_RESTART(0x5FA0004);
        }
    }

    debounceTime = currentTime;
}

//************************************
//********* Other Functions **********
//************************************

void enableREF200(void) {
    // connects REF200 to Vcc
    digitalWriteFast(pins.ref200_EN, HIGH); // active HIGH
}

void disableREF200(void) {
    // disconnects REF200 from Vcc
    digitalWriteFast(pins.ref200_EN, LOW); // active HIGH
}

// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXf& X)
{
    int i, j, nrow, ncol;

    nrow = X.rows();
    ncol = X.cols();

    //   Serial.print("nrow: "); Serial.println(nrow);
    //   Serial.print("ncol: "); Serial.println(ncol);       
    //   Serial.println();

    for (i = 0; i<nrow; i++)
    {
        for (j = 0; j<ncol; j++)
        {
            Serial.print(X(i, j), 6);   // print 6 decimal places
            Serial.print(", ");
        }
        Serial.println();
    }
    Serial.println();
}

// PRINT MATRIX (double type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXd& X)
{
    int i, j, nrow, ncol;

    nrow = X.rows();
    ncol = X.cols();

    //   Serial.print("nrow: "); Serial.println(nrow);
    //   Serial.print("ncol: "); Serial.println(ncol);       
    //   Serial.println();

    for (i = 0; i<nrow; i++)
    {
        for (j = 0; j<ncol; j++)
        {
            Serial.print(X(i, j), 6);   // print 6 decimal places
            Serial.print(", ");
        }
        Serial.println();
    }
    Serial.println();
}

//********************************************************************
// SETUP
//********************************************************************

void setup() {
    // initialize finite state machine
    stateNext = stepStateMachine(statePowerUp);
}

//********************************************************************
// MAIN LOOP
//********************************************************************

void loop() {
    // run state machine
    stateNext = stepStateMachine(stateNext);
}