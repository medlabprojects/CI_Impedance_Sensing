#include <stdint.h>
#include <utility>
#include <array>
#include <ADC.h>
#include "RingBuffer.h"
#include <IntervalTimer.h>
#include <Eigen.h>
#include <Eigen/Dense>
//#include <SFE_MicroOLED.h>
#include "ADG726.h"
//#include "CI_Impedance_pins_breadboard.h"
#include "CI_Impedance_pins.h"
#include "CI_Impedance_fsm.h"

using namespace Eigen;

ImpedanceSensingPins pins; // pin mappings

ADG726 mux(pins.mux_pins); // mux object

using ZPair = std::pair<uint8_t, uint8_t>; // pair of EA contacts (anode, cathode) to use for bipolar impedance measurement
const std::array<ZPair, 5> zpairs = { {    // array of all pairs to test
    { pins.EA[1], pins.EA[2] },
    { pins.EA[2], pins.EA[3] },
    { pins.EA[3], pins.EA[4] },
    { pins.EA[4], pins.EA[5] },
    { pins.EA[5], pins.EA[6] }
    } };
//const std::array<ZPair, 2> zpairs = { {    // array of all pairs to test
//    {pins.EA[2], pins.EA[3]},
//    {pins.EA[3], pins.EA[5]}
//} };
//const std::array<ZPair, 1> zpairs = { { { pins.EA[2], pins.EA[3] } } };
auto current_zpair = zpairs.begin(); // current pair being tested (iterator -> *current_zpair to access pair)


IntervalTimer timerPulse;
const int pulse_time = 55;       // [us] time for each current pulse (+/-); total time for biphasic pulse is 2*pulse_time
const int interPulseDelay = 0;   // multiples of pulse_time between pulse trains ([us] = pulse_time * interPulseDelay)
volatile bool timerFlag = false;
volatile int timerCount = 0;

IntervalTimer timerAdc;    // timer for triggering ADC samples
const float adcTime = 8.5; // [us] time between each adc trigger
const int nSamples = 5;    // number of ADC samples to take during positive pulse
const int nPulses = 16;    // number of pulse trains which will be sampled and averaged together for each single output measurement

ADC *adc = new ADC(); // ADC object
RingBuffer *adcRingBuffer = new RingBuffer; // buffer to capture adc samples (changed size in .h to 16 elements)
MatrixXf adcRaw(nSamples, nPulses); // stores raw (integer) values from adc; float because .mean() method is later called
volatile int adcCount = 0; // current sample number (row index of adcRaw)
volatile int pulseCount = 0; // current pulse number (column index of adcRaw)
bool adcFlag = false;
double adc2Voltage = 0.0;

MatrixXf Alinfit(nSamples, 2); // linear regression matrix for line fitting
double resistance = 0.0; // resistive component of measured impedance
double capacitance = 0.0;// capacitive component of measured impedance

const float filter_sigma = 4.0; // samples more than this many std deviations from the mean will be filtered out
const float filter_variance = filter_sigma * filter_sigma; // variance is actually used since it is faster to compute 
volatile bool bad_sample = false;

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
    
    // ensure EA is disconnected
    disableREF200();
    mux.selectA(pins.EA[0]);
    mux.selectB(pins.EA[0]);
    mux.enable();
   

    //attachInterrupt(pins.buttonPin, button_isr, FALLING);


    // set up linear regression matrix
    Alinfit.col(0) = VectorXf::Ones(nSamples);
    Alinfit.col(1) = VectorXf::LinSpaced(nSamples, 1, nSamples);
    //print_mtxf(Alinfit);


    //  setup ADC
    adc->setAveraging(0); // no averaging; take single samples
    adc->setResolution(16); // resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // sets ADCK to highest speed within spec for all resolutions
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);            // HIGH_SPEED adds +6 ADCK; MED_SPEED adds +10 ADCK
    adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);                 // use 3.3V internal reference
    adc2Voltage = 3.3 / adc->adc0->getMaxValue();                     // conversion factor from adc counts to voltage
    adc->adc0->analogRead(pins.adc_EA);                               // call once to setup
    adc->enableInterrupts(ADC_0);

    Serial.print("adc2Voltage = ");
    Serial.println(adc2Voltage, 10);

    Serial.print("adc_max_val = ");
    Serial.println(adc->adc0->getMaxValue());

    Serial.print("adcTime = ");
    Serial.println(adcTime);

    Serial.print("nSamples = ");
    Serial.println(nSamples);

    Serial.print("nPulses = ");
    Serial.println(nPulses);

    Serial.println("\nInitialization Complete");

    return stateNotRunning;
}

//*************************
// ****** notRunning ******

fsmState notRunning(void) {
    stateCurrent = stateNotRunning;

    // ensure EA is disconnected
    disableREF200();
    mux.selectA(pins.EA[0]);
    mux.selectB(pins.EA[0]);
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

    //while (!Serial.dtr());  // wait for user to start the serial monitor

    while ((cmd != 's') && (cmd != 't') && (!runFlag)) {
        if (Serial.available() > 0) {
            cmd = Serial.read();
        }
    }

    if (cmd == 't') {
        //ch_anode = sw_test1;
        //ch_cathode = sw_test2;
    }

    // reset zpair iterator and select first pair
    current_zpair = zpairs.begin();
    mux.selectA(current_zpair->first);
    mux.selectB(current_zpair->second);


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
    delayMicroseconds(pulse_time - 20); // wait until just before start of pulse
    digitalWriteFast(pins.short_EA, LOW);

    // wait for first pulse to sync up timing
    while (timerCount == 0);


    // start positive pulse (current from ch_anode to ch_cathode)
    enableREF200(); // turn on current source
    
    digitalWriteFast(pins.aux2, HIGH);

    // start triggering ADC
    // takes around (1.6 + adcTime + 2.4) [us] to set up IntervalTimer,  wait for first trigger, and start first adc measurement
    timerAdc.begin(timerAdc_isr, adcTime);

    digitalWriteFast(pins.aux2, LOW);


    return statePulseNegative;
}

//*****************************
// ****** pulseNegative *******

fsmState pulseNegative(void) {
    stateCurrent = statePulseNegative;

    // wait for positive pulse to finish
    while (timerCount < 2);
    
    // halt triggering
    timerAdc.end(); 

    // start negative pulse (current from cathode to anode)
    disableREF200();
    mux.disable(); // avoids temporary short
    mux.selectA(current_zpair->second);
    mux.selectB(current_zpair->first);
    mux.enable();
    enableREF200();

    // store adc samples
    while (!adcRingBuffer->isEmpty()) 
    {
        noInterrupts(); // ensure ring buffer isn't modified during read
        int adcLast = adcRingBuffer->read(); // read sample from buffer
        interrupts();

        if (adcCount < nSamples) {
            //// check for bad sample
            //if ((adcLast < 100) || (adcLast > 60000)) {
            //    digitalWriteFast(pins.aux2, HIGH);
            //    bad_sample = true;
            //    digitalWriteFast(pins.aux2, LOW);
            //}

            adcRaw(adcCount, pulseCount) = adcLast; // store

            adcCount++; // increment count
        }
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
    mux.selectA(current_zpair->first);
    mux.selectB(current_zpair->second);
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

    // stop timers
    timerPulse.end();
    timerAdc.end();


    // reset pulse count
    pulseCount = 0;




    digitalWriteFast(pins.aux1, HIGH);

    // convert to voltages and average together samples taken at same time during pulses (i.e. rows of adcRaw)
    MatrixXf rawVoltages(nSamples, nPulses);
    rawVoltages = adc2Voltage * adcRaw;

    VectorXf meanVoltages(nSamples);
    meanVoltages = rawVoltages.rowwise().mean();

    // compute variance
    VectorXf varianceVoltages(nSamples);          // sample variances [volts]
    MatrixXf dev_from_mean(nSamples, nPulses);    // sample deviations from the mean
    MatrixXf sq_dev_from_mean(nSamples, nPulses); // squared deviations from the mean
    const float inv_nSamples = 1.0 / (float)nSamples; // inverse of nSamples

    dev_from_mean = rawVoltages.colwise() - meanVoltages;
    sq_dev_from_mean = dev_from_mean.cwiseProduct(dev_from_mean);
    varianceVoltages = inv_nSamples * sq_dev_from_mean.rowwise().sum();


    // fit least-squares line to data
    Vector2f fit;
    fit = Alinfit.colPivHouseholderQr().solve(meanVoltages);

    // compute resistive component (via intercept of fit)
    // I = 100E-6 [A] ==> 1/I = 1E4 [A]
    resistance = fit(0) * 1.0E4;


    // compute capacitive component (via slope of fit)
    // C[nF] = (0.1[ma] * adcTime[us/sample]) / (slope[V/sample])
    capacitance = 0.1 * adcTime / fit(1);

    digitalWriteFast(pins.aux1, LOW);


    // check is there were outliers
    if (bad_sample)
        {
        // print means
        Serial.print("\nm");
        for (int ii = 0; ii<nSamples; ii++) {
            Serial.print(", ");
            Serial.print(1000.0*meanVoltages(ii), 2);
        }
        Serial.println();

        // print variances
        Serial.print("v");
        for (int ii = 0; ii<nSamples; ii++) {
            Serial.print(", ");
            Serial.print(1000.0*varianceVoltages(ii), 3);
        }
        Serial.println();

        // print raw data
        MatrixXf adcRawTpose(nPulses, nSamples);
        adcRawTpose = adcRaw.transpose();
        print_mtxf(adcRawTpose,0);

        bad_sample = false; // reset flag
    }


    // send results
    Serial.print(resistance);


    //for (int ii = 0; ii<nSamples; ii++) {
    //    Serial.print(" ,");
    //    Serial.print(1000.0* meanVoltages(ii), 1);
    //}

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

    // increment to next channel pair
    if (++current_zpair < zpairs.end()) {
        Serial.print(","); 
    }
    else {
        Serial.print("\n");
        current_zpair = zpairs.begin(); // reset to first pair
        Serial.send_now(); // ensure data is sent now and not buffered
    }

    mux.selectA(current_zpair->first);
    mux.selectB(current_zpair->second);

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
    //digitalWriteFast(pins.aux2, HIGH);
    adc->adc0->startSingleRead(pins.adc_EA);
    //digitalWriteFast(pins.aux2, LOW);
}

// called as soon as adc measurement is ready
void adc0_isr() {
    digitalWriteFast(pins.aux1, HIGH);

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
void print_mtxf(const Eigen::MatrixXf& X, uint8_t precision = 2)
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
            Serial.print(X(i, j), precision);
            Serial.print(", ");
        }
        Serial.println();
    }
    Serial.println();
}

// PRINT MATRIX (double type)
// By: randomvibe
//-----------------------------
void print_mtxf(const Eigen::MatrixXd& X, uint8_t precision = 2)
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
            Serial.print(X(i, j), precision);
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