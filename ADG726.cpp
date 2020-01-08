#include <Arduino.h>
#include "ADG726.h"
#include <stdint.h>
#include <array>

ADG726::ADG726(const std::array<uint8_t,4> ADDR, const uint8_t CSA, const uint8_t CSB, const uint8_t WR, const uint8_t EN)
	: ADDR_(ADDR)
	, CSA_(CSA)
	, CSB_(CSB)
	, WR_(WR)
	, EN_(EN)
{	
}

ADG726::ADG726(const std::array<uint8_t,8> pins)
  : ADDR_({pins.at(0), pins.at(1), pins.at(2), pins.at(3)})
  , CSA_(pins.at(4))
  , CSB_(pins.at(5))
  , WR_(pins.at(6))
  , EN_(pins.at(7))
{}

bool ADG726::init(void)
{
	bool successful = false;
	
	// initialize pins (all active LOW)
	pinMode(EN_, OUTPUT);
	digitalWriteFast(EN_, HIGH);
	enabled_ = false;

	pinMode(CSA_, OUTPUT);
	digitalWriteFast(CSA_, HIGH);

	pinMode(CSB_, OUTPUT);
	digitalWriteFast(CSB_, HIGH);

	pinMode(WR_, OUTPUT);
	digitalWriteFast(WR_, HIGH);

	for (int ii=0; ii<4; ii++){
		pinMode(ADDR_[ii], OUTPUT);
		digitalWriteFast(ADDR_[ii], LOW);
	}
	
	successful = true;
	
	return successful;
}

bool ADG726::enable(void)
{
	digitalWriteFast(EN_, LOW);
	enabled_ = true;
	
	return true;
}

bool ADG726::disable(void)
{
	digitalWriteFast(EN_, HIGH);
	enabled_ = false;
	
	return true;
}
	
bool ADG726::selectA(uint8_t channel)
{
	bool successful = false;
	
	if(isValidChannel(channel)){
		select(CSA_, channel);
		SWA_ = channel; // update state
	}
	
	return successful;
}

bool ADG726::selectB(uint8_t channel)
{
	bool successful = false;
	
	if(isValidChannel(channel)){
		select(CSB_, channel);
		SWB_ = channel; // update state
	}
	
	return successful;
}

bool ADG726::select(uint8_t cs_pin, uint8_t channel)
{
	// NOTE: all control pins are active LOW!

	uint8_t addr = channel - 1; // S1 address = [0,0,0,0]

	// ensure WR starts disabled
	digitalWriteFast(WR_, HIGH);

	// set chip select
	digitalWriteFast(cs_pin, LOW);

	// set address
	for (int ii=0; ii<4; ii++){
		digitalWriteFast(ADDR_[ii], bitRead(addr, ii));
	}

	// enable WR
	digitalWriteFast(WR_, LOW);

	// short delay (minimum WR pulse width is 10 ns)
  // not needed for Teensy 3.2 -> time between writes is ~100ns
//	delayMicroseconds(1);

	// latch WR
	digitalWriteFast(WR_, HIGH);

	// disable chip selects
	digitalWriteFast(cs_pin, HIGH);

	return true;
}

bool ADG726::isValidChannel(int channel){
	// channels are numbers 1-16
	
	bool is_valid = false;
	
	if((channel>=1) && (channel <=16)){
		is_valid = true;
	}
	
	return is_valid;
}
