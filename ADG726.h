/*=========================================================================//

	ADG726 Multiplexer Class	
	
	
	Designed for use with Teensy 3.2 and Teensyduino

	
	Trevor Bruns
	
	Changelog-
		10/26/2019: Initial creation

//=========================================================================*/


#ifndef ADG726_h
#define ADG726_h

#include <stdint.h>
#include <array>

class ADG726
{
public:
	ADG726(const std::array<uint8_t,4> ADDR, const uint8_t CSA, const uint8_t CSB, const uint8_t WR, const uint8_t EN);
    ADG726(const std::array<uint8_t,8> pins); // same order as above
	bool init(void);
	bool enable(void);
	bool disable(void);
	bool selectA(uint8_t channel);
	bool selectB(uint8_t channel);	
	bool isEnabled(void) {return enabled_;}
	uint8_t A(void) {return SWA_;}
	uint8_t B(void) {return SWB_;}
	
private:
	const std::array<uint8_t,4> ADDR_;
	const uint8_t CSA_;
	const uint8_t CSB_;
	const uint8_t WR_;
	const uint8_t EN_;
	
	bool enabled_;
	uint8_t SWA_; // currently selected switch A channel
	uint8_t SWB_; // currently selected switch B channel
	
	bool select(uint8_t cs_pin, uint8_t channel);
	bool isValidChannel(int channel);
};

#endif
