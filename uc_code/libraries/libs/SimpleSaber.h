#ifndef _SIMPLESABER_H_INCLUDED
#define _SIMPLESAVER_H_INCLUDED

#include "SaberSerial.h"

#define MIN_PWM_ANGLE			-127
#define MAX_PWM_ANGLE			127

class SimpleSaberClass {
public:
	SimpleSaberClass(uint8_t pwmPin, uint8_t umbral, int8_t sense);														// Control of SaberTooth by filtered PWM output, next by serial packets
	SimpleSaberClass(SaberSerialClass* saberSerial, uint8_t index, uint8_t umbral, int8_t sense);						// The SaberSerialClass object "knows" the address (usually because there is a single SaberTooth at 128), index must be 0 or 1
	SimpleSaberClass(SaberSerialClass* saberSerial, uint8_t address, uint8_t index, uint8_t umbral, int8_t sense);		// Use the saberSerial object but change its address on the fly (usually because there are multiple SaberTooths on the serial line)
																						// The original address is restored at the end of all init, write or stop calls
																						// Even though its possible to instantiate many SaberSerialClass objects (one for each address and/or line), this is memory expensive
																						// It's better to put the Sabers on the same TX line and control them using the same SaberSerial object, assigning each an address
	void write(int value);																// Write a PWM angle from -127 to +127 (-127 is full reverse and +127 is full forwards)
	int read(); 																		// Returns current PWM as an angle between -127 and 127 degrees
	void stop();																		// Sets center (sugar for setting a PWM angle of 0)
	void init();																		// Call saberserial init method at right address (though it would be better if the init method of saberserial is called instead
																						// if there is a single SaberSerialClass instance on the TX line for a single SaberTooth) and sets up the reamping, timeOut and deadband
																						// if in analog mode, just writes a 0 on pwmpin
private:
	SaberSerialClass* saberserial;														// Inheriting is not done because there may be a one or more instances of SaberSerialClass across all instances of this, let user decide
	bool isAnalog : 1;																	// Is an analog saber?
	bool isAddress : 1;																	// Each call of init, write or stop sets the saberserial address to this, then is restored to original
	uint8_t address;																	// Desired address at TX serial line in serial mode
	int value;																			// PWM angle from -127 to +127, in analog +127 -> 254 and -127 -> 0 PWM output
	uint8_t index;																		// If index is 0 sets motor 1, if 1 sets motor 2 (I know, but follows standards)
	uint8_t pwmpin;																		// The pin to use in analog mode
	uint8_t umbral;
	int8_t sense;
};
#endif
