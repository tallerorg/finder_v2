#ifndef _SIMPLEDRIVER_H_INCLUDED
#define _SIMPLEDRIVER_H_INCLUDED

#define MIN_PWM_ANGLE			-127
#define MAX_PWM_ANGLE			127

class SimpleDriverClass {
public:
	SimpleDriverClass(uint8_t pwmPin, uint8_t umbral, int8_t sense);										// Simple driver analog output, like with the SaberTooth in analog mode -127 -> 0 and 127 -> 254 PWM output
	SimpleDriverClass(uint8_t enablePin, uint8_t dirPin, uint8_t umbral, int8_t sense);						// Dual driver output, with an Enable Pin and a Direction Pin, forwards is dirPin = LOW, -127 or +127 -> 254 PWM output
	SimpleDriverClass(uint8_t enablePin, uint8_t dir1Pin, uint8_t dir2Pin, uint8_t umbral, int8_t sense);  	// Tri driver output, like with and L293D, forwards is dir1Pin = LOW, dir2Pin = HIGH
																											// Angles less or equal than umbral are sent as 0s
	void write(int value);																					// Write a PWM angle from -127 to +127
	int read(); 																							// Returns current PWM as an angle between -127 and 127 degrees
	void stop();																							// Sets center (sugar for setting a PWM of 0)
private:
	bool isEnable : 1;																						// Is single PWM pin or has enable and dir(s)?
	bool isDualDir : 1;																						// Is a dual direction pin?
	int8_t value;																							// PWM angle from -127 to +127
	uint8_t enablepin;
	uint8_t dir1pin;
	uint8_t dir2pin;
	uint8_t umbral;
	int8_t sense;
};
#endif
