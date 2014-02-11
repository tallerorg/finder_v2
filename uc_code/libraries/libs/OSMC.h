#ifndef _OSMC_H_INCLUDED
#define _OSMC_H_INCLUDED

#define MIN_PWM_ANGLE			-127
#define MAX_PWM_ANGLE			127

class OSMCClass {
public:
	OSMCClass(uint8_t pinDer, uint8_t pinIzq, uint8_t umbral, int8_t sense);
	OSMCClass(uint8_t pinDer, uint8_t pinIzq, uint8_t pinDis, uint8_t umbral, int8_t sense);
	void write(int value);																					// Write a PWM angle from -127 to +127
	int read(); 																							// Returns current PWM as an angle between -127 and 127 degrees
	void stop();																							// Sets center (sugar for setting a PWM of 0)
	void disable();
	void enable();
private:
	int8_t value;																							// PWM angle from -127 to +127
	bool isDisable;
	uint8_t disablepin;
	uint8_t pinder;
	uint8_t pinizq;
	uint8_t umbral;
	int8_t sense;
};
#endif
