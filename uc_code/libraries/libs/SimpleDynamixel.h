#ifndef _SIMPLESABER_H_INCLUDED
#define _SIMPLESAVER_H_INCLUDED

#include "DynamixelSerial.h"

class SimpleDynamixelClass {
public:
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int min, int max, int sense);
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, uint8_t ID, int min, int max, int sense);
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, uint8_t ID, uint8_t dirPin, int min, int max, int sense);
	void write(int value);																// Write an angle from -val->min to +val->max
	int read(); 																		// Returns current position as an angle between -val and val degrees
	void stop();																		// Sets center (sugar for setting an angle of 0)
	void init();																		// Inits Dynamixel object and sets servo values, max torque, max current, et al...
private:
	int min;
	int max;
	int sense;
	int value;
	DynamixelClass* dynamixelobj;														// Inheriting is not done because there may be one or more instances of DynamixelClass, let the user decide
	bool isDir : 1;
	uint8_t id : 3;
	uint8_t dirpin;
};
#endif
