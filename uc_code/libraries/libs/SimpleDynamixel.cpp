#include "Arduino.h"

#include "SimpleDynamixel.h"

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int min, int max, int sense) {
	isDir = false;
	dynamixelobj = dynamixelSerial;
	id = 1;

	value = 0;

	this->min = min;
	this->max = max;
	this->sense = sense;
}

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, uint8_t ID, int min, int max, int sense) {
	isDir = false;
	dynamixelobj = dynamixelSerial;
	id = ID;

	value = 0;

	this->min = min;
	this->max = max;
	this->sense = sense;
}

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, uint8_t ID, uint8_t dirPin, int min, int max, int sense) {
	isDir = true;
	dynamixelobj = dynamixelSerial;
	id = ID;

	dirpin = dirPin;
	value = 0;

	this->min = min;
	this->max = max;
	this->sense = sense;
}

void SimpleDynamixelClass::write(int value) {
	value = constrain(value, -abs(sense), abs(sense));
	this->value = value;
	if (abs(sense) < 0) value = -value;
	value = map(value, -abs(sense), abs(sense), min, max);

	// Make local copy of direction pin
	uint8_t orgdirpin = dynamixelobj->Direction_Pin;
	// Set
	if (isDir)
		dynamixelobj->setDirPin(dirpin);
	// Move
	dynamixelobj->move(id, value);
	// Reset
	if (isDir)
		dynamixelobj->setDirPin(orgdirpin);
}

int SimpleDynamixelClass::read() {
	return value;
}

void SimpleDynamixelClass::stop() {
	this->write(0);
}

void SimpleDynamixelClass::init() {
	// Make local copy of direction pin
	uint8_t orgdirpin = dynamixelobj->Direction_Pin;
	// Begin comm with this dirpin at 1MHz
	if (isDir)
		dynamixelobj->setDirPin(dirpin);
	dynamixelobj->begin(1000000UL);
	// Set parameters of dynamixel
	dynamixelobj->setTempLimit(id, 80);
	dynamixelobj->setVoltageLimit(id, 65, 100);
	dynamixelobj->setMaxTorque(id, 512);
	dynamixelobj->ledStatus(id, ON);
	// Restore direction pin
	if (isDir)
		dynamixelobj->setDirPin(orgdirpin);
}
