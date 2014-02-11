#include "Arduino.h"
#include "SimpleServo.h"

void SimpleServoClass::attach(uint8_t servoPin) {
	servo.attach(servoPin, min, max);
}

SimpleServoClass::SimpleServoClass(int min, int max, int8_t sense) : servo() {
	this->min = min;
	this->max = max;
	this->sense = sense;
}

SimpleServoClass::SimpleServoClass(int8_t sense) : servo() {
	min = MIN_SERVO_DEFAULT;
	max = MAX_SERVO_DEFAULT;
	this->sense = sense;
}

void SimpleServoClass::write(int value) {
	if (value < MIN_PULSE_WIDTH) {
		value = constrain(value, -90, 90);
		value = constrain(value, -abs(sense), abs(sense));
		if (sense < 0) value = -value;
		value = map(value, -90, 90, min, max);
	}
	this->writeMicroseconds(value);
}

void SimpleServoClass::writeMicroseconds(int value) {
	servo.writeMicroseconds(value);
}

int SimpleServoClass::read() {
	int value = this->readMicroseconds() + 1;
	value = map(value, min, max, -90, 90);
	if (sense < 0) value = -value;
	return value;
}

int SimpleServoClass::readMicroseconds() {
	return servo.readMicroseconds();
}

void SimpleServoClass::stop() {
	this->write(0);
}

