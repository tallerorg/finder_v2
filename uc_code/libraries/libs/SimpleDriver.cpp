#include "Arduino.h"

#include "SimpleDriver.h"

SimpleDriverClass::SimpleDriverClass(uint8_t pwmPin, uint8_t umbral, int8_t sense) {
	isEnable = false;
	isDualDir = false;
	enablepin = pwmPin;
	this->umbral = umbral;
	this->sense = sense;

	// ENSURE MOTOR STARTS OFF!!!
	analogWrite(enablepin, 127);
}

SimpleDriverClass::SimpleDriverClass(uint8_t enablePin, uint8_t dirPin, uint8_t umbral, int8_t sense) {
	isEnable = true;
	isDualDir = false;
	enablepin = enablePin;
	dir1pin = dirPin;
	this->umbral = umbral;
	this->sense = sense;

	// ENSURE MOTOR STARTS OFF AND FORWARDS!!!
	analogWrite(enablepin, 0);
	pinMode(dir1pin, OUTPUT);
	digitalWrite(dir1pin, LOW);
}

SimpleDriverClass::SimpleDriverClass(uint8_t enablePin, uint8_t dir1Pin, uint8_t dir2Pin, uint8_t umbral, int8_t sense) {
	isEnable = true;
	isDualDir = true;
	enablepin = enablePin;
	dir1pin = dir1Pin;
	dir2pin = dir2Pin;
	this->umbral = umbral;
	this->sense = sense;

	// ENSURE MOTOR STARTS OFF AND FORWARDS!!!
	analogWrite(enablepin, 0);
	pinMode(dir1pin, OUTPUT);
	digitalWrite(dir1pin, LOW);
	pinMode(dir2pin, OUTPUT);
	digitalWrite(dir2pin, HIGH);

}

void SimpleDriverClass::write(int value) {
	value = constrain(value, MIN_PWM_ANGLE, MAX_PWM_ANGLE);
	value = constrain(value, -abs(sense), abs(sense));
	if (abs(value) <= umbral) value = 0;
	this->value = value;
	if (sense < 0) value = -value;
	if (isEnable) {
		digitalWrite(dir1pin, value >= 0 ? LOW : HIGH);
		if (isDualDir)
			digitalWrite(dir2pin, value >= 0 ? HIGH : LOW);
		analogWrite(enablepin, 2 * abs(value));
	} else {
		analogWrite(enablepin, 127 + value);
	}
}

int SimpleDriverClass::read() {
	return value;
}

void SimpleDriverClass::stop() {
	this->write(0);
}
