#include "Arduino.h"

#include "OSMC.h"

OSMCClass::OSMCClass(uint8_t pinDer, uint8_t pinIzq, uint8_t umbral, int8_t sense) {
	pinder = pinDer;
	pinizq = pinIzq;

	isDisable = false;

	this->umbral = umbral;
	this->sense = sense;

	value = 0;

	// ENSURE MOTOR STARTS OFF!!!
	analogWrite(pinder, 0);
	analogWrite(pinizq, 0);
}

OSMCClass::OSMCClass(uint8_t pinDer, uint8_t pinIzq, uint8_t pinDis, uint8_t umbral, int8_t sense) {
	pinder = pinDer;
	pinizq = pinIzq;

	isDisable = true;
	disablepin = pinDis;

	this->umbral = umbral;
	this->sense = sense;

	value = 0;

	// ENSURE MOTOR STARTS OFF!!! ALSO ENABLED BY DEFAULT
	analogWrite(pinder, 0);
	analogWrite(pinizq, 0);
	digitalWrite(disablepin, LOW);
}

void OSMCClass::write(int value) {
	value = constrain(value, MIN_PWM_ANGLE, MAX_PWM_ANGLE);
	value = constrain(value, -abs(sense), abs(sense));
	if (abs(value) <= umbral) value = 0;
	this->value = value;
	if (sense < 0) value = -value;

	if (value > 0) {
		analogWrite(pinder, 2 * abs(value));
		analogWrite(pinizq, 0);
	}

	if (value == 0) {
		analogWrite(pinder, 0);
		analogWrite(pinizq, 0);
	}

	if (value < 0) {
		analogWrite(pinder, 0);
		analogWrite(pinizq, 2 * abs(value));
	}

	if (isDisable) digitalWrite(disablepin, LOW);

}

int OSMCClass::read() {
	return value;
}

void OSMCClass::stop() {
	this->write(0);
}

void OSMCClass::disable() {
	if (isDisable) digitalWrite(disablepin, HIGH);
}

void OSMCClass::enable() {
	if (isDisable) digitalWrite(disablepin, LOW);
}
