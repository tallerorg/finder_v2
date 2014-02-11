#include <Arduino.h>

#include <SimpleSaber.h>

SimpleSaberClass::SimpleSaberClass(uint8_t pwmPin, uint8_t umbral, int8_t sense) {
	isAnalog = true;
	isAddress = false;
	pwmpin = pwmPin;

	this->umbral = umbral;
	this->sense = sense;

	// ENSURE MOTOR STARTS OFF!!!
	analogWrite(pwmpin, 127);
}

SimpleSaberClass::SimpleSaberClass(SaberSerialClass* saberSerial, uint8_t index, uint8_t umbral, int8_t sense) {
	isAnalog = false;
	isAddress = false;
	saberserial = saberSerial;

	this->umbral = umbral;
	this->sense = sense;

	this->index = index;
	address = START_SABER_ADDRESS;
}

SimpleSaberClass::SimpleSaberClass(SaberSerialClass* saberSerial, uint8_t index, uint8_t address, uint8_t umbral, int8_t sense) {
	isAnalog = false;
	isAddress = true;
	saberserial = saberSerial;

	this->umbral = umbral;
	this->sense = sense;

	this->index = index;
	this->address = address;
}

void SimpleSaberClass::write(int value) {
	value = constrain(value, MIN_PWM_ANGLE, MAX_PWM_ANGLE);
	value = constrain(value, -abs(sense), abs(sense));
	if (abs(value) <= umbral) value = 0;
	this->value = value;
	if (sense < 0) value = -value;

	if (isAnalog)
		analogWrite(pwmpin, 127 + value);
	else {
		// Make local copy of original address
		uint8_t orgAddress = saberserial->addressSaber;

		// If needed, set the address in this
		if (isAddress)
			saberserial->setAddress(address);

		// Send the motor data depending on index and value, if index is wrong do nothing
		if (index == 0)
			saberserial->motor1(value);
		if (index == 1)
			saberserial->motor2(value);

		// if needed, restore original address
		if (isAddress)
			saberserial->setAddress(orgAddress);
	}
}

int SimpleSaberClass::read() {
	return value;
}

void SimpleSaberClass::stop() {
	this->write(0);
}

void SimpleSaberClass::init() {
	if (!isAnalog) {
		// Make local copy of original address
		uint8_t orgAddress = saberserial->addressSaber;

		// if needed, set the address in this
		if (isAddress)
			saberserial->setAddress(address);

		// init the SaberTooth (also performs begin on saberserial)
		saberserial->init();

		// if needed, restore original address
		if (isAddress)
			saberserial->setAddress(orgAddress);
	}
	// Ensure all starts off!!!
	this->stop();
}
