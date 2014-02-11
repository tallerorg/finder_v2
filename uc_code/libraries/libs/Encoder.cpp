#include "Arduino.h"

#include "Encoder.h"

EncoderClass::EncoderClass(uint8_t anPin, int min, int max, int maxChange,
		int sense) {
	this->min = min;
	this->max = max;
	this->maxChange = maxChange;
	this->sense = sense;

	isAnalog = true;
	pinanalog = anPin;
}

EncoderClass::EncoderClass(AS5043Class* AS5043, uint8_t pinCSn, int sense) {
	this->min = 0;
	this->max = MODULE_RANGE - 1;
	this->sense = sense;

	isAnalog = false;

	as5043 = AS5043;
	index = 0;
	pincsn = pinCSn;

	// ENSURE CHAIN START DESELECTED!
	pinMode(pinCSn, OUTPUT);
	digitalWrite(pinCSn, HIGH);
}

EncoderClass::EncoderClass(AS5043Class* AS5043, uint8_t pinCSn, int min,
		int max, int sense) {
	this->min = min;
	this->max = max;
	this->sense = sense;

	isAnalog = false;

	as5043 = AS5043;
	index = 0;
	pincsn = pinCSn;

	// ENSURE CHAIN STARTS DESELECTED!
	pinMode(pinCSn, OUTPUT);
	digitalWrite(pinCSn, HIGH);
}

EncoderClass::EncoderClass(AS5043Class* AS5043, uint8_t pinCSn,
		uint8_t indexChain, int sense) {
	this->min = 0;
	this->max = MODULE_RANGE - 1;
	this->sense = sense;

	isAnalog = false;

	as5043 = AS5043;
	index = max(indexChain, MAX_AS5043_DEVICES - 1);
	pincsn = pinCSn;

	// ENSURE CHAIN STARTS DESELECTED!
	pinMode(pinCSn, OUTPUT);
	digitalWrite(pinCSn, HIGH);
}

EncoderClass::EncoderClass(AS5043Class* AS5043, uint8_t pinCSn,
		uint8_t indexChain, int min, int max, int sense) {
	this->min = min;
	this->max = max;
	this->sense = sense;

	isAnalog = false;

	as5043 = AS5043;
	index = max(indexChain, MAX_AS5043_DEVICES - 1);
	pincsn = pinCSn;

	// ENSURE CHAIN STARTS DESELECTED!
	pinMode(pinCSn, OUTPUT);
	digitalWrite(pinCSn, HIGH);
}

// Reads sensor and returns 10 bit ANGLE stream or the RAW analogRead
unsigned int EncoderClass::read() {
	if (isAnalog) {
		lecture = analogRead(pinanalog);
	} else {
		// Make local copy of pincsn and maxdevices
		uint8_t pinCSnCopy = as5043->pincsn;
		uint8_t maxDevicesCopy = as5043->maxdevices;

		// Establish pinCSn as the one in this and maxdevices to a convenient number (faster reads)
		as5043->setPinCSn(pincsn);
		as5043->maxdevices = index + 1;

		// Read the sensors
		as5043->read();

		// Extract the value of the angle at index
		lecture = as5043->getAngle(index);

		// Restore pinCSn and maxdevices
		as5043->setPinCSn(pinCSnCopy);
		as5043->maxdevices = maxDevicesCopy;
	}
	// Calculate the angle always at each read operation
	this->getAngleAtom();
	return lecture;
}

// Performs read and returns angle between -abs_sense and +abs_sense
int EncoderClass::readAngle() {
	this->read();
	return angle;
}

int EncoderClass::readChange() {
	this->read();
	return this->getChange();
}

// Gets last read 10 bit ANGLE stream or the RAW analogRead saved in lecture
unsigned int EncoderClass::get() {
	return lecture;
}

int EncoderClass::getAngle() {
	return angle;
}

// Returns angle between -abs_sense and +abs_sense from last read
void EncoderClass::getAngleAtom() {
	// As said in the header file, if sense == 0 return 0
	if (sense == 0)
		return;

	// Now, here we suppose that the lectures all are "conforming", this is, either we have the analogRead or the AS5043
	// 10 bit angle in the lecture variable. We have to apply the min max stuff then. First, we must check whether
	// (lecture - min) mod 1024 (the positive value) is within range, where range is max - min or min - max mod 1024,
	// depending on the sense sign. Of course, we want the positive value for the range.

	// Calculate the range depending on sense sign and fix module 1024 if negative
	int range = sense > 0 ? max - min : min - max;
	if (range < 0)
		range += MODULE_RANGE;

	// Calculate the distance from min or max, depending on sense and fix module 1024 if negative
	int distance = sense > 0 ? lecture - min : min - lecture;
	if (distance < 0)
		distance += MODULE_RANGE;

	// Pre-check if we are dealing with a value outside normal range, we must get the "closest" side
	int minside =
			min(abs(lecture - min),
					min(abs(lecture - min + MODULE_RANGE), abs(lecture - min - MODULE_RANGE)));
	int maxside =
			min(abs(lecture - max),
					min(abs(lecture - max + MODULE_RANGE), abs(lecture - max - MODULE_RANGE)));

	// If we are dealing with a value within range OR the closest side is maxside, we can go as usual
	// and return the mapping, the only correction is needed in the case we are OUTSIDE the range AND
	// the closest side is the minside, because we would be better served by obtaining a NEGATIVE
	// distance, but we ALREADY know distance is in 0/1023, then we only have to do:
	if ((minside < maxside) && (distance > range))
		distance -= MODULE_RANGE;

	// In analog mode, correct if change was too great because of jump through zero and filter delay
	// basically, just ignore data points unitl change is within ok range, the angle returned is then the
	// last angle that was valid
	int tempangle = map(distance, 0, range, -abs(sense), abs(sense));
	if (isAnalog) {
		if (abs(sense) - abs(angle) <= maxChange) {
			// lastAngle is near transition zone, angle could trigger the transition condition
			if ( abs(tempangle - angle) > maxChange) {
				// the change was too abrupt, we are in the transition zone, then we should not update
				// the angle value, maybe we can simply return
				return;
			}
		}
	}

	// And save the mapped value
	lastAngle = angle;
	angle = tempangle;
}

int EncoderClass::getChange() {
	// Now, getting the change is somewhat interesting. Lets say that in the last lecture we read 230, and in the next lecture we read 500.
	// How do we know that we did not went backwards then? If we assume that in any single read it's not possible to turn more than HALF a
	// full turn, then it's a simple thing:
	//
	// Since all values should be between -127 and +127, and we saved them in angle and lastAngle, we could do
	// change = angle - lasAngle
	//
	// The only problem is when passing, lets say, from +127 to -127 (or viceversa if in reverse), then
	//
	// change = -127 - 127 = -254
	//
	// However, if we assume that in a single lecture, the change module 255 cannot exceed 127 (HALF the turn), than we could
	//
	// change = -254 mod 255 == +1 mod 255
	//
	// Which is the expected value. The, we only have to eval the minimal (in absolute value) mod 255 and return that.
	// Obviusly, if in a lecture this jump is greater than 127 all will go wrong

	int change = angle - lastAngle;

	// Correct in each possible case
	if (change > abs(sense))
		change -= 2 * abs(sense) + 1;
	if (change < -abs(sense))
		change += 2 * abs(sense) + 1;

	return change;
}

unsigned int EncoderClass::singleAlign() {
	if (isAnalog)
		return 0;

	// Make local copy of pincsn
	uint8_t pinCSnCopy = as5043->pincsn;

	// Establish pinCSn as the one in this
	as5043->setPinCSn(pincsn);

	// Read the censors in alignment mode
	unsigned int value = as5043->singleAlign();

	// Restore pinCSn
	as5043->setPinCSn(pinCSnCopy);

	// return value
	return value;
}

void EncoderClass::softProg(unsigned int data) {
	if (isAnalog)
		return;

	// Make local copy of pincsn
	uint8_t pinCSnCopy = as5043->pincsn;

	// Establish pinCSn as the one in this
	as5043->setPinCSn(pincsn);

	// Send data
	as5043->softProg(data);

	// Restore pinCSn
	as5043->setPinCSn(pinCSnCopy);
}

void EncoderClass::begin() {
	if (!isAnalog) {
		// Make local copy of pincsn
		uint8_t pinCSnCopy = as5043->pincsn;

		// Establish pinCSn as the one in this
		as5043->setPinCSn(pincsn);

		// start the class
		as5043->begin();

		// Restore pinCSn
		as5043->setPinCSn(pinCSnCopy);
	}
	this->readAngle();
}
