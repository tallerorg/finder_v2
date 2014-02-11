#include <Arduino.h>

#include "ros.h"
#include "finder/TwoWheelInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"

#include "OSMC.h"

//// (pinpwm1, pinpwm2, umbral, maxpwm_sense) outputs below umbral are taken as 0
OSMCClass motor1(3, 5, 1, 127);
OSMCClass motor2(9, 6, 1, 127);

ros::NodeHandle nh;

int input_vdes_1 = 0;			// speed in motor1
int input_vdes_2 = 0;			// speed in motor2


void vdes(const finder::TwoWheelInt16& dmsg) {
	input_vdes_1 = dmsg.wheel1;
	input_vdes_2 = dmsg.wheel2;
}
ros::Subscriber<finder::TwoWheelInt16> vdes_sub("traction_motors", vdes);

unsigned long milisLast = 0;

void setup() {
	nh.initNode();
	nh.subscribe(vdes_sub);

	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);

	digitalWrite(2, LOW);
	digitalWrite(4, LOW);
//	analogWrite(3, 150);
//	analogWrite(5, 0);
//	analogWrite(6, 150);
//	analogWrite(9, 0);
}

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();
	// 20 Hz / 50 ms operation is best, it seems
	if (milisNow - milisLast >= 50) {
		milisLast = milisNow;
		motor1.write((input_vdes_1));
		motor2.write((input_vdes_2));
	}
}
