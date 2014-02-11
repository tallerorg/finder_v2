#include <Arduino.h>

#include "Servo.h"
#include "ros.h"
#include "finder/FourArmsInt16.h"
#include "Talon.h"

ros::NodeHandle nh;

TalonClass DOF1(2, 127);
TalonClass DOF2(2, 127);
TalonClass DOF3(2, 127);
TalonClass DOF4(2, 127);

// The incoming 6 DOF int16 information from ROS
int d1_desired = 0;
int d2_desired = 0;
int d3_desired = 0;
int d4_desired = 0;

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void arms_h(const finder::FourArmsInt16& d_msg) {
	d1_desired = d_msg.arm1;
	d2_desired = d_msg.arm2;
	d3_desired = d_msg.arm3;
	d4_desired = d_msg.arm4;
}
ros::Subscriber<finder::FourArmsInt16> sub_arm("traction_arms", arms_h);

unsigned long milisLast = 0;

void setup() {
	DOF1.attach(3);
	DOF2.attach(5);
	DOF3.attach(6);
	DOF4.attach(9);
	nh.initNode();
	nh.subscribe(sub_arm);
}

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();
	// 20 Hz operation
	if (milisNow - milisLast >= 50) {
		milisLast = milisNow;
		DOF1.write(d1_desired);
		DOF2.write(d2_desired);
		DOF3.write(d3_desired);
		DOF4.write(d4_desired);
	}
}

