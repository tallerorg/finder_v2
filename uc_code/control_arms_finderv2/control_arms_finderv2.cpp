#include <Arduino.h>

#include "Servo.h"
#include "ros.h"
#include "finder/FourArmsInt16.h"
#include "Talon.h"
#include "AS5043.h"
#include "Encoder.h"
#include "SimplePID.h"

ros::NodeHandle nh;

TalonClass DOF1(2, 127);
TalonClass DOF2(2, 127);
TalonClass DOF3(2, 127);
TalonClass DOF4(2, 127);

AS5043Class AS5043obj(13, 12, 11);
EncoderClass ENC1(&AS5043obj, A0, 0, 1023, 511);
EncoderClass ENC2(&AS5043obj, A1, 0, 1023, 511);
EncoderClass ENC3(&AS5043obj, A2, 0, 1023, 511);
EncoderClass ENC4(&AS5043obj, A3, 0, 1023, 511);

SimplePID pid1(.1, .1, .1, 0, 1);
SimplePID pid2(.1, .1, .1, 0, 1);
SimplePID pid3(.1, .1, .1, 0, 1);
SimplePID pid4(.1, .1, .1, 0, 0);

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

// All the pubs msgs, keep names short
finder::FourArmsInt16 armsOut;
ros::Publisher armsOut_f("arms_lecture", &armsOut);

unsigned long milisLast = 0;

void setup() {
	DOF1.attach(7);
	DOF2.attach(8);
	DOF3.attach(9);
	DOF4.attach(10);

	nh.initNode();

	nh.subscribe(sub_arm);
	nh.advertise(armsOut_f);

	AS5043obj.begin();

	// Initialize desired values to ACTUAL values (to avoid jerk)
	d1_desired = ENC1.begin();
	d2_desired = ENC2.begin();
	d3_desired = ENC3.begin();
	d4_desired = ENC4.begin();
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	// 20 Hz operation
	if (milisNow - milisLast >= 50) {

		int dof1lec = ENC1.readAngle();
		int dof2lec = ENC2.readAngle();
		int dof3lec = ENC3.readAngle();
		int dof4lec = ENC4.readAngle();

		pid1.compute(dof1lec, d1_desired);
		pid2.compute(dof2lec, d2_desired);
		pid3.compute(dof3lec, d3_desired);
		pid4.compute(dof4lec, d4_desired);

		DOF1.write(pid1.get());
		DOF2.write(pid2.get());
		DOF3.write(pid3.get());
		DOF4.write(pid4.get());

		armsOut.arm1 = dof1lec;
		armsOut.arm2 = dof2lec;
		armsOut.arm3 = dof3lec;
		armsOut.arm4 = dof4lec;

		armsOut_f.publish(&armsOut);

		milisLast = milisNow;
	}
}

