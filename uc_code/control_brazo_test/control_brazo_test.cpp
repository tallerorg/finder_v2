#include <Arduino.h>
#include "Servo.h"
#include "ros.h"
#include "arm_interface/Arm.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"

ros::NodeHandle nh;

// Pins 0, 1 (Serial1 in the LEONARDO) and 2 (DirectionPin) for communication with Dynamixel AX-12 servo (sixth DOF)
// DynamixelClass(Serial, direction_pin)
// SimpleDynamixelClass(DynamixelClass, min_angle_0_1000, max_angle_0_1000, map_abs_angle) // min maps to -angle and max to +angle, min > max is possible
DynamixelClass Dynamixel(&Serial1, 2);
SimpleDynamixelClass DOF6(&Dynamixel, 400, 600, 90);

// Pins 3, 5 for DC Motor control (first DOF) and pins 4, 6 for DC Motor Control (fourth DOF)
// SimpleDriverClass(pin_en, pin_dir, umbral, val) // sense == sign(val), max_angle == abs(val)
SimpleDriverClass DOF1(5, 3, 2, 127);

// Pins 7 for Talon control (second DOF) and 8 for Talon control (third DOF)
// TalonClass(pin, umbral, val) // sense == sign(val), max_angle == abs(val)
TalonClass DOF2(2, 127);
TalonClass DOF3(2, 127);

// Pin 9 for standard servo control (fifth DOF)
// SimpleServoClass(pin, val) // sense == sign(val), max_angle == abs(val) (from -90 to +90)
SimpleServoClass DOF4(85);
SimpleServoClass DOF5(85);

// The incoming 6 DOF int16 information from ROS
int d1_desired = 0;
int d2_desired = 0;
int d3_desired = 0;
int d4_desired = 0;
int d5_desired = 0;
int d6_desired = 0;

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void arm_h(const arm_interface::Arm& d_msg) {
	d1_desired = d_msg.dof1;
	d2_desired = d_msg.dof2;
	d3_desired = d_msg.dof3;
	d4_desired = d_msg.dof4;
	d5_desired = d_msg.dof5;
	d6_desired = d_msg.dof6;
}
ros::Subscriber<arm_interface::Arm> sub_arm("arm_motors", arm_h);

unsigned long milisLast = 0;

void setup() {
	DOF2.attach(7);
	DOF3.attach(8);
	DOF4.attach(10);
	DOF5.attach(9);
	Dynamixel.begin(1000000UL, 2);
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
		DOF5.write(d5_desired);
		DOF6.write(d6_desired);
	}
}

