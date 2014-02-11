#include <Arduino.h>

/*
 * New robotic arm control library, using the LEONARDO ONLY (needs more than 2K RAM)
 *
 * *****************************************************************************
 *
 * Device map:
 *
 * 	DOF		MOTOR		DRIVER		NEEDS		PINS		SENSOR		MODE
 * 	1		DC			SHIELD		EN, SE		3, 5		POT, MGN	A0, SPI
 * 	2		BIG DC		TALON		SERVO		7			MGN			A1, SPI
 * 	3		BIG DC		TALON		SERVO		8			MGN			A2, SPI
 * 	4		DC			SHIELD		EN, SE		4, 6		MGN			A3, SPI
 * 	5		SERVO		-			SERVO		9			-			-
 * 	6		DYN			-			DYN			0, 1, 2		-			-
 *
 * SPI takes pins 11, 12, 13 (in software mode because it's a LEONARDO, use 10 as CS if encoders are chained
 * DYN takes pins 0, 1, 2, NEEDS A LEONARDO!!! (because of Hardware Serial1)
 *
 * *****************************************************************************
 *
 * Pin map:
 *
 * 	PIN		DESC
 * 	0		Dynamixel RX-TX
 * 	1		Dynamixel RX-TX
 * 	2		Dynamixel direction select
 * 	3		DC Motor 1 Direction
 * 	4		DC Motor 2 Direction
 * 	5		DC Motor 1 Enable
 * 	6		DC Motor 2 Enable
 * 	7		Talon 1
 * 	8		Talon 2
 * 	9		Servo
 * 	10		AS5043 CS if chained encoders OR extra Servo
 * 	11		AS5043 PROG
 * 	12		AS5043 DO
 * 	13		AS5043 CLK
 *
 * 	A0		DC Motor ANALOG INPUT if potentiometer, if MGN then CS0 or AnalogOut
 * 	A1		AS5043 CS1 or AnalogOut
 * 	A2		AS5043 CS2 or AnalogOut
 * 	A3		AS5043 CS3 or AnalogOut
 *
 * *****************************************************************************
 *
 * The following libraries are needed:
 *
 *	Servo
 *	ros
 *	DynamixelSerial
 *	SimpleDynamixel
 *	SimpleDriver
 *	Talon
 *	SimpleServo
 *	AS5043
 *	Encoder
 *	SimplePID
 *
 * The ros library is a modified version for the Leonardo
 * The DynamixelSerial class needs a HardwareSerial object passed in the constructor
 * The SimpleDynamixel is a wrapper class for DynamixelSerial and needs one instance of it passed
 * The SimpleDriver is a wrapper class for motor control
 * The Talon is a simple class for Talon control (uses the Servo class)
 * The SimpleServo is a simple class for servo control (to ensure consistency with other libs)
 * The AS5043 class needs the pin numbers when used in software mode (software mode needed in LEONARDO)
 * The Encoder class is a wrapper for the AS5043 class and for an analogInput "equivalent" functionality
 * The SimplePID is a simple controller class
 *
 * *****************************************************************************
 *
 * Now, all sensors (and all sensor should be magnetic encoders, though the base sensor
 * could be a simple multiturn or endless pot) have a resolution of 10 bits, from 0
 * to 1023, now, only the first 4 DOF need any form of control, since the last 2 DOF,
 * being servos, can simply be written to with the desired angle value.
 *
 * However, there are differences between the use of small DC motors and the BIG DC motors. The
 * DC motors use a shield, and can be set with a PWM from 2 * (-127 to + 127). The Talon
 * motors, being "servo-like", could be called with an angle, but the encapsulating Talon
 * class deals with them as "motor-like" devices, going from -127 to +127. This is done
 * to ensure "orthogonality", and to create general control functions. Since -128 * 2 = -256 is
 * outside the range of PWM output, we must restrict values to [-127, +127].
 *
 * The max "angle" absolute value of the Servo output classes SimpleDynamixel and SimpleServo
 * can be specified in the constructor. This way, we can set "angles" for each servo as
 * however we like, -90 to +90, -127 to +127, etcetera, mapping to "standard" 0 to 180
 *
 * All motors, then, are set with values from -angle to +angle, be it speed (SimpleDriver, Talon)
 * or position (SimpleDynamixel, SimpleServo). Another class, SaberSerial, also follows this
 * convention (set in hardware too) with -127 to +127. The operation range is covered with Int16
 *
 * This way all sensors are 10 bits, and all actuators are 8/9/10 bits. Even though this loses
 * resolution in the PWM output of the simple motor driver and servo output of the Talon wrapper, we
 * consider that 255 values are enough for most non high precision scenarios, and that
 * the PWM signal does not loses appreciable effect. But we gain uniformity, consistency,
 * and less hassle when dealing with all the DOF for control tasks.
 *
 * *****************************************************************************
 *
 * All encoders use (or should use) the same AS5043Class instance. The "proper" way to read them
 * would be to use the read() method of the AS5043Class instance, and then get the information of each
 * encoder with the get() method of each EncoderClass instance. There is a read() method in the
 * EncoderClass, but using it is expensive (ALL sensor would be reread at each call, this takes
 * much more time and scrambles timeStamps and angle changes).
 *
 * *****************************************************************************
 *
 * Finally, since a resolution reduction is necessary in some of the DOFs, the same -127 to +127
 * convention can also be applied to encoder reads, if the max angle of the encoder is set to +127 at
 * construction, but could be set to something else too. This is particularly important if using a multiturn
 * pot to read position since resolution is constrained anyway. Or for example, when using a restricted
 * DOF, a resolution reduction is convenient. Also reduces noise, somewhat. Range is covered with Int16
 *
 * *****************************************************************************
 *
 * Keep in ming that the "best" initial request for each DOF should be something on the line of:
 *
 * 	DOF1 -> 0		(goes from -180 to +180)
 * 	DOF2 -> -90		(goes from -90 to +90)
 * 	DOF3 -> -90		(goes from -90 to +90)
 * 	DOF4 -> 0		(goes from -90 to +90)
 * 	DOF5 -> 0		(goes from -90 to +90)
 * 	DOF6 -> 0		(goes from -90 to +90)
 */

#include "Servo.h"
#include "ros.h"
#include "arm_interface/Arm.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"
#include "SimplePID.h"
#include "SimpleOpen.h"

ros::NodeHandle nh;

// Pins 0, 1 (Serial1 in the LEONARDO) and 2 (DirectionPin) for communication with Dynamixel AX-12 servo (sixth DOF)
// DynamixelClass(Serial, direction_pin)
// SimpleDynamixelClass(DynamixelClass, min_angle_0_1000, max_angle_0_1000, map_abs_angle) // min maps to -angle and max to +angle, min > max is possible
DynamixelClass Dynamixel(&Serial1, 2);
SimpleDynamixelClass DOF6(&Dynamixel, 100, 600, 90);

// Pins 3, 5 for DC Motor control (first DOF) and pins 4, 6 for DC Motor Control (fourth DOF)
// SimpleDriverClass(pin_en, pin_dir, umbral, val) // sense == sign(val), max_angle == abs(val)
SimpleDriverClass DOF1(5, 3, 2, 127);
SimpleDriverClass DOF4(6, 4, 2, 127);
// Pins 7 for Talon control (second DOF) and 8 for Talon control (third DOF)
// TalonClass(pin, umbral, val) // sense == sign(val), max_angle == abs(val)
TalonClass DOF2(2, 127);
TalonClass DOF3(2, 127);

// Pin 9 for standard servo control (fifth DOF)
// SimpleServoClass(pin, val) // sense == sign(val), max_angle == abs(val) (from -90 to +90)
SimpleServoClass DOF5(85);

// Pins 13, 12, 11 for SPI single reads of magnetic encoders (second, third and fourth DOFs) in software emulation mode
// The CS pin must be specified at each constructor of the EncoderClass, pin 10 is best left ALONE in the UNO!
// AS5043Class(pin_clk, pin_do, pin_prog)
AS5043Class AS5043obj(13, 12, 11);

// Pin A0 for pot read (first DOF), specifies min and max values in EFFECTIVE TURN, positive sense, sampling size of 1
// EncoderClass(pin_An, min, max, val) // min > max is possible (not recommended), sense == sign(val), max_angle == abs(val)
EncoderClass ENC1(A0, 100, 600, 30, 180);
// Pins A1...A3 as CS for SPI read of each sensor, positive sense, sampling size of 1
// EncoderClass(AS5043Class, pin_csn, min, max, val), as before
EncoderClass ENC2(&AS5043obj, A1, 100, 600, 90);
EncoderClass ENC3(&AS5043obj, A2, 100, 600, 90);

// The set of PID controllers for each controllable DOF, set kp, ki, kd, km IN SECONDS!!!, last value is umbral for zero error/ zero output
SimplePID pid1(.1, .1, .1, 0, 0); // NEEDS an umbral of zero to avoid vanishing of the ITerm contribution (if set), because motor cannot hold itself
SimplePID pid2(.1, .1, .1, 0, 1);
SimplePID pid3(.1, .1, .1, 0, 1);

// float KScale, int StartValue, int OutputFixed (+/-)
SimpleOpen open4(0.9, 0, 127);

// The incoming 6 DOF int16 information from ROS
int d1_desired = 0;
int d2_desired = -90;
int d3_desired = -90;
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

// All the pubs msgs, keep names short
arm_interface::Arm armOut;
ros::Publisher armOut_f("arm_lectures", &armOut);

unsigned long milisLast = 0;

void setup() {
	DOF2.attach(7);
	DOF3.attach(8);
	DOF5.attach(9);

	nh.initNode();

	nh.subscribe(sub_arm);
	nh.advertise(armOut_f);

	// Comm at 1 Mhz
	Dynamixel.begin(1000000UL, 2);
	AS5043obj.begin();

	// Initialize desired values to ACTUAL values (to avoid jerk)
	d1_desired = ENC1.begin();
	d2_desired = ENC2.begin();
	d3_desired = ENC3.begin();
	d4_desired = open4.read();
	d5_desired = 0;
	d6_desired = 0;
}

void loop() {

	nh.spinOnce();
	unsigned long milisNow = millis();

	// Maybe in here
	// DOF4.write(open4.check());

	// 20 Hz operation
	if (milisNow - milisLast >= 50) {

		milisLast = milisNow;

		int dof1lec = ENC1.readAngle();
		int dof2lec = ENC2.readAngle();
		int dof3lec = ENC3.readAngle();
		int dof4lec = open4.read();

		pid1.compute(dof1lec, d1_desired);
		pid2.compute(dof2lec, d2_desired);
		pid3.compute(dof3lec, d3_desired);
		open4.setGoal(d4_desired);

		DOF1.write(pid1.get());
		DOF2.write(pid2.get());
		DOF3.write(pid3.get());
		DOF4.write(open4.check());
		DOF5.write(d5_desired);
		DOF6.write(d6_desired);

		armOut.dof1 = dof1lec;
		armOut.dof2 = dof2lec;
		armOut.dof3 = dof3lec;
		armOut.dof4 = dof4lec;
		armOut.dof5 = DOF5.read();
		armOut.dof6 = DOF6.read();

		armOut_f.publish(&armOut);
	}
}

