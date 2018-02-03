/* 2018 RobotMap.h */

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

#ifndef SRC_ROBOTMAP_H_
#define SRC_ROBOTMAP_H_

// Turn on or off various features
// -------------------------------

// Gyros
#define NAVX
//#define ADXRS_GYRO

// Controllers
//#define JOYSTICK
#define XBOX

// Check to make sure JOYSTICK and XBOX aren't both enabled
#if defined(JOYSTICK) and defined(XBOX)
#pragma message "WARNING! Both joystick and XBox controllers are enabled!"
#endif

// Select arm and in/out speed control mode (the default) or position mode using
// position sensors.
#define ARM_UP_DOWN_USING_POSITION
//#define IN_OUT_USING_POSITION

// CAN bus addresses
// -----------------
#define CAN_PDP				0
#define CAN_TALON_LEFT_FRONT		1
#define CAN_TALON_RIGHT_FRONT		2
#define CAN_TALON_LEFT_REAR		3
#define CAN_TALON_RIGHT_REAR		4
#define CAN_TALON_IN_OUT_MOTOR		5
#define CAN_TALON_WRIST_MOTOR		6
#define CAN_TALON_ARM_MOTOR		7
#define CAN_TALON_CLIMB_MOTOR		8
#define CAN_TALON_CLIMB_MOTOR2		9
#define CAN_PCM				11

// PCM Channels
#define PCM_CHAN_CLIMB_UP		0
#define PCM_CHAN_CLIMB_DOWN		1
#define PCM_CHAN_ARM_UP			3
#define PCM_CHAN_ARM_DOWN		4

// Digital I/O Channels
#define DIO_LIMIT_SW			0
#define DIO_LIMIT_ARM			1


#endif /* SRC_ROBOTMAP_H_ */
