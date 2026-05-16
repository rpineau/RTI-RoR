//
// Firmware configuration
// Copyright © 2025 Rodolphe Pineau. All rights reserved.
//
#ifndef __R_CONFIG__
#define __R_CONFIG__

// #define DEBUG   // enable debug to serial port defined as DebugPort
// #define DEBUG_TO_COMPUTER // send all debug to usb serial instead of 3 pin serial debug.

#ifdef DEBUG
#pragma message "Debug messages enabled"
#ifndef DEBUG_TO_COMPUTER
#define DebugPort Serial1    //  Rx2,Tx2 =  Serial1
#endif
#define DBPrint(x) if(DebugPort) DebugPort.print(x)
#define DBPrintln(x) if(DebugPort) DebugPort.println(x)
#define DBPrintHex(x) if(DebugPort) DebugPort.print(x, HEX)
#else
#pragma message "Debug messages disabled"
#define DBPrint(x)
#define DBPrintln(x)
#define DBPrintHex(x)
#endif // DEBUG

#define VERSION "2.645"
#define USE_ALPACA
#define Computer Serial     // USB = Serial
#ifdef DEBUG
#ifdef DEBUG_TO_COMPUTER
#define DebugPort Serial
#endif
#endif

//
// ESP32 dev boards
//
// input
#define OPEN_PIN            33
#define CLOSE_PIN           15
#define BUTTON_CLOSE        14
#define BUTTON_OPEN         27
#define COND_SENSOR_PIN     25
#define SPARE1				34
#define SPARE2				26
// ouput
#define STEPPER_ENABLE_PIN  13  // Digital Output
#define DIRECTION_PIN        2  // Digital Output
#define STEP_PIN            32  // Digital Output
#define SPARE_OUT1			 0
#define SPARE_OUT2			12

// analog
#define VOLTAGE_MONITOR_PIN A0  // GPIO26/ADC0
#define AD_REF      3.3
#define RES_MULT    5.0 // resistor voltage divider on the shield


#define MOVE_NEGATIVE       -1
#define MOVE_NONE            0
#define MOVE_POSITIVE        1

// #define M_ENABLE    HIGH
// #define M_DISABLE   LOW
#define M_ENABLE    LOW
#define M_DISABLE   HIGH

// A4988
//#define M_ENABLE    LOW
//#define M_DISABLE   HIGH

#define MAX_SPEED           8000
#define ACCELERATION        7000

/*
Micro-steps per Stroke with original motor and 15.3:1 gearbox
	NexDome 2m      : 440640
	Explora-Dome 8' : 479800
*/
#define STEPS_DEFAULT       440640

// DM556T stepper controller min pulse width  = 2.5uS
// ISD02/04/08 stepper controller min pulse width = 5uS at 1600rev/s (8 microsteps).
// TB6600 Stepper controller min pulse width = 5uS
#define MIN_PULSE_WIDTH 5

#define ETHERNET_CS     5
#define ETHERNET_INT	0
#define ETHERNET_RESET  4
#define CMD_SERVER_PORT 2323
#define domeEthernet Ethernet

#endif