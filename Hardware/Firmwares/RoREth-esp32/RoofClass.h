//
// RTI-Zone Dome Rotator firmware.
//
//  Copyright © 2024 Rodolphe Pineau. All rights reserved.
//
//

#include <atomic>

#include <extEEPROM.h>
#include <Wire.h>

#define I2C_WIRE    Wire

#define EEPROM_ADDR 0x50
#define I2C_CHUNK_SIZE  16


#include <AccelStepper.h>
#include "StopWatch.h"

// set this to match the type of steps configured on the
// stepper controller
#define STEP_TYPE 8


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
// #define MIN_PULSE_WIDTH 3

// ISD02/04/08 stepper controller min pulse width = 5uS at 1600rev/s (8 microsteps).
// TB6600 Stepper controller min pulse width = 5uS
#define MIN_PULSE_WIDTH 5

// used to offset the config location.. at some point.
#define EEPROM_LOCATION     0  // not used with Arduino Due flash
#define EEPROM_SIGNATURE    0001

#ifdef USE_ETHERNET
typedef struct IPCONFIG {
	bool            bUseDHCP;
	IPAddress       ip;
	IPAddress       dns;
	IPAddress       gateway;
	IPAddress       subnet;
} IPConfig;
#endif // USE_ETHERNET


typedef struct RoofConfiguration {
	int             signature;
	long            stepsPerStroke;
	long			openPos;
	long            acceleration;
	long            maxSpeed;
	bool            reversed;
	int             cutOffVolts;
#ifdef USE_ETHERNET
	IPConfig        ipConfig;
#endif // USE_ETHERNET

} Configuration;


enum Seeks { NOT_MOVING,           // Not homing or calibrating
			MOVING_GOTO,
			MOVING_OPENING,
			MOVING_CLOSING,
			FINISHING_OPEN,
			FINISHING_CLOSE,
			CALIBRATION_STEP1,
			CALIBRATION_MEASURE
};

enum ShutterStates { OPEN, CLOSED, OPENING, CLOSING, ERROR, FINISHING_OPENING, FINISHING_CLOSING };

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);


class RoofClass
{

public:

	RoofClass();

	void		SaveToEEProm();

	// Condition sensor methods
	bool		GetConditionStatus();
	void		openInterrupt();
	void		closedInterrupt();
	void		conditionInterrupt();

	// motor methods
	long        GetAcceleration();
	void        SetAcceleration(const long);

	long        GetMaxSpeed();
	void        SetMaxSpeed(const long);

	long        GetPosition();

	bool        GetReversed();
	void        SetReversed(const bool reversed);
	int         GetDirection();

	long        GetStepsPerStroke();
	void        SetStepsPerStroke(const long);

	long		getOpenPosition();

	void        restoreDefaultMotorSettings();

	double      GetAngularDistance(const double fromAngle, const double toAngle);

	// Voltage methods
	int         GetLowVoltageCutoff();
	void        SetLowVoltageCutoff(const int);
	bool        GetVoltsAreLow();
	String      GetVoltString();

	int         GetSeekMode();

	// Homing and Calibration
	void        StartCalibrating();
	void        Calibrate();

	// Movers
	void        EnableMotor(const bool);
	void        MoveRelative(const long steps);
	void        Run();
	void        Stop();
	void        motorStop();
	void        motorMoveRelative(const long howFar);

	void		ButtonCheck();

#ifdef USE_ETHERNET
	void        getIpConfig(IPConfig &config);
	bool        getDHCPFlag();
	void        setDHCPFlag(bool bUseDHCP);
	String      getIPAddress();
	void        setIPAddress(String ipAddress);
	String      getIPSubnet();
	void        setIPSubnet(String ipSubnet);
	String      getIPGateway();
	void        setIPGateway(String ipGateway);
#endif // USE_ETHERNET

#ifdef USE_WIFI
	void		getWiFiConfig(WIFIConfig &config);
#endif // USE_WIFI
	std::atomic<int>    nStepperInterruptFreq;

	static String IpAddress2String(const IPAddress& ipAddress);
private:
	Configuration   m_Config;

	// Rotator
	bool            m_bWasRunning;
	bool            m_bisClose;
	bool            m_bisOpen;
	std::atomic<enum Seeks>	m_seekMode;
	bool            m_bDoStepsPerStroke;

	StopWatch       m_MoveOffUntilTimer;
	unsigned long   m_nMOVE_OFFUntilLapse = 2000;
	int             m_nMoveDirection;

	std::atomic<long>	m_nStepsAtHome;
	std::atomic<long>	m_nHomePosEdgePass1;
	volatile 	long	m_nHomePosEdgePass2;

	// Power values
	double           m_fAdcConvert;
	int             m_nVolts;
	int             ReadVolts();


	StopWatch       m_periodicReadingTimer;
	unsigned long   m_nNextPeriodicReadingLapse = 10;


	// Utility
	bool        LoadFromEEProm();
	void        SetDefaultConfig();

	std::atomic<bool>	m_bIsBadCondition;

	bool        m_bDoEEPromSave;
	// eeprom
	byte        m_EEPROMpageSize;

	byte        readEEPROMByte(int deviceaddress, unsigned int eeaddress);
	void        readEEPROMBuffer(int deviceaddress, unsigned int eeaddress, byte *buffer, int length);
	void        readEEPROMBlock(int deviceaddress, unsigned int address, byte *data, int offset, int length);
	void        writeEEPROM(int deviceaddress, unsigned int address, byte *data, int length);
	void        writeEEPROMBlock(int deviceaddress, unsigned int address, byte *data, int offset, int length);
};



RoofClass::RoofClass()
{
	DBPrintln("Using external AT24AA128 eeprom");
	Wire.setClock(100000);
	Wire.begin();
	// AT24AA128 page size is 64 byte
	m_EEPROMpageSize = 64;

	m_seekMode = NOT_MOVING;
	m_bWasRunning = false;
	m_bDoStepsPerStroke = false;
	m_nMoveDirection = MOVE_NONE;
	// input

	pinMode(OPEN_PIN,               INPUT_PULLUP);
	pinMode(CLOSE_PIN,               INPUT_PULLUP);
	pinMode(BUTTON_CLOSE,             INPUT_PULLUP);
	pinMode(BUTTON_OPEN,              INPUT_PULLUP);
	pinMode(COND_SENSOR_PIN,        INPUT_PULLUP);
	pinMode(VOLTAGE_MONITOR_PIN,    INPUT_PULLUP);

	pinMode(SPARE1,    INPUT_PULLUP);
	pinMode(SPARE2,    INPUT_PULLUP);

	// output
	pinMode(STEP_PIN,               OUTPUT);
	pinMode(DIRECTION_PIN,          OUTPUT);
	pinMode(STEPPER_ENABLE_PIN,     OUTPUT);
	pinMode(SPARE_OUT1,     		OUTPUT);
	pinMode(SPARE_OUT2,     		OUTPUT);

	LoadFromEEProm();

	m_bDoEEPromSave = false;  // we just read the config, no need to resave all the value we're setting
	SetMaxSpeed(m_Config.maxSpeed);
	SetAcceleration(m_Config.acceleration);
	SetStepsPerStroke(m_Config.stepsPerStroke);
	SetReversed(m_Config.reversed);
	// set pulse width
	stepper.setMinPulseWidth(MIN_PULSE_WIDTH); // 5uS to test. Default in the source seems to be set to 1 ...

	m_bDoEEPromSave = true;

	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsBadCondition = true;
	}
	else {
		m_bIsBadCondition = false;
	}

	if(digitalRead(CLOSE_PIN) == LOW) {
		// we're at the close position
		m_bisClose = true;
		DBPrintln("At close on startup");
	}
	else if(digitalRead(OPEN_PIN) == LOW) {
		// we're at the close position
		m_bisOpen = true;
		DBPrintln("At close on startup");
	}


	m_fAdcConvert = RES_MULT * (AD_REF / 1023.0) * 100;


	nStepperInterruptFreq = 0; // used to pass interrupt frequency to core1 from call to methods from core0

	// reset all timers
	m_MoveOffUntilTimer.reset();
	m_periodicReadingTimer.reset();
}

void RoofClass::openInterrupt()
{
	long  nPos;

	// debounce
	if (digitalRead(OPEN_PIN) != LOW)
		return;

	nPos = stepper.currentPosition(); // read position immediately

	switch(m_seekMode) {
		case CLOSING: // stop and take note of where we are so we can reverse.
			motorStop();
			// at close position = 0;
			m_bisOpen = false;
			m_bisClose = true;
			break;

		case OPENING: // stop and take note of where we are so we can reverse.
			// at open position = nPos;
			motorStop();
			m_bisOpen = true;
			m_bisClose = false;
			break;
#pragma message "FixMe"
/*
		case CALIBRATION_STEP1: // take note of the first edge
			m_nHomePosEdgePass1 = nPos;
			m_seekMode = CALIBRATION_MOVE_OFF2; // let's not be fooled by the double trigger
			m_MoveOffUntilTimer.reset();
			break;

		case CALIBRATION_MEASURE: // stop and take note of where we are so we can reverse.
			m_nStepsAtHome = nPos;
			m_nHomePosEdgePass2 = m_nStepsAtHome;
			motorStop();
			break;
*/
		default: // we need to set some sane default
			break;
	}

}

void RoofClass::closedInterrupt()
{
	long  nPos;

	// debounce
	if (digitalRead(CLOSE_PIN) != LOW)
		return;

	nPos = stepper.currentPosition(); // read position immediately

	switch(m_seekMode) {
		case CLOSING: // stop and take note of where we are so we can reverse.
			motorStop();
			// at close position = 0;
			m_bisOpen = false;
			m_bisClose = true;
			break;

		case OPENING: // stop and take note of where we are so we can reverse.
			// at open position = nPos;
			motorStop();
			m_bisOpen = true;
			m_bisClose = false;
			break;
#pragma message "FixMe"
/*
		case CALIBRATION_STEP1: // take note of the first edge
			m_nHomePosEdgePass1 = nPos;
			m_seekMode = CALIBRATION_MOVE_OFF2; // let's not be fooled by the double trigger
			m_MoveOffUntilTimer.reset();
			break;

		case CALIBRATION_MEASURE: // stop and take note of where we are so we can reverse.
			m_nStepsAtHome = nPos;
			m_nHomePosEdgePass2 = m_nStepsAtHome;
			motorStop();
			break;
*/
		default: // we need to set some sane default
			break;
	}

}


inline void RoofClass::conditionInterrupt()
{
	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsBadCondition = true;
	}
	else
		m_bIsBadCondition = false;
}

void RoofClass::SaveToEEProm()
{
	if(!m_bDoEEPromSave)
		return;

	DBPrintln("RoofClass::SaveToEEProm");

	m_Config.signature = EEPROM_SIGNATURE;

	writeEEPROM(EEPROM_ADDR, EEPROM_LOCATION, (byte *) &m_Config, sizeof(Configuration));
}

bool RoofClass::LoadFromEEProm()
{
	bool response = true;

	DBPrintln("RoofClass::LoadFromEEProm");
	//  zero the structure so currently unused parts
	//  dont end up loaded with random garbage
	memset(&m_Config, 0, sizeof(Configuration));
	readEEPROMBuffer(EEPROM_ADDR, EEPROM_LOCATION, (byte *) &m_Config, sizeof(Configuration) );

	if (m_Config.signature != EEPROM_SIGNATURE) {
		DBPrintln("Setting default value for new signature");
		SetDefaultConfig();
		SaveToEEProm();
		response = false;
	}
	DBPrintln("expected signature : " + String(EEPROM_SIGNATURE));
	DBPrintln("m_Config.signature : " + String(m_Config.signature));
	DBPrintln("maxSpeed          : " + String(m_Config.maxSpeed));
	DBPrintln("acceleration      : " + String(m_Config.acceleration));
	DBPrintln("stepsPerStroke    : " + String(m_Config.stepsPerStroke));
	DBPrintln("openPos           : " + String(m_Config.openPos));
	DBPrintln("reversed          : " + String(m_Config.reversed));
	DBPrintln("cutOffVolts       : " + String(m_Config.cutOffVolts));
#ifdef USE_ETHERNET
	DBPrintln("ipConfig.bUseDHCP : " + String(m_Config.ipConfig.bUseDHCP?"Yes":"No"));
	DBPrintln("ipConfig.ip       : " + IpAddress2String(m_Config.ipConfig.ip));
	DBPrintln("ipConfig.dns      : " + IpAddress2String(m_Config.ipConfig.dns));
	DBPrintln("ipConfig.gateway  : " + IpAddress2String(m_Config.ipConfig.gateway));
	DBPrintln("ipConfig.subnet   : " + IpAddress2String(m_Config.ipConfig.subnet));
#endif
	return response;
}

void RoofClass::SetDefaultConfig()
{
	memset(&m_Config, 0, sizeof(Configuration));

	m_Config.signature = EEPROM_SIGNATURE;
	m_Config.maxSpeed = MAX_SPEED;
	m_Config.acceleration = ACCELERATION;
	m_Config.stepsPerStroke = STEPS_DEFAULT;
	m_Config.openPos = 160000000L;
	m_Config.reversed = 0;
	m_Config.cutOffVolts = 1150;
#ifdef USE_ETHERNET
	m_Config.ipConfig.bUseDHCP = true;
	m_Config.ipConfig.ip.fromString("192.168.0.99");
	m_Config.ipConfig.dns.fromString("192.168.0.1");
	m_Config.ipConfig.gateway.fromString("192.168.0.1");
	m_Config.ipConfig.subnet.fromString("255.255.255.0");
#endif // USE_ETHERNET
}

#ifdef USE_ETHERNET
void RoofClass::getIpConfig(IPConfig &config)
{
	config.bUseDHCP = m_Config.ipConfig.bUseDHCP;
	config.ip = m_Config.ipConfig.ip;
	config.dns = m_Config.ipConfig.dns;
	config.gateway = m_Config.ipConfig.gateway;
	config.subnet = m_Config.ipConfig.subnet;
}


bool RoofClass::getDHCPFlag()
{
	return m_Config.ipConfig.bUseDHCP;
}

void RoofClass::setDHCPFlag(bool bUseDHCP)
{
	m_Config.ipConfig.bUseDHCP = bUseDHCP;
	DBPrintln("New bUseDHCP : " + bUseDHCP?"Yes":"No");
	SaveToEEProm();
}

String RoofClass::getIPAddress()
{
	return IpAddress2String(m_Config.ipConfig.ip);
}

void RoofClass::setIPAddress(String ipAddress)
{
	m_Config.ipConfig.ip.fromString(ipAddress);
	DBPrintln("New IP address : " + IpAddress2String(m_Config.ipConfig.ip));
	SaveToEEProm();
}

String RoofClass::getIPSubnet()
{
	return IpAddress2String(m_Config.ipConfig.subnet);
}

void RoofClass::setIPSubnet(String ipSubnet)
{
	m_Config.ipConfig.subnet.fromString(ipSubnet);
	DBPrintln("New subnet mask : " + IpAddress2String(m_Config.ipConfig.subnet));
	SaveToEEProm();
}

String RoofClass::getIPGateway()
{
	return IpAddress2String(m_Config.ipConfig.gateway);
}

void RoofClass::setIPGateway(String ipGateway)
{
	m_Config.ipConfig.gateway.fromString(ipGateway);
	DBPrintln("New gateway : " + IpAddress2String(m_Config.ipConfig.gateway));

	// setting DNS IP to gateway IP as we don't use it and this is probably correct for most home users
	m_Config.ipConfig.dns.fromString(ipGateway);
	SaveToEEProm();
}

#endif // USE_ETHERNET

String RoofClass::IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +
  		String(ipAddress[1]) + String(".") +
		String(ipAddress[2]) + String(".") +
		String(ipAddress[3]);
}

//
// Condition sensor methods
//
bool RoofClass::GetConditionStatus()
{
	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsBadCondition = true;
	}
	else
		m_bIsBadCondition = false;

	return m_bIsBadCondition;
}

//
// motor methods
//
long RoofClass::GetAcceleration()
{
	return m_Config.acceleration;
}

void RoofClass::SetAcceleration(const long newAccel)
{
	m_Config.acceleration = newAccel;
	stepper.setAcceleration(double(newAccel));
	SaveToEEProm();
}

long RoofClass::GetMaxSpeed()
{
	return m_Config.maxSpeed;
}

void RoofClass::SetMaxSpeed(const long newSpeed)
{
	m_Config.maxSpeed = newSpeed;
	stepper.setMaxSpeed(double(newSpeed));
	SaveToEEProm();
}

long RoofClass::getOpenPosition()
{
	return m_Config.openPos;
}

long RoofClass::GetPosition()
{
	long position;
	position = stepper.currentPosition();
#pragma message "FixMe"
/*	if (m_seekMode < CALIBRATION_MOVE_OFF) {
		while (position >= m_Config.stepsPerStroke)
			position -= m_Config.stepsPerStroke;

		while (position < 0)
			position += m_Config.stepsPerStroke;
	}
*/
	return position;
}


bool RoofClass::GetReversed()
{
	return m_Config.reversed;
}

void RoofClass::SetReversed(const bool isReversed)
{
	m_Config.reversed = isReversed;
	stepper.setPinsInverted(isReversed, isReversed, isReversed);
	SaveToEEProm();
}

int RoofClass::GetDirection()
{
	return m_nMoveDirection;
}

long RoofClass::GetStepsPerStroke()
{
	return m_Config.stepsPerStroke;
}

void RoofClass::SetStepsPerStroke(const long newCount)
{
#pragma message "FixMe"
	// m_fStepsPerDegree = (double)newCount / 360.0;
	m_Config.stepsPerStroke = newCount;
	SaveToEEProm();
}

void RoofClass::restoreDefaultMotorSettings()
{
	m_Config.maxSpeed = MAX_SPEED;
	m_Config.acceleration = ACCELERATION;
	m_Config.stepsPerStroke = STEPS_DEFAULT;
	SetMaxSpeed(m_Config.maxSpeed);
	SetAcceleration(m_Config.acceleration);
	SetStepsPerStroke(m_Config.stepsPerStroke);
}


//
// Voltage methods
//
int RoofClass::GetLowVoltageCutoff()
{
	return m_Config.cutOffVolts;
}

void RoofClass::SetLowVoltageCutoff(const int lowVolts)
{
	m_Config.cutOffVolts = lowVolts;
	SaveToEEProm();
}

inline bool RoofClass::GetVoltsAreLow()
{
	bool voltsLow = false;

	if (m_nVolts <= m_Config.cutOffVolts)
		voltsLow = true;
	return voltsLow;
}

inline String RoofClass::GetVoltString()
{
	return String(m_nVolts) + "," + String(m_Config.cutOffVolts);
}

int RoofClass::ReadVolts()
{
	int adc;
	double calc;

	adc = analogRead(VOLTAGE_MONITOR_PIN);
	calc = adc * m_fAdcConvert;
	return int(calc);
}

int RoofClass::GetSeekMode()
{
	return m_seekMode;
}


void RoofClass::StartCalibrating()
{
	stepper.setCurrentPosition(0);
	m_bDoStepsPerStroke = false;
	m_nHomePosEdgePass1 = 0;
	m_nHomePosEdgePass2 = 0;

	if(m_bisClose) {
		m_MoveOffUntilTimer.reset();
#pragma message "FixMe"
		// m_seekMode = CALIBRATION_MOVE_OFF;
		MoveRelative(-5000);
	}
	else {
		m_seekMode = CALIBRATION_STEP1;
		MoveRelative(160000000L);
	}
}

void RoofClass::Calibrate()
{
#pragma message "FixMe"

/*
	if (m_seekMode > HOMING_HOME) {
		switch (m_seekMode) {
			case(CALIBRATION_MOVE_OFF):
				if (!stepper.isRunning()) {
					m_seekMode = CALIBRATION_STEP1;
					stepper.setCurrentPosition(0);
					MoveRelative(160000000L);
				}
				break;

			case(CALIBRATION_MOVE_OFF2):
				if(m_MoveOffUntilTimer.elapsed() >= m_nMOVE_OFFUntilLapse) {
					m_seekMode = CALIBRATION_MEASURE;
				}
				break;

			case(CALIBRATION_MEASURE):
				if (!stepper.isRunning()) { // we have to wait for it to have stopped
					m_seekMode = HOMING_FINISH;
					m_bSetToHomeAzimuth = true;
					m_bDoStepsPerStroke = true; // Once stopped, set SPR to stepper position and save to eeprom.
				}
				break;
			default:
				break;
		}
	}
*/
}

//
// Movers
//
void RoofClass::EnableMotor(const bool bEnabled)
{
	if (!bEnabled) {
		DBPrintln("Motor OFF");
		digitalWrite(STEPPER_ENABLE_PIN, M_DISABLE);
	}
	else {
		DBPrintln("Motor ON");
		digitalWrite(STEPPER_ENABLE_PIN, M_ENABLE);
	}

}

void RoofClass::MoveRelative(const long howFar)
{
	m_nMoveDirection = MOVE_NEGATIVE;
	if (howFar > 0)
		m_nMoveDirection = MOVE_POSITIVE;
	else if(howFar == 0 ) {
		m_nMoveDirection = MOVE_NONE;
		m_seekMode = NOT_MOVING;
		return;
		}

	motorMoveRelative(howFar);
}


void RoofClass::ButtonCheck()
{
	if (digitalRead(BUTTON_OPEN) == LOW) {
		MoveRelative(160000000L);
	}
	else if (digitalRead(BUTTON_CLOSE) == LOW)  {
		MoveRelative(-160000000L);
	}
	else {
		Stop();
	}
}

void RoofClass::Run()
{
	long stepsFromZero;
	long position;
	double azimuthDelta;

	if (m_periodicReadingTimer.elapsed() >= m_nNextPeriodicReadingLapse) {
		m_nVolts = ReadVolts();
		m_periodicReadingTimer.reset();
	}
#pragma message "FixMe"

	//if (m_seekMode > HOMING_HOME)
	// 	Calibrate();

	stepper.run(); // on Core 1

	if (stepper.isRunning()) {
		m_bWasRunning = true;
#pragma message "FixMe"
		// if (m_seekMode == HOMING_HOME && m_HomeFound) { // We're looking for home and found it
		// 	Stop();
		// 	m_bSetToHomeAzimuth = true; // Need to set home az but not until rotator is stopped;
		// 	m_seekMode = HOMING_FINISH;
		// 	return;
		// }
		return;
	}

#pragma message "FixMe"

	// if( m_seekMode == HOMING_BACK_HOME) {
	// 	m_bisClose = true; // we're back home and done homing.
	// 	m_seekMode = NOT_MOVING;
	// }

	if (m_bDoStepsPerStroke) {
		m_bDoStepsPerStroke = false;
		SetStepsPerStroke(m_nHomePosEdgePass2 - m_nHomePosEdgePass1);
		SaveToEEProm();
		position = stepper.currentPosition();
	}

	if (m_bWasRunning) {
		stepsFromZero = GetPosition();
		if (stepsFromZero < 0) {
			while (stepsFromZero < 0)
				stepsFromZero += m_Config.stepsPerStroke;

			stepper.setCurrentPosition(stepsFromZero);
		}

		if (stepsFromZero > m_Config.stepsPerStroke) {
			while (stepsFromZero > m_Config.stepsPerStroke)
				stepsFromZero -= m_Config.stepsPerStroke;

			stepper.setCurrentPosition(stepsFromZero);
		}

		if( m_seekMode == NOT_MOVING) {
			// not moving anymore ..
			m_nMoveDirection = MOVE_NONE;
			EnableMotor(false);
			m_bWasRunning = false;
			// check if we stopped on the home sensor
			if(digitalRead(CLOSE_PIN) == LOW) {
				// we're at the home position
				m_bisClose = true;
			}
			position = stepper.currentPosition();
			while (position >= m_Config.stepsPerStroke)
				position -= m_Config.stepsPerStroke;

			while (position < 0)
				position += m_Config.stepsPerStroke;

			if(position == (m_Config.stepsPerStroke -1))
				position = 0;
			stepper.setCurrentPosition(position);
		}

		if(m_seekMode == MOVING_GOTO) {
			m_nMoveDirection = MOVE_NONE;
			EnableMotor(false);
			m_seekMode = NOT_MOVING;
			position = stepper.currentPosition();
			while (position >= m_Config.stepsPerStroke)
				position -= m_Config.stepsPerStroke;

			while (position < 0)
				position += m_Config.stepsPerStroke;

			if(position == (m_Config.stepsPerStroke -1))
				position = 0;
			stepper.setCurrentPosition(position);
		}
	} // end if (m_bWasRunning)
}

void RoofClass::Stop()
{
	if (!stepper.isRunning())
		return;

	m_seekMode = NOT_MOVING;
	motorStop();
}



void RoofClass::motorStop()
{
	stepper.stop();
}


void RoofClass::motorMoveRelative(const long howFar)
{
	DBPrintln("motorMoveRelative");
	EnableMotor(true);
	stepper.move(howFar);
}

//
// EEProm code to access the AT24AA128 I2C eeprom
//

// read one byte
byte RoofClass::readEEPROMByte(int deviceaddress, unsigned int eeaddress)
{
	byte rdata = 0xFF;
	Wire.beginTransmission(deviceaddress);
	Wire.write(byte(eeaddress >> 8)); // MSB
	Wire.write(byte(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,1);
	if (Wire.available()) {
		rdata = Wire.read();
	}
	return rdata;
}

// Read from EEPROM into a buffer
// slice read into I2C_CHUNK_SIZE block read. I2C_CHUNK_SIZE <=16
void RoofClass::readEEPROMBuffer(int deviceaddress, unsigned int eeaddress, byte *buffer, int length)
{

	int c = length;
	int offD = 0;
	int nc = 0;

	// read until length bytes is read
	while (c > 0) {
		// read maximal I2C_CHUNK_SIZE bytes
		nc = c;
		if (nc > I2C_CHUNK_SIZE)
			nc = I2C_CHUNK_SIZE;
		readEEPROMBlock(deviceaddress, eeaddress, buffer, offD, nc);
		eeaddress+=nc;
		offD+=nc;
		c-=nc;
	}
}

// Read from eeprom into a buffer  (assuming read lenght if I2C_CHUNK_SIZE or less)
void RoofClass::readEEPROMBlock(int deviceaddress, unsigned int eeaddress, byte *data, int offset, int length)
{
	int r = 0;


	Wire.beginTransmission(deviceaddress);
	if (Wire.endTransmission()==0) {
	 	Wire.beginTransmission(deviceaddress);
		Wire.write(byte(eeaddress >> 8));
		Wire.write(byte(eeaddress & 0xFF));
		if (Wire.endTransmission()==0) {
			r = 0;
			Wire.requestFrom(deviceaddress, length);
			while (Wire.available() > 0 && r<length) {
				data[offset+r] = (byte)Wire.read();
				r++;
			}
		}
	}
}



// Write a buffer to EEPROM
// slice write into CHUNK_SIZE block write. I2C_CHUNK_SIZE <=16
void RoofClass::writeEEPROM(int deviceaddress, unsigned int eeaddress, byte *data, int length)
{
	int c = length;					// bytes left to write
	int offD = 0;					// current offset in data pointer
	int offP;						// current offset in page
	int nc = 0;						// next n bytes to write

	// write all bytes in multiple steps
	while (c > 0) {
		// calc offset in page
		offP = eeaddress % m_EEPROMpageSize;
		// maximal 30 bytes to write
		nc = min(min(c, I2C_CHUNK_SIZE), m_EEPROMpageSize - offP);
		writeEEPROMBlock(deviceaddress, eeaddress, data, offD, nc);
		c-=nc;
		offD+=nc;
		eeaddress+=nc;
	}
}

// Write a buffer to EEPROM
void RoofClass::writeEEPROMBlock(int deviceaddress, unsigned int eeaddress, byte *data, int offset, int length)
{

	Wire.beginTransmission(deviceaddress);
	if (Wire.endTransmission()==0) {
	 	Wire.beginTransmission(deviceaddress);
		Wire.write(byte(eeaddress >> 8));
		Wire.write(byte(eeaddress & 0xFF));
		byte *adr = data+offset;
		Wire.write(adr, length);
		Wire.endTransmission();
		delay(20);
	} else {
		DBPrintln("No device at address 0x" + String(deviceaddress, HEX));
	}
}
