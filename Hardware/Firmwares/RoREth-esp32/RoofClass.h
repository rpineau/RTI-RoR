//
// RTI-Zone RoR firmware.
//
//  Copyright © 2024 Rodolphe Pineau. All rights reserved.
//
//

#include <atomic>
#include <AccelStepper.h>
#include <Preferences.h>

#include "StopWatch.h"
#include "config.h"


typedef struct IPCONFIG {
	bool            bUseDHCP;
	IPAddress       ip;
	IPAddress       dns;
	IPAddress       gateway;
	IPAddress       subnetMask;
} IPConfig;


typedef struct RoofConfiguration {
	int             signature;
	long            stepsPerStroke;
	long			openPos;
	long            acceleration;
	long            maxSpeed;
	bool            reversed;
	int             cutOffVolts;
	IPConfig        ipConfig;
} Configuration;



enum RoofStates { OPEN, CLOSED, NOT_MOVING, OPENING, CLOSING, ROOF_ERROR, FINISHING_OPENING, FINISHING_CLOSING, CALIBRATION_STEP_RESET, CALIBRATION_STEP_OPENING, CALIBRATION_STEP_OPEN, CALIBRATION_MEASURE};

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);


class RoofClass
{

public:

	RoofClass();

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

	int         getRoofState();

	// Homing and Calibration
	void        StartCalibrating();
	void        Calibrate();

	// Movers
	void        EnableMotor(const bool);
	void        MoveRelative(const long steps);
	void        Run();
	void        Stop();
	void        motorStop();
	void		Open();
	void		Close();
	void        motorMoveRelative(const long howFar);
	void		GotoPosition(const long nPos);
	bool		isRunning();
	void		ButtonCheck();

	void        getIpConfig(IPConfig &config);
	bool        getDHCPFlag();
	void        setDHCPFlag(bool bUseDHCP);
	String      getIPAddress();
	void        setIPAddress(String ipAddress);
	String      getIPSubnetMask();
	void        setIPSubnetMask(String ipSubnetMask);
	String      getIPGateway();
	void        setIPGateway(String ipGateway);

#ifdef USE_WIFI
	void		getWiFiConfig(WIFIConfig &config);
#endif // USE_WIFI
	std::atomic<int>    nStepperInterruptFreq;

	static String IpAddress2String(const IPAddress& ipAddress);

	// fake function for Alpaca and other app that expect a dome
	void GoToAzimuth(double dAz);
	double GetAzimuth();
	void SetParkAzimuth(double dAz);
	void SyncPosition(double dNewPos);
private:
	Configuration   m_Config;
	Preferences 	m_preferences;
	// Rotator
	bool            m_bWasRunning;
	int				m_nRoofState;
	bool            m_bDoStepsPerStroke;

	unsigned long   m_nMOVE_OFFUntilLapse = 2000;
	int             m_nMoveDirection;

	std::atomic<long>	m_nStepsAtOpen;
	std::atomic<long>	m_nHomePosEdgePass1;
	volatile 	long	m_nHomePosEdgePass2;

	// fake function varialbles.
	double m_dAz = 0;
	double m_dParkAz = 0;

	// Power values
	double           m_fAdcConvert;
	int             m_nVolts;
	int             MeasureVoltage();


	StopWatch       m_periodicReadingTimer;
	unsigned long   m_nNextPeriodicReadingLapse = 10;


	// Utility
	bool 		LoadConfig();
	void        SetDefaultConfig();

	std::atomic<bool>	m_bIsSafe;
	bool	m_bDoSave;
};



RoofClass::RoofClass()
{

	m_preferences.begin("RTI_RoR", false);

	m_nRoofState = NOT_MOVING;
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

	LoadConfig();
	m_bDoSave = false;  // we just read the config, no need to resave all the value we're setting
	SetMaxSpeed(m_Config.maxSpeed);
	SetAcceleration(m_Config.acceleration);
	SetStepsPerStroke(m_Config.stepsPerStroke);
	SetReversed(m_Config.reversed);
	m_bDoSave = true;

	// set pulse width
	stepper.setMinPulseWidth(MIN_PULSE_WIDTH); // 5uS to test. Default in the source seems to be set to 1 ...

	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsSafe = false;
	}
	else {
		m_bIsSafe = true;
	}

	if(digitalRead(CLOSE_PIN) == LOW) {
		// we're at the close position
		m_nRoofState = CLOSED;
		DBPrintln("At close on startup");
	}
	else if(digitalRead(OPEN_PIN) == LOW) {
		// we're at the open position
		m_nRoofState = OPEN;
		DBPrintln("At close on startup");
	}


	m_fAdcConvert = RES_MULT * (AD_REF / 1023.0) * 100;


	nStepperInterruptFreq = 0; // used to pass interrupt frequency to core1 from call to methods from core0

	// reset all timers
	m_periodicReadingTimer.reset();
}

void RoofClass::openInterrupt()
{
	long  nPos;

	// debounce
	if (digitalRead(OPEN_PIN) != LOW)
		return;

	nPos = stepper.currentPosition(); // read position immediately

	switch(m_nRoofState) {

		case OPENING: // stop and take note of where we are so we can reverse.
			// at open position = nPos;
			motorStop();
			m_nRoofState = FINISHING_OPENING;
			break;

		case CALIBRATION_STEP_OPENING: // take note of the first edge
			motorStop();
			m_nRoofState = CALIBRATION_STEP_OPEN; // let's not be fooled by the double trigger
			break;

		case CALIBRATION_MEASURE: // stop and take note of where we are so we can reverse.
			m_nStepsAtOpen = nPos;
			motorStop();
			break;
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

	switch(m_nRoofState) {
		case CLOSING: // stop and take note of where we are so we can reverse.
			motorStop();
			m_nRoofState = FINISHING_CLOSING;

			// at close position = 0;
			break;

		case CALIBRATION_STEP_RESET: // take note of the first edge
			motorStop();
			// at close position = 0;
			break;

		default: // we need to set some sane default
			break;
	}

}


inline void RoofClass::conditionInterrupt()
{
	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsSafe = false;
	}
	else
		m_bIsSafe = true;
}


bool RoofClass::LoadConfig()
{
	bool response = true;

	DBPrintln("RoofClass::LoadConfig");
	m_Config.signature = m_preferences.getInt("signature",0);
	DBPrintln("expected signature : " + String(CONF_SIGNATURE));
	DBPrintln("m_Config.signature : " + String(m_Config.signature));
	if (m_Config.signature != CONF_SIGNATURE) {
		DBPrintln("Setting default value for new signature");
		SetDefaultConfig();
		response = false;
	}
	else {
		m_Config.stepsPerStroke = m_preferences.getLong("stepsPerStroke",0);
		m_Config.openPos = m_preferences.getLong("openPos",0);
		m_Config.acceleration = m_preferences.getLong("acceleration",0);
		m_Config.maxSpeed = m_preferences.getLong("maxSpeed",0);
		m_Config.reversed = m_preferences.getBool("reverse", false);
		m_Config.cutOffVolts = m_preferences.getInt("cutOffVolts",1200);

		m_Config.ipConfig.bUseDHCP = m_preferences.getBool("bUseDHCP", true);
		m_Config.ipConfig.ip.fromString(m_preferences.getString("ip","192.168.0.99"));
		m_Config.ipConfig.dns.fromString(m_preferences.getString("dns","192.168.0.1"));
		m_Config.ipConfig.gateway.fromString(m_preferences.getString("gateway","192.168.0.1"));
		m_Config.ipConfig.subnetMask.fromString(m_preferences.getString("subnetMask","255.255.255.0"));
	}

	DBPrintln("maxSpeed          : " + String(m_Config.maxSpeed));
	DBPrintln("acceleration      : " + String(m_Config.acceleration));
	DBPrintln("stepsPerStroke    : " + String(m_Config.stepsPerStroke));
	DBPrintln("openPos           : " + String(m_Config.openPos));
	DBPrintln("reversed          : " + String(m_Config.reversed));
	DBPrintln("cutOffVolts       : " + String(m_Config.cutOffVolts));
	DBPrintln("ipConfig.bUseDHCP : " + String(m_Config.ipConfig.bUseDHCP?"Yes":"No"));
	DBPrintln("ipConfig.ip       : " + IpAddress2String(m_Config.ipConfig.ip));
	DBPrintln("ipConfig.dns      : " + IpAddress2String(m_Config.ipConfig.dns));
	DBPrintln("ipConfig.gateway  : " + IpAddress2String(m_Config.ipConfig.gateway));
	DBPrintln("ipConfig.subnetMask   : " + IpAddress2String(m_Config.ipConfig.subnetMask));
	return response;
}

void RoofClass::SetDefaultConfig()
{
	memset(&m_Config, 0, sizeof(Configuration));

	m_Config.signature = CONF_SIGNATURE;
	m_Config.stepsPerStroke = STEPS_DEFAULT;
	m_Config.openPos = 160000000L;
	m_Config.acceleration = ACCELERATION;
	m_Config.maxSpeed = MAX_SPEED;
	m_Config.reversed = 0;
	m_Config.cutOffVolts = 1150;

	m_Config.ipConfig.bUseDHCP = true;
	m_Config.ipConfig.ip.fromString("192.168.0.99");
	m_Config.ipConfig.dns.fromString("192.168.0.1");
	m_Config.ipConfig.gateway.fromString("192.168.0.1");
	m_Config.ipConfig.subnetMask.fromString("255.255.255.0");

	// save all pref to lvs
	m_preferences.putInt("signature",m_Config.signature);
	m_preferences.putLong("stepsPerStroke",m_Config.stepsPerStroke);
	m_preferences.putLong("openPos",m_Config.openPos);
	m_preferences.putLong("acceleration",m_Config.acceleration);
	m_preferences.putLong("maxSpeed",m_Config.maxSpeed);
	m_preferences.putBool("reversed",m_Config.reversed);
	m_preferences.putInt("cutOffVolts",m_Config.cutOffVolts);

	m_preferences.putBool("bUseDHCP",m_Config.ipConfig.bUseDHCP);
	m_preferences.putString("ip","192.168.0.99");
	m_preferences.putString("dns","192.168.0.1");
	m_preferences.putString("gateway","192.168.0.1");
	m_preferences.putString("subnetMask","255.255.255.0");
}

void RoofClass::getIpConfig(IPConfig &config)
{
	config.bUseDHCP = m_Config.ipConfig.bUseDHCP;
	config.ip = m_Config.ipConfig.ip;
	config.dns = m_Config.ipConfig.dns;
	config.gateway = m_Config.ipConfig.gateway;
	config.subnetMask = m_Config.ipConfig.subnetMask;
}


bool RoofClass::getDHCPFlag()
{
	return m_Config.ipConfig.bUseDHCP;
}

void RoofClass::setDHCPFlag(bool bUseDHCP)
{
	m_Config.ipConfig.bUseDHCP = bUseDHCP;
	DBPrintln("New bUseDHCP : " + bUseDHCP?"Yes":"No");
	m_preferences.putBool("bUseDHCP", bUseDHCP);
}

String RoofClass::getIPAddress()
{
	return IpAddress2String(m_Config.ipConfig.ip);
}

void RoofClass::setIPAddress(String ipAddress)
{
	m_Config.ipConfig.ip.fromString(ipAddress);
	DBPrintln("New IP address : " + ipAddress);
	m_preferences.putString("ip", ipAddress);
}

String RoofClass::getIPSubnetMask()
{
	return IpAddress2String(m_Config.ipConfig.subnetMask);
}

void RoofClass::setIPSubnetMask(String ipSubnetMask)
{
	m_Config.ipConfig.subnetMask.fromString(ipSubnetMask);
	DBPrintln("New subnet mask : " + ipSubnetMask);
	m_preferences.putString("subnetMask", ipSubnetMask);
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

	m_preferences.putString("gateway", ipGateway);
	m_preferences.putString("dns", ipGateway);
}


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
		m_bIsSafe = false;
	}
	else
		m_bIsSafe = true;

	return m_bIsSafe;
}

//
// Fake method for Alpaca and app that expect a dome
//
void RoofClass::GoToAzimuth(double dAz)
{
	m_dAz = dAz;
}

double RoofClass::GetAzimuth()
{
	return m_dAz;
}

void RoofClass::SetParkAzimuth(double dAz)
{
	m_dParkAz = dAz;
}

void RoofClass::SyncPosition(double dNewPos)
{
		m_dAz = dNewPos;

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
	if(m_bDoSave)
		m_preferences.putLong("acceleration", newAccel);
}

long RoofClass::GetMaxSpeed()
{
	return m_Config.maxSpeed;
}

void RoofClass::SetMaxSpeed(const long newSpeed)
{
	m_Config.maxSpeed = newSpeed;
	stepper.setMaxSpeed(double(newSpeed));
	if(m_bDoSave)
		m_preferences.putLong("maxSpeed", newSpeed);
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
/*	if (m_nRoofState < CALIBRATION_MOVE_OFF) {
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
	if(m_bDoSave)
		m_preferences.putBool("reversed", isReversed);
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
	if(m_bDoSave)
		m_preferences.putBool("stepsPerStroke", newCount);
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
	m_preferences.putInt("cutOffVolts", lowVolts);
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

int RoofClass::MeasureVoltage()
{
	int adc;
	double calc;

	adc = analogRead(VOLTAGE_MONITOR_PIN);
	calc = adc * m_fAdcConvert;
	return int(calc);
}

int RoofClass::getRoofState()
{
	return m_nRoofState;
}


void RoofClass::StartCalibrating()
{
	m_nRoofState = NOT_MOVING;

	if (digitalRead(CLOSE_PIN) == 0) {
		m_nRoofState = CLOSED;
	}
	if (digitalRead(OPEN_PIN) == 0) {
		m_nRoofState = OPEN;
	}

	// are we open, closed or somewhere in between ?
	if (m_nRoofState != CLOSED ) { // close to restart calibration from close state
		MoveRelative(-160000000L); // move toward close position
		m_nRoofState = CALIBRATION_STEP_RESET;
	}
	else {
		stepper.setCurrentPosition(0);
		m_nRoofState = CALIBRATION_STEP_OPENING;
		MoveRelative(160000000L); // move toward open position
	}
	m_bDoStepsPerStroke = false;
}

void RoofClass::Calibrate()
{
#pragma message "FixMe"

	switch (m_nRoofState) {
		case(CALIBRATION_STEP_RESET):
			if (!stepper.isRunning()) {
				m_nRoofState = CALIBRATION_STEP_OPENING;
				stepper.setCurrentPosition(0);
				MoveRelative(160000000L);
			}
			break;

		case(CALIBRATION_STEP_OPEN):
				m_nRoofState = CALIBRATION_MEASURE;
			break;

		case(CALIBRATION_MEASURE):
			if (!stepper.isRunning()) { // we have to wait for it to have stopped
				SetStepsPerStroke(stepper.currentPosition());
			}
			break;
		default:
			break;

	}
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
		m_nRoofState = NOT_MOVING;
		return;
		}

	motorMoveRelative(howFar);
}

void RoofClass::GotoPosition(const long nPos)
{
		// Goto new target
	double position;
	double delta;

	position = stepper.currentPosition();
	delta = nPos - position;
	MoveRelative(delta);
}

void RoofClass::Open()
{
	m_nVolts = MeasureVoltage();
	if(GetVoltsAreLow()) // do not try to open if we're already at low voltage
		return;

	if (digitalRead(OPEN_PIN) == 0) {
		m_nRoofState = OPEN;
		return;
	}

	m_nRoofState = OPENING;
	DBPrintln("shutterState = OPENING");
	GotoPosition(m_Config.stepsPerStroke);
}

void RoofClass::Close()
{
	if (digitalRead(CLOSE_PIN) == 0) {
		m_nRoofState = CLOSED;
		return;
	}
	m_nRoofState = CLOSING;
	DBPrintln("shutterState = CLOSING");
	GotoPosition(0L); // close
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

bool RoofClass::isRunning()
{
	return m_bWasRunning;
}


void RoofClass::Run()
{
	long stepsFromZero;
	long position;
	double azimuthDelta;

	if (m_periodicReadingTimer.elapsed() >= m_nNextPeriodicReadingLapse) {
		m_nVolts = MeasureVoltage();
		m_periodicReadingTimer.reset();
	}
#pragma message "FixMe"

	if (m_nRoofState >= CALIBRATION_STEP_RESET)
		Calibrate();

	stepper.run(); // on Core 1

	if (stepper.isRunning()) {
		m_bWasRunning = true;
		if (m_nRoofState == CALIBRATION_STEP_OPENING && m_nRoofState == OPEN) {
			Stop();
			m_nRoofState = CALIBRATION_STEP_OPEN;
			return;
		}
		return;
	}

#pragma message "FixMe"

	if( m_nRoofState == CALIBRATION_STEP_RESET) {
		m_nRoofState = CALIBRATION_STEP_OPENING;
	}

	if (m_bDoStepsPerStroke) {
		m_bDoStepsPerStroke = false;
		// we count from close, close is 0, full open is the current position
		position = stepper.currentPosition();
		SetStepsPerStroke(position);
		m_preferences.putLong("stepsPerStroke", position);
	}

	if (m_bWasRunning) {
		if( m_nRoofState == NOT_MOVING) {
			// not moving anymore ..
			m_nMoveDirection = MOVE_NONE;
			EnableMotor(false);
			m_bWasRunning = false;
			// check if we stopped on the close sensor
			if(digitalRead(CLOSE_PIN) == LOW) {
				// we're at the close position
				m_nRoofState = CLOSED;
			}
			if(digitalRead(OPEN_PIN) == LOW) {
				// we're at the open position
				m_nRoofState = OPEN;
			}
			position = stepper.currentPosition();
		}

		if(m_nRoofState == FINISHING_CLOSING) {
			if(digitalRead(CLOSE_PIN) != LOW) {
				// not quite close. move a bit more.
				if(position == 0)
					position = 1000;
				Close();
			}
			else {
				m_nRoofState = NOT_MOVING;
			}
		}

		if(m_nRoofState == FINISHING_OPENING) {
			if(digitalRead(OPEN_PIN) != LOW) {
				if(position == m_Config.stepsPerStroke)
					m_Config.stepsPerStroke +=1000;
				m_bDoStepsPerStroke = true; // adjust open position value
				Open();
			}
			else {
				m_nRoofState = NOT_MOVING;
			}
		}

		if(m_nRoofState == NOT_MOVING) {
			m_nMoveDirection = MOVE_NONE;
			EnableMotor(false);
			position = stepper.currentPosition();
		}

	} // end if (m_bWasRunning)
}

void RoofClass::Stop()
{
	if (!stepper.isRunning())
		return;

	m_nRoofState = NOT_MOVING;
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

