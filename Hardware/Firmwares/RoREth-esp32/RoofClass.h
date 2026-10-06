//
// RTI-Zone RoR firmware.
//
//  Copyright © 2024 Rodolphe Pineau. All rights reserved.
//
//

#include <Preferences.h>
#include <FastAccelStepper.h>
#include <nvs_flash.h>

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
	long            stepsPerStroke;
	long			openPos;
	long            acceleration;
	long            maxSpeed;
	bool            reversed;
	IPConfig        ipConfig;
	int				calibrationState;
	int				unsafeAction;
} Configuration;



enum RoofStates { OPEN, CLOSED, NOT_MOVING, OPENING, CLOSING, ROOF_ERROR, FINISHING_OPENING, FINISHING_CLOSING };
enum CalibrationStates { NOT_CALIBRATED, CALIBRATION_STEP_RESET, CALIBRATION_STEP_OPENING, CALIBRATION_STEP_OPEN, CALIBRATION_MEASURE, CALIBRATED };
enum UnsafeActions { DO_NOTHING, CLOSE_ROOF_ON_UNSAFE };
enum AtParkState { MOUNT_PARKED, MOUNT_UNPARKED };

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


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
	void        SetStepsPerStroke(const long, bool bSave = true);

	long		getOpenPosition();

	void        restoreDefaultMotorSettings();

	double      GetAngularDistance(const double fromAngle, const double toAngle);

	int         getRoofState();
	int			getAtParkState();

	// Homing and Calibration
	void        StartCalibrating();
	void        Calibrate();
	int			getRoofCalibrationState();
	void		saveRoofCalibrationState();

	// Movers
	void        MoveRelative(const long steps);
	void        Run();
	void        motorStop();
	void		Open();
	void		Open(long steps);
	void		Close();
	void		Close(long steps);
	void		Stop(bool bHardStop = false);
	void        motorMoveRelative(const long howFar);
	void		GotoPosition(const long nPos);
	bool		isRunning();
	void		ButtonOpenCheck();
	void		ButtonCloseCheck();

	void        getIpConfig(IPConfig &config);
	bool        getDHCPFlag();
	void        setDHCPFlag(bool bUseDHCP);
	String      getIPAddress();
	void        setIPAddress(String ipAddress);
	String      getIPSubnetMask();
	void        setIPSubnetMask(String ipSubnetMask);
	String      getIPGateway();
	void        setIPGateway(String ipGateway);
	void		resetNetworkToDefaults();

	void		SetConditionsAction(int nAction);
	int			GetConditionsAction();

	void		resetAlltoDefault();

#ifdef USE_WIFI
	void		getWiFiConfig(WIFIConfig &config);
#endif // USE_WIFI

	static String IpAddress2String(const IPAddress& ipAddress);

	// fake function for Alpaca and other app that expect a dome
	void GoToAzimuth(double dAz);
	double GetAzimuth();
	void SetParkAzimuth(double dAz);
	void SyncPosition(double dNewPos);
private:
	Configuration   m_Config;
	Preferences 	m_preferences;
	// Roof movement
	bool            m_bWasRunning;
	int				m_nRoofState;

	bool			bOpenButtonPressed = false;
	bool			bCloseButtonPressed = false;
	unsigned long   m_nMOVE_OFFUntilLapse = 2000;

	int             m_nMoveDirection;
	volatile long	m_nStepsAtOpen = STEPS_DEFAULT;
	int				m_nCalibrationState = NOT_CALIBRATED;

	// fake function varialbles.
	double m_dAz = 0;
	double m_dParkAz = 0;

	// Utility
	void 		LoadConfig();
	volatile bool	m_bIsSafe;
};



RoofClass::RoofClass()
{

	m_nRoofState = NOT_MOVING;
	m_bWasRunning = false;
	m_nMoveDirection = MOVE_NONE;
	// input

	pinMode(OPEN_PIN,				INPUT);
	pinMode(CLOSE_PIN,				INPUT);
	pinMode(BUTTON_CLOSE,			INPUT);
	pinMode(BUTTON_OPEN,			INPUT);
	pinMode(COND_SENSOR_PIN,		INPUT);
	pinMode(AT_PARK_PIN,			INPUT);
	pinMode(SPARE1,					INPUT);
	pinMode(SPARE2,					INPUT);

	// output
	pinMode(STEP_PIN,				OUTPUT);
	pinMode(DIRECTION_PIN,			OUTPUT);
	pinMode(STEPPER_ENABLE_PIN,		OUTPUT);
	pinMode(SPARE_OUT1,				OUTPUT);
	pinMode(SPARE_OUT2,				OUTPUT);

	LoadConfig();

	engine.init();
	stepper = engine.stepperConnectToPin(STEP_PIN);
	stepper->setDirectionPin(DIRECTION_PIN,(!m_Config.reversed));
	stepper->setEnablePin(STEPPER_ENABLE_PIN);
	stepper->setAutoEnable(true);
	stepper->setSpeedInHz(m_Config.maxSpeed);  //  steps/s
	stepper->setAcceleration(m_Config.acceleration);    //  steps/s²

	if (digitalRead(COND_SENSOR_PIN) == LOW) {
		m_bIsSafe = false;
	}
	else {
		m_bIsSafe = true;
	}

	if(digitalRead(CLOSE_PIN) == LOW) {
		// we're at the close position
		m_nRoofState = CLOSED;
		stepper->setCurrentPosition(0L);
		DBPrintln("At close on startup");
	}
	else if(digitalRead(OPEN_PIN) == LOW) {
		// we're at the open position
		m_nRoofState = OPEN;
		stepper->setCurrentPosition(m_Config.stepsPerStroke);
		DBPrintln("At Open on startup");
	}
}

void RoofClass::openInterrupt()
{
	long  nPos;

	// debounce
	if (digitalRead(OPEN_PIN) != LOW)
		return;

	switch(m_nRoofState) {
		case OPENING:
			motorStop();
			m_nRoofState = FINISHING_OPENING;
			if(m_nCalibrationState ==  CALIBRATION_STEP_OPENING) {
				m_nCalibrationState = CALIBRATION_STEP_OPEN;
			}
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

	nPos = stepper->getCurrentPosition(); // read position immediately

	switch(m_nRoofState) {
		case CLOSING:
			motorStop();
			m_nRoofState = FINISHING_CLOSING;
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


void RoofClass::LoadConfig()
{
	bool nvsInitDone = false;

	DBPrintln("RoofClass::LoadConfig");
	m_preferences.begin("RTI_RoR", false);
	nvsInitDone = m_preferences.isKey("nvsInit");
	if(!nvsInitDone) {
		DBPrintln("Initializing NVS");
		m_preferences.end();
		nvs_flash_erase();
		nvs_flash_init();
		m_preferences.begin("RTI_RoR", false);
		m_preferences.putBool("nvsInit", true);
	}
	m_Config.stepsPerStroke = m_preferences.getLong("stepsPerStroke",STEPS_DEFAULT);
	m_Config.openPos = m_preferences.getLong("openPos",STEPS_DEFAULT);
	m_Config.acceleration = m_preferences.getLong("acceleration",ACCELERATION);
	m_Config.maxSpeed = m_preferences.getLong("maxSpeed",MAX_SPEED);
	m_Config.reversed = m_preferences.getBool("reverse", false);
	m_Config.ipConfig.bUseDHCP = m_preferences.getBool("bUseDHCP", true);
	m_Config.ipConfig.ip.fromString(m_preferences.getString("ip","192.168.0.99"));
	m_Config.ipConfig.dns.fromString(m_preferences.getString("dns","192.168.0.1"));
	m_Config.ipConfig.gateway.fromString(m_preferences.getString("gateway","192.168.0.1"));
	m_Config.ipConfig.subnetMask.fromString(m_preferences.getString("subnetMask","255.255.255.0"));
	m_Config.calibrationState = m_preferences.getInt("calState",NOT_CALIBRATED);
	m_Config.unsafeAction = m_preferences.getInt("condAction", DO_NOTHING);

	DBPrintln("maxSpeed          : " + String(m_Config.maxSpeed));
	DBPrintln("acceleration      : " + String(m_Config.acceleration));
	DBPrintln("stepsPerStroke    : " + String(m_Config.stepsPerStroke));
	DBPrintln("openPos           : " + String(m_Config.openPos));
	DBPrintln("reversed          : " + String(m_Config.reversed));
	DBPrintln("ipConfig.bUseDHCP : " + String(m_Config.ipConfig.bUseDHCP?"Yes":"No"));
	DBPrintln("ipConfig.ip       : " + IpAddress2String(m_Config.ipConfig.ip));
	DBPrintln("ipConfig.dns      : " + IpAddress2String(m_Config.ipConfig.dns));
	DBPrintln("ipConfig.gateway  : " + IpAddress2String(m_Config.ipConfig.gateway));
	DBPrintln("ipConfig.subnetMask   : " + IpAddress2String(m_Config.ipConfig.subnetMask));
	m_preferences.end();
}

void RoofClass::resetAlltoDefault()
{
	DBPrintln("Resetting do factory defaults");
	DBPrintln("Initializing NVS");
	m_preferences.end();
	nvs_flash_erase();
	nvs_flash_init();
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putBool("nvsInit", true);
	m_preferences.end();
	ESP.restart();
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
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putBool("bUseDHCP", bUseDHCP);
	m_preferences.end();
}

String RoofClass::getIPAddress()
{
	return IpAddress2String(m_Config.ipConfig.ip);
}

void RoofClass::setIPAddress(String ipAddress)
{
	m_Config.ipConfig.ip.fromString(ipAddress);
	DBPrintln("New IP address : " + ipAddress);
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putString("ip", ipAddress);
	m_preferences.end();
}

String RoofClass::getIPSubnetMask()
{
	return IpAddress2String(m_Config.ipConfig.subnetMask);
}

void RoofClass::setIPSubnetMask(String ipSubnetMask)
{
	m_Config.ipConfig.subnetMask.fromString(ipSubnetMask);
	DBPrintln("New subnet mask : " + ipSubnetMask);
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putString("subnetMask", ipSubnetMask);
	m_preferences.end();
}

String RoofClass::getIPGateway()
{
	return IpAddress2String(m_Config.ipConfig.gateway);
}

void RoofClass::setIPGateway(String ipGateway)
{
	m_Config.ipConfig.gateway.fromString(ipGateway);
	DBPrintln("New gateway : " + ipGateway);
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putString("gateway", ipGateway);
	m_preferences.end();
}


String RoofClass::IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +
  		String(ipAddress[1]) + String(".") +
		String(ipAddress[2]) + String(".") +
		String(ipAddress[3]);
}

void RoofClass::resetNetworkToDefaults()
{
	m_preferences.begin("RTI_RoR", false);
	m_preferences.remove("bUseDHCP");
	m_preferences.remove("ip");
	m_preferences.remove("subnetMask");
	m_preferences.remove("gateway");
	m_preferences.remove("dns");
	m_preferences.end();

	// reload defaults
	m_preferences.begin("RTI_RoR", false);
	m_Config.ipConfig.bUseDHCP = m_preferences.getBool("bUseDHCP", true);
	m_Config.ipConfig.ip.fromString(m_preferences.getString("ip","192.168.1.9"));
	m_Config.ipConfig.dns.fromString(m_preferences.getString("gateway","192.168.1.1"));
	m_Config.ipConfig.gateway.fromString(m_preferences.getString("dns","1.1.1.1"));
	m_Config.ipConfig.subnetMask.fromString(m_preferences.getString("subnetMask","255.255.255.0"));
	m_preferences.end();
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
	stepper->setAcceleration(m_Config.acceleration);    //  steps/s²
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putLong("acceleration", newAccel);
	m_preferences.end();
}

long RoofClass::GetMaxSpeed()
{
	return m_Config.maxSpeed;
}

void RoofClass::SetMaxSpeed(const long newSpeed)
{
	m_Config.maxSpeed = newSpeed;
	stepper->setSpeedInHz(newSpeed);  //  steps/s
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putLong("maxSpeed", newSpeed);
	m_preferences.end();
}

long RoofClass::getOpenPosition()
{
	return m_Config.openPos;
}

long RoofClass::GetPosition()
{
	return stepper->getCurrentPosition();
}


bool RoofClass::GetReversed()
{
	return m_Config.reversed;
}

void RoofClass::SetReversed(const bool isReversed)
{
	m_Config.reversed = isReversed;
	stepper->setDirectionPin(DIRECTION_PIN,(!isReversed));
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putBool("reversed", isReversed);
	m_preferences.end();
}

int RoofClass::GetDirection()
{
	return m_nMoveDirection;
}

long RoofClass::GetStepsPerStroke()
{
	return m_Config.stepsPerStroke;
}

void RoofClass::SetStepsPerStroke(const long newCount, bool bSave)
{
 	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
 	DBPrintln("newCount = " + String(newCount));

	m_Config.stepsPerStroke = newCount;

 	DBPrintln("m_Config.stepsPerStroke = " + String(m_Config.stepsPerStroke));
	if(bSave) {
		m_preferences.begin("RTI_RoR", false);
		m_preferences.putLong("stepsPerStroke", newCount);
		m_preferences.end();
	}
}

void RoofClass::saveRoofCalibrationState()
{
 	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
	m_Config.calibrationState = m_nCalibrationState;
	DBPrintln("m_Config.calibrationState = " + String(m_Config.calibrationState));
	m_Config.calibrationState = m_nCalibrationState;
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putInt("calState", m_Config.calibrationState);
	m_preferences.end();
}
void RoofClass::restoreDefaultMotorSettings()
{
	SetMaxSpeed(MAX_SPEED);
	SetAcceleration(ACCELERATION);
	SetStepsPerStroke(STEPS_DEFAULT);
}



int RoofClass::getRoofState()
{
	return m_nRoofState;
}

int RoofClass::getAtParkState()
{
	int nAtPark;
	nAtPark = digitalRead(AT_PARK_PIN);
	return nAtPark;
}

void RoofClass::StartCalibrating()
{
 	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
	m_nRoofState = NOT_MOVING;

	if (digitalRead(CLOSE_PIN) == 0) {
		m_nRoofState = CLOSED;
	}
	if (digitalRead(OPEN_PIN) == 0) {
		m_nRoofState = OPEN;
	}

	// are we open, closed or somewhere in between ?
	if (m_nRoofState != CLOSED ) { // close to restart calibration from close state
		DBPrintln("Closing for calibration");
		m_nCalibrationState = CALIBRATION_STEP_RESET;
		Close(-STEPS_DEFAULT); // move toward close position
	}
	else {
		stepper->setCurrentPosition(0);
		DBPrintln("Opening for calibration");
		DBPrintln("m_nCalibrationState = CALIBRATION_STEP_OPENING");
		m_nCalibrationState = CALIBRATION_STEP_OPENING;
		Open(STEPS_DEFAULT); // move toward open position
	}
	DBPrintln("m_nRoofState " + String(m_nRoofState));
	DBPrintln("m_nCalibrationState " + String(m_nCalibrationState));
}

void RoofClass::Calibrate()
{
	int position;

	switch (m_nCalibrationState) {
		case(CALIBRATION_STEP_RESET):
			if (m_nRoofState == CLOSED) {
	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
				DBPrintln("CALIBRATION_STEP_OPENING");
				m_nCalibrationState = CALIBRATION_STEP_OPENING;
				stepper->setCurrentPosition(0);
				Open(STEPS_DEFAULT);
			}
			break;

		case(CALIBRATION_MEASURE):
			if (m_nRoofState == NOT_MOVING) { // we have to wait for it to have stopped
	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
				DBPrintln("CALIBRATION_MEASURE");
				m_nCalibrationState = CALIBRATED;
				saveRoofCalibrationState();
				position = stepper->getCurrentPosition();
				SetStepsPerStroke(position);
				DBPrintln("CALIBRATION_MEASURE steps per move = " + String(position));
			}
			break;
		default:
			break;
	}
}

int RoofClass::getRoofCalibrationState()
{
	return m_Config.calibrationState;
}

void RoofClass::SetConditionsAction(int nAction)
{
	m_Config.unsafeAction = nAction;
	m_preferences.begin("RTI_RoR", false);
	m_preferences.putInt("condAction", m_Config.unsafeAction);
	m_preferences.end();

}

int RoofClass::GetConditionsAction()
{
	return m_Config.unsafeAction;
}


//
// Movers
//

void RoofClass::MoveRelative(const long howFar)
{
	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
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
	// Goto new target.
	// Cancel any in-flight move first and wait for the stepper to settle,
	// otherwise getCurrentPosition() returns a mid-move value and the new
	// relative move is appended behind the old one, overshooting the target.
	double position;
	double delta;

	stepper->forceStop();
	while(stepper->isRunning()) {
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}

	position = stepper->getCurrentPosition();
	delta = nPos - position;
	MoveRelative(lround(delta));
}

void RoofClass::Open()
{
	if(!m_bIsSafe)
		return;
	Open(m_Config.stepsPerStroke);
}

void RoofClass::Open(long nSteps)
{
	if(!m_bIsSafe)
		return;

	if (digitalRead(OPEN_PIN) == 0) {
		m_nRoofState = OPEN;
		return;
	}

	m_nRoofState = OPENING;
	DBPrintln("m_nRoofState = OPENING");
	GotoPosition(nSteps);
}

void RoofClass::Close()
{
	// if we are at position "0" but not closed :
	if(stepper->getCurrentPosition() == 0)
		Close(-STEPS_DEFAULT); // Force close
	else
		Close(0L);
}

void RoofClass::Close(long nSteps)
{
	if (digitalRead(CLOSE_PIN) == 0) {
		m_nRoofState = CLOSED;
		return;
	}
	m_nRoofState = CLOSING;
	DBPrintln("m_nRoofState = CLOSING");
	GotoPosition(nSteps); // close
}

void RoofClass::ButtonOpenCheck()
{
	bOpenButtonPressed = true;
}

void RoofClass::ButtonCloseCheck()
{
	bCloseButtonPressed = true;
}

bool RoofClass::isRunning()
{
	return m_bWasRunning;
}


void RoofClass::Run()
{
	long position = stepper->getCurrentPosition();

	if (m_nCalibrationState >= NOT_CALIBRATED && m_nCalibrationState!=CALIBRATED)
		Calibrate();

	if(bOpenButtonPressed) {
		bOpenButtonPressed = false;
		if(m_nRoofState == OPENING || m_nCalibrationState >= CALIBRATION_STEP_RESET) {
			Stop(); // we want a soft stop to not break hardware as a roof can have quite a bit of momentum due to its weight
		}
		else {
			Open();
		}
	}
	if(bCloseButtonPressed) {
		bCloseButtonPressed = false;
		if(m_nRoofState == CLOSING || m_nCalibrationState >= CALIBRATION_STEP_RESET) {
			Stop(); // we want a soft stop to not break hardware as a roof can have quite a bit of momentum due to its weight
		}
		else {
			Close();
		}

	}

	if (stepper->isRunning()) {
		m_bWasRunning = true;
		return;
	}


	if (m_bWasRunning) {
		DBPrintln("[ ********** " + String(__func__) + " ********** ]");
		DBPrintln("m_nRoofState " + String(m_nRoofState));
		DBPrintln("m_nCalibrationState " + String(m_nCalibrationState));

		if(m_nCalibrationState == CALIBRATION_STEP_OPEN) {
			DBPrintln("[ ********** " + String(__func__) + " ********** ]");
			DBPrintln("CALIBRATION_STEP_OPEN");
			if(digitalRead(OPEN_PIN) != LOW) {
				if(position == m_Config.stepsPerStroke) {
					m_Config.stepsPerStroke +=1000;
					stepper->setCurrentPosition(position);
				}
				m_nCalibrationState = CALIBRATION_STEP_OPENING;
				Open();
			}
			else { // opened and not moving anymore, store open position
				m_nCalibrationState = CALIBRATION_MEASURE;
			}
			DBPrintln("m_nCalibrationState " + String(m_nCalibrationState));
		}
		if( m_nRoofState == NOT_MOVING) {
			DBPrintln("[ ********** " + String(__func__) + " ********** ]");
			DBPrintln("NOT_MOVING");
			// not moving anymore ..
			m_nMoveDirection = MOVE_NONE;
			m_bWasRunning = false;
			// check if we stopped on the close sensor
			if(digitalRead(CLOSE_PIN) == LOW) {
				// we're at the close position
				m_nRoofState = CLOSED;
				stepper->setCurrentPosition(0L);
			}
			if(digitalRead(OPEN_PIN) == LOW) {
				// we're at the open position
				m_nRoofState = OPEN;
			}
			position = stepper->getCurrentPosition();
		}

		if(m_nRoofState == FINISHING_CLOSING) {
			DBPrintln("[ ********** " + String(__func__) + " ********** ]");
			DBPrintln("FINISHING_CLOSING");
			if(digitalRead(CLOSE_PIN) != LOW) {
				// not quite close. move a bit more.
				if(position == 0) {
					position = 1000;
					stepper->setCurrentPosition(position);
				}
				Close();
			}
			else {
				stepper->setCurrentPosition(0L); // closed is position 0
				m_nRoofState = NOT_MOVING;
			}
		}

		if(m_nRoofState == FINISHING_OPENING) {
			DBPrintln("[ ********** " + String(__func__) + " ********** ]");
			DBPrintln("FINISHING_OPENING");
			if(digitalRead(OPEN_PIN) != LOW) {
				if(position == m_Config.stepsPerStroke) {
					m_Config.stepsPerStroke +=1000;
					stepper->setCurrentPosition(position);
				}
				Open();
			}
			else {
				m_nRoofState = NOT_MOVING;
			}
		}

		if(m_nRoofState == CLOSING) { // if for some reason the closed limit switch was touched.. and we're not closed..
			DBPrintln("[ ********** " + String(__func__) + " ********** ]");
			DBPrintln("CLOSING if stopped in lalaland");
			if(digitalRead(CLOSE_PIN) != LOW) {
				MoveRelative(-STEPS_DEFAULT); // move toward close position
			}
		}

		if(m_nRoofState == NOT_MOVING) {
			m_nMoveDirection = MOVE_NONE;
		}
	DBPrintln("End of Run() m_nRoofState " + String(m_nRoofState));
	DBPrintln("End of Run() m_nCalibrationState " + String(m_nCalibrationState));

	} // end if (m_bWasRunning)
}

void RoofClass::Stop(bool bHardStop)
{
	m_nRoofState = NOT_MOVING;
	if(bHardStop)
		stepper->forceStop();
	else
		stepper->stopMove();
}

void RoofClass::motorStop()
{
	stepper->stopMove();
}

void RoofClass::motorMoveRelative(const long howFar)
{
	DBPrintln("[ ********** " + String(__func__) + " ********** ]");
	DBPrintln("motorMoveRelative " + String(howFar));
	stepper->move(howFar);
	// Wait for the motor to start
	while(!stepper->isRunning()) {
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}
