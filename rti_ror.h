//
//  RTIRoR.h
//  RTIRoR
//
//  Created by Rodolphe Pineau on 4/27/2019.
//  RTIRoR X2 plugin

#ifndef __RTIRoR__
#define __RTIRoR__

// standard C includes
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <math.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

// SB includes
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000
#define RTI_LOG_BUFFER_SIZE 256
#define INTER_COMMARTI_PAUSE_MS	100

// #define RTI_DEBUG 2

// Error code
enum RTIRoRErrors {RTI_OK=0, NOT_CONNECTED, RTI_CANT_CONNECT, RTI_BAD_CMD_RESPONSE, COMMARTI_FAILED};
enum RTIRoRShutterState {OPEN = 0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR };

// RG-11
enum RainSensorStates {RAINING= 0, NOT_RAINING};

class CRTIRoR
{
public:
    CRTIRoR();
    ~CRTIRoR();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    const bool  IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeprPinter(SleeperInterface *p) {m_pSleeper = p; }

    // Dome commands
    int gotoAzimuth(double dNewAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *szVersion, int nStrMaxLen);
    int getFirmwareVersion(float &fVersion);
    int goHome();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);
    int abortCurrentCommand();
    void syncDome(double dAz, double dEl);
    
    // getter/setter
    int getNbTicksPerRev();
    int setNbTicksPerRev(int nSteps);

    int getBatteryLevel();

    double getHomeAz();
    int setHomeAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();
    int getBatteryLevels(double &dShutterVolts, double &dShutterCutOff);
    int setBatteryCutOff(double dShutterCutOff);
    
    int getDefaultDir(bool &bNormal);
    int setDefaultDir(bool bNormal);

    int getRainSensorStatus(int &nStatus);

    int getShutterSpeed(int &nSpeed);
    int setShutterSpeed(int nSpeed);

    int getShutterAcceleration(int &nAcceleration);
    int setShutterAcceleration(int nAcceleration);

	
	int	getSutterWatchdogTimerValue(int &nValue);
	int	setSutterWatchdogTimerValue(const int &nValue);

    int    getRainTimerValue(int &nValue);
    int    setRainTimerValue(const int &nValue);

    void setDebugLog(bool bEnable);

protected:
    
    int             readResponse(char *respBuffer, int nBufferLen);
    int             getShutterState(int &nState);
    int             getStepPerRev(int &nStepPerRev);

    int             roofCommand(const char *cmd, char *result, char respCmdCode, int resultMaxLen);
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);

    SerXInterface   *m_pSerx;
    SleeperInterface *m_pSleeper;
    
    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bShutterOpened;
    
    int             m_nNbStepPerRev;
    double          m_dShutterBatteryVolts;
    double          m_dHomeAz;
    
    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    float           m_fVersion;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    char            m_szLogBuffer[RTI_LOG_BUFFER_SIZE];
    int             m_nIsRaining;

#ifdef RTI_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;	  // LogFile
#endif

};

#endif
