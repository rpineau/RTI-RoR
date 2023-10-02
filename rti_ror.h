//
//  RTI-RoR.h
//  RTI-RoR
//
//  Created by Rodolphe Pineau on 2020/10/4.
//  RTI-RoR X2 plugin

#ifndef __RTI_RoR__
#define __RTI_RoR__

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
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>

// SB includes
#include "../../licensedinterfaces/driverrootinterface.h"
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"

#include "StopWatch.h"

#define MAKE_ERR_CODE(P_ID, DTYPE, ERR_CODE)  (((P_ID<<24) & 0xff000000) | ((DTYPE<<16) & 0x00ff0000)  | (ERR_CODE & 0x0000ffff))

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 500
#define MAX_READ_WAIT_TIMEOUT 25
#define NB_RX_WAIT 10
#define ND_LOG_BUFFER_SIZE 256
#define PANID_TIMEOUT 15    // in seconds
#define RAIN_CHECK_INTERVAL 10

#define PLUGIN_VERSION      1.34
#define PLUGIN_ID   1

#define PLUGIN_DEBUG 2


// Roof commands
const char ABORT_MOVE_CMD               = 'a'; // Tell everything to STOP!
const char ETH_RECONFIG                 = 'b'; // reconfigure ethernet
const char CALIBRATE_ROTATOR_CMD        = 'c'; // Calibrate the dome
const char RESTORE_MOTOR_DEFAULT        = 'd'; // restore default values for motor control.
const char ETH_MAC_ADDRESS              = 'f'; // get the MAC address.
const char IP_ADDRESS                   = 'j'; // get/set the IP address
const char VOLTS_ROTATOR_CMD            = 'k'; // Get volts and get/set cutoff
const char IP_SUBNET                    = 'p'; // get/set the ip subnet
const char SPEED_ROTATOR_CMD            = 'r'; // Get/Set step rate (speed)
const char STEPSPER_ROTATOR_CMD         = 't'; // Get/set Steps per rotation
const char IP_GATEWAY                   = 'u'; // get/set default gateway IP
const char IP_DHCP                      = 'w'; // get/set DHCP mode
const char RAIN_ROOF_GET             = 'F'; // Get rain status
// Roof commands
const char CLOSE_ROOF_CMD            = 'C'; // Close shutter
const char SHUTTER_RESTORE_MOTOR_DEFAULT= 'D'; // Restore default values for motor control.
const char ACCELERATION_ROOF_CMD     = 'E'; // Get/Set stepper acceleration
const char OPEN_ROOF_ORDER           = 'G'; // set the sequencing order for roof/south wall opening
const char STATE_ROOF_GET            = 'M'; // Get shutter state
const char OPEN_ROOF_CMD             = 'O'; // Open the shutter
const char POSITION_ROOF_GET            = 'P'; // Get step position
const char SHUTTER_PANID_GET            = 'Q'; // get and set the XBEE PAN ID
const char SPEED_ROOF_CMD            = 'R'; // Get/Set step rate (speed)
const char STEPSPER_ROOF_CMD         = 'T'; // Get/Set steps per stroke
const char VERSION_ROOF_GET          = 'V'; // Get version string
const char REVERSED_ROOF_CMD         = 'Y'; // Get/Set stepper reversed status
const char SOUTH_WALL_PRESENT        =  'Z'; // enable/disable south wall operations


// Error code
enum RTIRoRErrors {PLUGIN_OK=0, NOT_CONNECTED, CANT_CONNECT, BAD_CMD_RESPONSE, COMMAND_FAILED, COMMAND_TIMEOUT, ERR_RAINING, ERR_BATTERY_LOW};
enum RTIRoRRoofState { OPEN=0 , CLOSED, OPENING, CLOSING, BOTTOM_OPEN, BOTTOM_CLOSED, BOTTOM_OPENING, BOTTOM_CLOSING, ROOF_ERROR, FINISHING_OPEN, FINISHING_CLOSE };

enum HomeStatuses {NOT_AT_HOME = 0, HOMED, ATHOME};
enum RainActions {DO_NOTHING=0, HOME, PARK};
enum MoveDirection {MOVE_NEGATIVE = -1, MOVE_NONE, MOVE_POSITIVE};
// RG-11
enum RainSensorStates {RAINING= 0, NOT_RAINING, RAIN_UNKNOWN};

class CRTIRoR
{
public:
    CRTIRoR();
    ~CRTIRoR();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    const bool  IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }

    // RoR commands
    int openRoof();
    int closeRoof();
    int getFirmwareVersion(std::string &sVersion, float &fVersion);
    int getFirmwareVersion(float &fVersion);

    // command complete functions
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);

    int abortCurrentCommand();

    int getBatteryLevel();

    int  getSouthWallPeesent(bool &bPresent);
    int  setSouthWallPeesent(bool bPresent);

    int  getRoofOpenOrder(bool &bRoofFirst);
    int  setRoofOpenOrder(bool bRoofFirst);

    int getBatteryLevels(double &dRoofVolts, double &dRoofCutOff);
    int setBatteryCutOff(double dRoofCutOff);

    int getDefaultDir(bool &bNormal);
    int setDefaultDir(bool bNormal);

    int getRainSensorStatus(int &nStatus);

    int getRoofSpeed(int &nSpeed);
    int setRoofSpeed(int nSpeed);

    int getRoofAcceleration(int &nAcceleration);
    int setRoofAcceleration(int nAcceleration);

    void    getRoofStatus(int &nStatus);

    int restoreRoofMotorSettings();
    
    void enableRainStatusFile(bool bEnable);
    void getRainStatusFileName(std::string &fName);
    void writeRainStatus();

    // network config
    int getMACAddress(std::string &MACAddress);
    int reconfigureNetwork();

    int getUseDHCP(bool &bUseDHCP);
    int setUseDHCP(bool bUseDHCP);

    int getIpAddress(std::string &IpAddress);
    int setIpAddress(std::string IpAddress);

    int getSubnetMask(std::string &subnetMask);
    int setSubnetMask(std::string subnetMask);

    int getIPGateway(std::string &IpAddress);
    int setIPGateway(std::string IpAddress);

    
protected:

    int             roofCommand(const std::string sCmd, std::string &sResp, char respCmdCode, int nTimeout = MAX_TIMEOUT);
    int             readResponse(std::string &sResp, int nTimeout = MAX_TIMEOUT);

    bool            isRoRMoving();
    int             getRoofState(int &nState);
    int             parseFields(std::string sResp, std::vector<std::string> &svFields, char cSeparator);

    SerXInterface   *m_pSerx;

    std::string     m_Port;
    bool            m_bNetworkConnected;
    bool            m_bSouthWallPresent;
    
    bool            m_bOpenRoofFirst;
    bool            m_bIsConnected;
    bool            m_bRoofOpened;
    double          m_dRoofBatteryVolts;
    std::string     m_sFirmwareVersion;
    float           m_fVersion;
    int             m_nRoofState;
    int             m_nIsRaining;
    std::string     m_sRainStatusfilePath;
    std::ofstream   m_RainStatusfile;
    bool            m_bSaveRainStatus;
    int             m_nRainStatus;
    CStopWatch      m_cRainCheckTimer;
    
    std::string     m_IpAddress;
    std::string     m_SubnetMask;
    std::string     m_GatewayIP;
    bool            m_bUseDHCP;
    
#ifdef PLUGIN_DEBUG
    // timestamp for logs
    const std::string getTimeStamp();
    std::ofstream m_sLogFile;
    std::string m_sLogfilePath;
#endif

};

#endif
