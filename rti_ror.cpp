//
//  RTI-Dome.cpp
//  RTI-Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2020/10/4.


#include "rti_ror.h"

CRTIRoR::CRTIRoR()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;
    m_bOpenRoofFirst = false;
    m_dRoofBatteryVolts = 0.0;
    m_bRoofOpened = false;
    m_fVersion = 0.0;
    m_nIsRaining = NOT_RAINING;
    m_bSaveRainStatus = false;
    m_cRainCheckTimer.Reset();
    m_nRainStatus = RAIN_UNKNOWN;
    m_Port.clear();
    m_bNetworkConnected = false;
    m_IpAddress.clear();
    m_SubnetMask.clear();
    m_GatewayIP.clear();
    m_bUseDHCP = false;
    m_nRoofState = CLOSED;
    m_sFirmwareVersion.clear();

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\RTI-Dome-Log.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RTI-Dome-Log.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RTI-Dome-Log.txt";
#endif
    m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
#endif

#if defined(SB_WIN_BUILD)
    m_sRainStatusfilePath = getenv("HOMEDRIVE");
    m_sRainStatusfilePath += getenv("HOMEPATH");
    m_sRainStatusfilePath += "\\RTI_Rain.txt";
#elif defined(SB_LINUX_BUILD)
    m_sRainStatusfilePath = getenv("HOME");
    m_sRainStatusfilePath += "/RTI_Rain.txt";
#elif defined(SB_MAC_BUILD)
    m_sRainStatusfilePath = getenv("HOME");
    m_sRainStatusfilePath += "/RTI_Rain.txt";
#endif
    
#if defined PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CRTIRoR] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CRTIRoR] Constructor Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [CRTIRoR] Rains status file : " << m_sRainStatusfilePath<<std::endl;
    m_sLogFile.flush();
#endif

}

CRTIRoR::~CRTIRoR()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if(m_sLogFile.is_open())
        m_sLogFile.close();
#endif
}

int CRTIRoR::Connect(const char *pszPort)
{
    int nErr;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // 115200 8N1 DTR
    nErr = m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connection failed, nErr = " << nErr <<  std::endl;
        m_sLogFile.flush();
#endif
        m_bIsConnected = false;
        return nErr;
    }

    m_Port.assign(pszPort);
    
    m_bIsConnected = true;
    if(m_Port.size()>=3 && m_Port.find("TCP")!= -1)  {
        m_bNetworkConnected = true;
    }
    else
        m_bNetworkConnected = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] connected to " << pszPort << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] connected via network : " << (m_bNetworkConnected?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getIpAddress(m_IpAddress);
    if(nErr)
        m_IpAddress = "";
    nErr |= getSubnetMask(m_SubnetMask);
    if(nErr)
        m_SubnetMask = "";
    nErr |= getIPGateway(m_GatewayIP);
    if(nErr)
        m_GatewayIP = "";
    nErr |= getUseDHCP(m_bUseDHCP);
    if(nErr)
        m_bUseDHCP = false;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    if(nErr) {
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Board without network feature." << std::endl;
        m_sLogFile.flush();
    }
#endif
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Getting Firmware." << std::endl;
    m_sLogFile.flush();
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_sFirmwareVersion, m_fVersion);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Error Getting Firmware : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect]Got Firmware "<<  m_sFirmwareVersion << "( " << std::fixed << std::setprecision(2) << m_fVersion << ")."<< nErr << std::endl;
    m_sLogFile.flush();
#endif
    if(m_fVersion < 2.0f && m_fVersion != 0.523f && m_fVersion != 0.522f)  {
        return FIRMWARE_NOT_SUPPORTED;
    }

    // we need to get the initial state
    getRoofState(m_nRoofState);

    return SB_OK;
}

void CRTIRoR::Disconnect()
{
    if(m_bIsConnected) {
        abortCurrentCommand();
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] m_bIsConnected : " << (m_bIsConnected?"true":"false") << std::endl;
    m_sLogFile.flush();
#endif
}

int CRTIRoR::roofCommand(const std::string sCmd, std::string &sResp, char respCmdCode, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;
    std::string localResp;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    m_pSerx->purgeTxRx();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [roofCommand] Sending : '" << sCmd <<  "'" << std::endl;
    m_sLogFile.flush();
#endif
    nErr = m_pSerx->writeFile((void *)(sCmd.c_str()), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [roofCommand] writeFile error : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    if (respCmdCode == 0x00)
        return nErr;

    // read response
    nErr = readResponse(localResp, nTimeout);
    if(nErr)
        return nErr;

    if(!localResp.size())
        return BAD_CMD_RESPONSE;

    if(localResp.at(0) != respCmdCode)
        nErr = BAD_CMD_RESPONSE;

    sResp = localResp.substr(1, localResp.size());

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [roofCommand] response : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::readResponse(std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    char pszBuf[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    sResp.clear();
    memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting = " << nBytesWaiting << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        if(!nBytesWaiting) {
            nbTimeouts += MAX_READ_WAIT_TIMEOUT;
            if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] bytesWaitingRx timeout, no data for" << nbTimeouts <<" ms" << std::endl;
                m_sLogFile.flush();
#endif
                nErr = COMMAND_TIMEOUT;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile error." << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile Timeout Error." << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile nBytesWaiting = " << nBytesWaiting << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile ulBytesRead =" << ulBytesRead << std::endl;
            m_sLogFile.flush();
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    } while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != '#');


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] pszBuf = '" << pszBuf << "'" << std::endl;
    m_sLogFile.flush();
#endif


    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else
        *(pszBufPtr-1) = 0; //remove the #

    sResp.assign(pszBuf);
    return nErr;
}


int CRTIRoR::getRoofState(int &nState)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    std::vector<std::string> shutterStateFileds;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << STATE_ROOF_GET << '#';
    nErr = roofCommand(ssCmd.str(), sResp, STATE_ROOF_GET);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofState] ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        nState = ROOF_ERROR;
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofState] response =  " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    try {
        nState = std::stoi(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofState] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        nState = 0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofState] nState =  " << nState << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::getBatteryLevels(double &dRoofVolts, double &dRoofCutOff)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> voltsFields;
    std::stringstream ssCmd;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    dRoofVolts  = 0;
    dRoofCutOff = 0;

    ssCmd << VOLTS_ROTATOR_CMD << '#';
    nErr = roofCommand(ssCmd.str(), sResp, VOLTS_ROTATOR_CMD);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        dRoofVolts = -1;
        dRoofCutOff = -1;
        return nErr;
    }
    nErr = parseFields(sResp, voltsFields, ',');

    if(!voltsFields.size()) { // no shutter value
        dRoofVolts = -1;
        dRoofCutOff = -1;
        return nErr;
    }

    try {
        dRoofVolts = std::stof(voltsFields[0]);
        dRoofCutOff = std::stof(voltsFields[1]);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        dRoofVolts = 0;
        dRoofCutOff = 0;
    }

    dRoofVolts = dRoofVolts / 100.0;
    dRoofCutOff = dRoofCutOff / 100.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] shutterVolts = " << std::fixed << std::setprecision(2) << dRoofVolts << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getBatteryLevels] dRoofCutOff = " << std::fixed << std::setprecision(2) << dRoofCutOff << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::setBatteryCutOff(double dRoofCutOff)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    int nRoofCutOff;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nRoofCutOff = dRoofCutOff * 100.0;

    ssCmd << VOLTS_ROTATOR_CMD << nRoofCutOff <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, VOLTS_ROTATOR_CMD);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setBatteryCutOff] dRoofCutOff ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    return nErr;
}

int CRTIRoR::getSouthWallPresent(bool &bPresent)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    ssCmd << SOUTH_WALL_PRESENT << '#';
    nErr = roofCommand(ssCmd.str(), sResp, SOUTH_WALL_PRESENT);

    if(nErr) {
        return nErr;
    }
    try {
        bPresent = std::stoi(sResp) ? false:true;
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSouthWallPeesent] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        bPresent = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSouthWallPeesent] bPresent = " << (bPresent?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif
    
    m_bSouthWallPresent = bPresent;

    return nErr;

}

int CRTIRoR::setSouthWallPresent(bool bPresent)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd <<SOUTH_WALL_PRESENT << (bPresent?"1":"0") <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, SOUTH_WALL_PRESENT);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSouthWallPeesent] bPresent ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    m_bSouthWallPresent = bPresent;
    return nErr;}


int CRTIRoR::getRoofOpenOrder(bool &bRoofFirst)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    ssCmd << OPEN_ROOF_ORDER  <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, OPEN_ROOF_ORDER);

    if(nErr) {
        return nErr;
    }
    try {
        bRoofFirst = std::stoi(sResp) ? false:true;
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofOpenOrder] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        bRoofFirst = true;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofOpenOrder] bRoofFirst = " << (bRoofFirst?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::setRoofOpenOrder(bool bRoofFirst)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << OPEN_ROOF_ORDER << (bRoofFirst?"1":"0") <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, OPEN_ROOF_ORDER);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setRoofOpenOrder] m_bOpenRoofFirst ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    m_bOpenRoofFirst = bRoofFirst;
    return nErr;
}


bool CRTIRoR::isRorMoving()
{
    bool bIsMoving;
    int nTmp;
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << SLEW_STATUS_GET  <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, SLEW_STATUS_GET);

    if(nErr ) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isRoRMoving] ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return false;
    }

    bIsMoving = false;
    try {
        nTmp = std::stoi(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isRoRMoving] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        nTmp = MOVE_NONE;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isRoRMoving] nTmp : " << nTmp << std::endl;
    m_sLogFile.flush();
#endif
    if(nTmp != MOVE_NONE)
        bIsMoving = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isRoRMoving] bIsMoving : " << (bIsMoving?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return bIsMoving;
}


int CRTIRoR::openRoof()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    double dRoofVolts;
    double dRoofCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getBatteryLevels( dRoofVolts, dRoofCutOff);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openRoof] Opening shutter" << std::endl;
    m_sLogFile.flush();
#endif

    ssCmd << OPEN_ROOF_CMD  <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, OPEN_ROOF_CMD);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openRoof] ERROR = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openRoof] response = " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    if(sResp.size() && sResp.at(0) == 'L') { // battery LOW.. can't open
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openRoof] Voltage too low to open" << std::endl;
        m_sLogFile.flush();
#endif
        nErr = MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_BATTERY_LOW);
    }
    if(sResp.size() && sResp.at(0) == 'R') { // Raining. can't open
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [openRoof] Voltage too low to open" << std::endl;
        m_sLogFile.flush();
#endif
        nErr = MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_RAINING);
    }

    return nErr;
}

int CRTIRoR::closeRoof()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    double dRoofVolts;
    double dRoofCutOff;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;


    getBatteryLevels(dRoofVolts, dRoofCutOff);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeRoof] Closing shutter" << std::endl;
    m_sLogFile.flush();
#endif

    ssCmd << CLOSE_ROOF_CMD  <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, CLOSE_ROOF_CMD);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [closeRoof] ERROR Closing shutter : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    if(sResp.size() && sResp.at(0) == 'L') { // batteryb LOW.. can't close :(
        nErr = MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_BATTERY_LOW);
    }

    return nErr;
}




int CRTIRoR::getFirmwareVersion(std::string &sVersion, float &fVersion)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;
    std::stringstream ssCmd;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << VERSION_ROOF_GET  <<"#";
    nErr = roofCommand(ssCmd.str(), sResp, VERSION_ROOF_GET);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] ERROR = " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

    nErr = parseFields(sResp,firmwareFields, 'V');
    if(nErr) {
        sVersion = "N/A";
        fVersion = 0.0;
        return PLUGIN_OK;
    }

    if(firmwareFields.size()>0) {
        try {
            sVersion.assign(firmwareFields[0]);
            fVersion = std::stof(firmwareFields[0]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] conversion exception = " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            fVersion = 0;
        }
    }
    else {
        sVersion = "N/A";
        fVersion = 0.0;
    }
    return nErr;
}

int CRTIRoR::getFirmwareVersion(float &fVersion)
{
    int nErr = PLUGIN_OK;

    if(m_fVersion == 0.0f) {
        nErr = getFirmwareVersion(m_sFirmwareVersion, m_fVersion);
        if(nErr)
            return nErr;
    }

    fVersion = m_fVersion;

    return nErr;
}


int CRTIRoR::isOpenComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getRoofState(m_nRoofState);
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_CMDFAILED);
    if(m_nRoofState == OPEN){
        m_bRoofOpened = true;
        bComplete = true;
    }
    else {
        m_bRoofOpened = false;
        bComplete = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isOpenComplete] bComplete = " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::isCloseComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;


    nErr = getRoofState(m_nRoofState);
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_CMDFAILED);
    if(m_nRoofState == CLOSED){
        m_bRoofOpened = false;
        bComplete = true;
    }
    else {
        m_bRoofOpened = true;
        bComplete = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isCloseComplete] bComplete = " << (bComplete?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int CRTIRoR::abortCurrentCommand()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << ABORT_MOVE_CMD << "#";
    nErr = roofCommand( ssCmd.str(), sResp, ABORT_MOVE_CMD);

    return nErr;
}


#pragma mark - Getter / Setter


int CRTIRoR::getDefaultDir(bool &bNormal)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    bNormal = true;

    ssCmd << REVERSED_ROOF_CMD << "#";
    nErr = roofCommand( ssCmd.str(), sResp, REVERSED_ROOF_CMD);

    if(nErr) {
        return nErr;
    }
    try {
        bNormal = std::stoi(sResp) ? false:true;
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDefaultDir] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        bNormal = true;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDefaultDir] bNormal = " << (bNormal?"True":"False") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::setDefaultDir(bool bNormal)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << REVERSED_ROOF_CMD << (bNormal?"0":"1") << "#";

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDefaultDir] bNormal = " << (bNormal?"True":"False") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDefaultDir] ssTmp = " << ssCmd.str() << std::endl;
    m_sLogFile.flush();
#endif

    nErr = roofCommand(ssCmd.str(), sResp, REVERSED_ROOF_CMD);
    return nErr;

}

int CRTIRoR::getRainSensorStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    nStatus = NOT_RAINING;

    ssCmd << RAIN_ROOF_GET << "#";
    nErr = roofCommand( ssCmd.str(), sResp, RAIN_ROOF_GET);

    if(nErr) {
        return nErr;
    }

    try {
        nStatus = std::stoi(sResp) ? false:true;
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRainSensorStatus] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        nStatus = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRainSensorStatus] nStatus = " << (nStatus?"NOT RAINING":"RAINING") << std::endl;
    m_sLogFile.flush();
#endif

    m_nIsRaining = nStatus;
    return nErr;
}


int CRTIRoR::getRoofSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << SPEED_ROOF_CMD << "#";
    nErr = roofCommand( ssCmd.str(), sResp, SPEED_ROOF_CMD);

    if(nErr) {
        return nErr;
    }

    try {
        nSpeed = std::stoi(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofSpeed] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        nSpeed = 0;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofSpeed] nSpeed = " << nSpeed << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int CRTIRoR::setRoofSpeed(int nSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << SPEED_ROOF_CMD << nSpeed << "#";
    nErr = roofCommand(ssCmd.str(), sResp, SPEED_ROOF_CMD);

    return nErr;
}

int CRTIRoR::getRoofAcceleration(int &nAcceleration)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << ACCELERATION_ROOF_CMD << "#";
    nErr = roofCommand( ssCmd.str(), sResp, ACCELERATION_ROOF_CMD);

    if(nErr) {
        return nErr;
    }

    try {
        nAcceleration = std::stoi(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofAcceleration] conversion exception = " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        nAcceleration = 0;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRoofAcceleration] nAcceleration = " << nAcceleration << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;
}

int CRTIRoR::setRoofAcceleration(int nAcceleration)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << ACCELERATION_ROOF_CMD << nAcceleration << "#";
    nErr = roofCommand(ssCmd.str(), sResp, ACCELERATION_ROOF_CMD);
    return nErr;
}

void CRTIRoR::getRoofStatus(int &nStatus)
{
    getRoofState(nStatus);
}

int CRTIRoR::restoreRoofMotorSettings()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;
    int nDummy;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    ssCmd << RESTORE_MOTOR_DEFAULT << "#";
    nErr = roofCommand( ssCmd.str(), sResp, RESTORE_MOTOR_DEFAULT);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [restoreRoofMotorSettings] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    nErr = getRoofAcceleration(nDummy);
    nErr |= getRoofSpeed(nDummy);
    return nErr;
}

void CRTIRoR::enableRainStatusFile(bool bEnable)
{
    m_bSaveRainStatus = bEnable;
}

void CRTIRoR::getRainStatusFileName(std::string &fName)
{
    fName.assign(m_sRainStatusfilePath);
}

void CRTIRoR::writeRainStatus()
{
    int nStatus;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] m_nIsRaining = " <<(m_nIsRaining==RAINING?"Raining":"Not Raining") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] m_bSaveRainStatus = " <<(m_bSaveRainStatus?"YES":"NO") << std::endl;
    m_sLogFile.flush();
#endif

    if(m_bSaveRainStatus) {
        getRainSensorStatus(nStatus);
        if(m_nRainStatus != nStatus) {
            m_nRainStatus = nStatus;
            if(m_RainStatusfile.is_open())
                m_RainStatusfile.close();
            try {
                m_RainStatusfile.open(m_sRainStatusfilePath, std::ios::out |std::ios::trunc);
                if(m_RainStatusfile.is_open()) {
                    m_RainStatusfile << "Raining:" << (nStatus == RAINING?"YES":"NO") << std::endl;
                    m_RainStatusfile.close();
                }
            }
            catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [writeRainStatus] Error writing file = " << e.what() << std::endl;
                m_sLogFile.flush();
#endif
                if(m_RainStatusfile.is_open())
                    m_RainStatusfile.close();
            }
        }
    }
}


int CRTIRoR::getMACAddress(std::string &MACAddress)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << ETH_MAC_ADDRESS << "#";
    nErr = roofCommand( ssCmd.str(), sResp, ETH_MAC_ADDRESS);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getMACAddress] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    MACAddress.assign(sResp);
    return nErr;
}

int CRTIRoR::reconfigureNetwork()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bNetworkConnected) {
        ssCmd << ETH_RECONFIG << "#";
        nErr = roofCommand( ssCmd.str(), sResp, 0x00);
    }
    else {
        ssCmd << ETH_RECONFIG << "#";
        nErr = roofCommand( ssCmd.str(), sResp, ETH_RECONFIG);
    }

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [reconfigureNetwork] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}

int CRTIRoR::getUseDHCP(bool &bUseDHCP)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_DHCP << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_DHCP);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getUseDHCP] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    bUseDHCP = false;
    if(sResp.size())
        bUseDHCP = (sResp.at(0) == '0'? false: true);
    return nErr;
}

int CRTIRoR::setUseDHCP(bool bUseDHCP)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_DHCP << (bUseDHCP?"1":"0") << "#";
    nErr = roofCommand(ssCmd.str(), sResp, IP_DHCP);
    return nErr;
}

int CRTIRoR::getIpAddress(std::string &IpAddress)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_ADDRESS << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_ADDRESS);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getIpAddress] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    IpAddress.assign(sResp);
    return nErr;
}

int CRTIRoR::setIpAddress(std::string IpAddress)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_ADDRESS << IpAddress << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_ADDRESS);
    return nErr;
}

int CRTIRoR::getSubnetMask(std::string &subnetMask)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_SUBNET << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_SUBNET);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSubnetMask] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    subnetMask.assign(sResp);
    return nErr;

}

int CRTIRoR::setSubnetMask(std::string subnetMask)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_SUBNET << subnetMask << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_SUBNET);

    return nErr;
}

int CRTIRoR::getIPGateway(std::string &IpAddress)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_GATEWAY << "#";
    nErr = roofCommand( ssCmd.str(), sResp, IP_GATEWAY);

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getIPGateway] ERROR = " <<nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    IpAddress.assign(sResp);
    return nErr;
}

int CRTIRoR::setIPGateway(std::string IpAddress)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    ssCmd << IP_GATEWAY << IpAddress << "#";
    nErr = roofCommand(ssCmd.str(), sResp, IP_GATEWAY);
    return nErr;
}


int CRTIRoR::parseFields(const std::string sResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] sResp = " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    if(sResp.size()==0) {
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_CMDFAILED);
    }

    std::stringstream ssTmp(sResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] sSegment = " << sSegment << std::endl;
        m_sLogFile.flush();
#endif
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, ERR_CMDFAILED);
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] Done all good." << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

#ifdef PLUGIN_DEBUG
const std::string CRTIRoR::getTimeStamp()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
#endif
