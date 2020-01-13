//
//  RTIRoR.cpp
//  RTIRoR X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "rti_ror.h"

CRTIRoR::CRTIRoR()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;
    m_dHomeAz = 0;
    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;
    m_bShutterOpened = false;
    m_bHomed = false;
    m_fVersion = 0.0;
    m_nIsRaining = NOT_RAINING;


    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,RTI_LOG_BUFFER_SIZE);

#ifdef RTI_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\RTIRoRLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RTIRoRLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RTIRoRLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CRTIRoR::~CRTIRoR()
{
#ifdef	RTI_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif

}

int CRTIRoR::Connect(const char *pszPort)
{
    int nErr;
    
#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 9600 8N1
    nErr = m_pSerx->open(pszPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_bIsConnected = true;
    m_bHomed = false;

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::Connect connected to %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // the arduino take over a second to start as it need to init the XBee and there is 1100 ms pause in the code :(
    if(m_pSleeper)
        m_pSleeper->sleep(2000);

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::Connect Getting Firmware\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::Connect] Error Getting Firmware.\n", timestamp);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::Connect Got Firmware %s ( %f )\n", timestamp, m_szFirmwareVersion, m_fVersion);
    fflush(Logfile);
#endif

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
    m_bHomed = false;

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::Disconnect] m_bIsConnected = %d\n", timestamp, m_bIsConnected);
    fflush(Logfile);
#endif
}


int CRTIRoR::readResponse(char *szRespBuffer, int nBufferLen)
{
    int nErr = RTI_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRTIRoR::readResponse] readFile error\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined RTI_DEBUG && RTI_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CRTIRoR::readResponse Timeout while waiting for response from controller\n", timestamp);
            fflush(Logfile);
#endif

            nErr = RTI_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}


int CRTIRoR::roofCommand(const char *pszCmd, char *pszResult, char respCmdCode, int nResultMaxLen)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::roofCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    if (!respCmdCode)
        return nErr;

    // read response
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CRTIRoR::roofCommand ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::roofCommand response : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif


    if (szResp[0] != respCmdCode)
        nErr = RTI_BAD_CMD_RESPONSE;

    if(pszResult)
        strncpy(pszResult, &szResp[1], nResultMaxLen);

    return nErr;

}


int CRTIRoR::getShutterState(int &nState)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> shutterStateFileds;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
		
    nErr = roofCommand("M#", szResp, 'M', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::getShutterState] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getShutterState] response = '%s'\n", timestamp, szResp);
    fflush(Logfile);
#endif

    nState = atoi(szResp);

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getShutterState] nState = '%d'\n", timestamp, nState);
    fflush(Logfile);
#endif

    return nErr;
}


int CRTIRoR::getStepPerRev(int &nStepPerRev)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("T#", szResp, 'T', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::getDomeStepPerRev] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    nStepPerRev = atoi(szResp);
    m_nNbStepPerRev = nStepPerRev;
    return nErr;
}


int CRTIRoR::getBatteryLevels(double &dShutterVolts, double &dShutterCutOff)
{
    int nErr = RTI_OK;
    int rc = 0;
    char szResp[SERIAL_BUFFER_SIZE];

    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("K#", szResp, 'K', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    if(strlen(szResp)<2) { // no shutter value
        dShutterVolts = -1;
        dShutterCutOff = -1;
        return nErr;
    }

    rc = sscanf(szResp, "%lf,%lf", &dShutterVolts, &dShutterCutOff);
    if(rc == 0) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] sscanf ERROR\n", timestamp);
        fflush(Logfile);
#endif
        return COMMARTI_FAILED;
    }

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] shutterVolts = %f\n", timestamp, dShutterVolts);
    fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] dShutterCutOff = %f\n", timestamp, dShutterCutOff);
    fflush(Logfile);
#endif

    dShutterVolts = dShutterVolts / 100.0;
    dShutterCutOff = dShutterCutOff / 100.0;
#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] shutterVolts = %f\n", timestamp, dShutterVolts);
    fprintf(Logfile, "[%s] [CRTIRoR::getBatteryLevels] dShutterCutOff = %f\n", timestamp, dShutterCutOff);
    fflush(Logfile);
#endif

    return nErr;
}

int CRTIRoR::setBatteryCutOff(double dShutterCutOff)
{
    int nErr = RTI_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nShutCutOff;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::setBatteryCutOff] dShutterCutOff  = %3.2f\n", timestamp, dShutterCutOff);
    fflush(Logfile);
#endif

    nShutCutOff = dShutterCutOff * 100.0;
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "K%d#", nShutCutOff);
    nErr = roofCommand(szBuf, szResp, 'K', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::setBatteryCutOff] dShutterCutOff ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    return nErr;
}



int CRTIRoR::gotoAzimuth(double dNewAz)
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;


    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;
    

    m_dCurrentAzPosition = dNewAz;
    return RTI_OK;
}

int CRTIRoR::openShutter()
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::openShutter] Opening shutter\n", timestamp);
    fflush(Logfile);
#endif

    nErr = roofCommand("O#", szResp, 'O', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::openShutter] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

    return nErr;
}

int CRTIRoR::closeShutter()
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::closeShutter] Closing shutter\n", timestamp);
    fflush(Logfile);
#endif

    nErr = roofCommand("C#", szResp, 'C', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::openShutter] closeShutter = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

    return nErr;
}

int CRTIRoR::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = RTI_OK;
    int i;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("V#", szResp, 'V', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RTI_DEBUG && RTI_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::getFirmwareVersion] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = parseFields(szResp,firmwareFields, 'V');
    if(nErr) {
        strncpy(szVersion, szResp, nStrMaxLen);
        m_fVersion = atof(szResp);
        return RTI_OK;
    }

    nErr = parseFields(firmwareFields[0].c_str(),versionFields, '.');
    if(versionFields.size()>1) {
        strVersion=versionFields[0]+".";
        for(i=1; i<versionFields.size(); i++) {
            strVersion+=versionFields[i];
        }
        strncpy(szVersion, szResp, nStrMaxLen);
        m_fVersion = atof(strVersion.c_str());
    }
    else {
        strncpy(szVersion, szResp, nStrMaxLen);
        m_fVersion = atof(szResp);
    }
    return nErr;
}

int CRTIRoR::getFirmwareVersion(float &fVersion)
{
    int nErr = RTI_OK;

    if(m_fVersion == 0.0f) {
        nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
    }

    fVersion = m_fVersion;

    return nErr;
}

int CRTIRoR::goHome()
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bHomed = true;

    return RTI_OK;
}



int CRTIRoR::isGoToComplete(bool &bComplete)
{
    bComplete = true;
    return RTI_OK;
}

int CRTIRoR::isOpenComplete(bool &bComplete)
{
    int nErr = RTI_OK;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::isOpenComplete bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CRTIRoR::isCloseComplete(bool &bComplete)
{
    int nErr = RTI_OK;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

#if defined RTI_DEBUG && RTI_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRTIRoR::isCloseComplete bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}




int CRTIRoR::isFindHomeComplete(bool &bComplete)
{
    bComplete = true;
    m_dCurrentAzPosition = m_dHomeAz;
    return RTI_OK;
}


void CRTIRoR::syncDome(double dAz, double dEl)
{
    m_dCurrentAzPosition = dAz;
    m_dCurrentElPosition = dEl;
}


int CRTIRoR::abortCurrentCommand()
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("A#", szResp, 'A', SERIAL_BUFFER_SIZE);

    return nErr;
}


int CRTIRoR::getNbTicksPerRev()
{
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getNbTicksPerRev] m_bIsConnected = %s\n", timestamp, m_bIsConnected?"True":"False");
    fflush(Logfile);
#endif

    if(m_bIsConnected)
        getStepPerRev(m_nNbStepPerRev);

    return m_nNbStepPerRev;
}


double CRTIRoR::getHomeAz()
{
    return m_dHomeAz;
}



double CRTIRoR::getCurrentAz()
{

    return m_dCurrentAzPosition;
}

double CRTIRoR::getCurrentEl()
{
    return m_dCurrentElPosition;
}

int CRTIRoR::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}


int CRTIRoR::getDefaultDir(bool &bNormal)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bNormal = true;
    nErr = roofCommand("Y#", szResp, 'Y', SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    bNormal = atoi(szResp) ? false:true;
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fflush(Logfile);
#endif


    return nErr;
}

int CRTIRoR::setDefaultDir(bool bNormal)
{
    int nErr = RTI_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "Y %1d#", bNormal?0:1);

#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::setDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fprintf(Logfile, "[%s] [CRTIRoR::setDefaultDir] szBuf =  %s\n", timestamp, szBuf);
    fflush(Logfile);
#endif

    nErr = roofCommand(szBuf, szResp, 'y', SERIAL_BUFFER_SIZE);
    return nErr;

}

int CRTIRoR::getRainSensorStatus(int &nStatus)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nStatus = NOT_RAINING;
    nErr = roofCommand("F#", szResp, 'F', SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    nStatus = atoi(szResp) ? false:true;
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getRainSensorStatus] nStatus =  %s\n", timestamp, nStatus?"NOT RAINING":"RAINING");
    fflush(Logfile);
#endif


    m_nIsRaining = nStatus;
    return nErr;
}


int CRTIRoR::getShutterSpeed(int &nSpeed)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("R#", szResp, 'R', SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    nSpeed = atoi(szResp);
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getShutterSpeed] nSpeed =  %d\n", timestamp, nSpeed);
    fflush(Logfile);
#endif

    return nErr;
}

int CRTIRoR::setShutterSpeed(int nSpeed)
{
    int nErr = RTI_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "R%d#", nSpeed);
    nErr = roofCommand(szBuf, szResp, 'R', SERIAL_BUFFER_SIZE);

    return nErr;
}

int CRTIRoR::getShutterAcceleration(int &nAcceleration)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("E#", szResp, 'E', SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    nAcceleration = atoi(szResp);
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getShutterAcceleration] nAcceleration =  %d\n", timestamp, nAcceleration);
    fflush(Logfile);
#endif
    return nErr;
}

int CRTIRoR::setShutterAcceleration(int nAcceleration)
{
    int nErr = RTI_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "E%d#", nAcceleration);
    nErr = roofCommand(szBuf, szResp, 'E', SERIAL_BUFFER_SIZE);
    return nErr;
}

int	CRTIRoR::getSutterWatchdogTimerValue(int &nValue)
{
	int nErr = RTI_OK;
	char szResp[SERIAL_BUFFER_SIZE];
	
	if(!m_bIsConnected)
		return NOT_CONNECTED;
	
	nErr = roofCommand("I#", szResp, 'I', SERIAL_BUFFER_SIZE);
	if(nErr) {
		return nErr;
	}
	
	nValue = atoi(szResp)/1000; // value is in ms
#ifdef RTI_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CRTIRoR::getSutterWatchdogTimerValue] nValue =  %d\n", timestamp, nValue);
	fflush(Logfile);
#endif
	return nErr;
}

int	CRTIRoR::setSutterWatchdogTimerValue(const int &nValue)
{
	int nErr = RTI_OK;
	char szBuf[SERIAL_BUFFER_SIZE];
	char szResp[SERIAL_BUFFER_SIZE];
	
	if(!m_bIsConnected)
		return NOT_CONNECTED;

	snprintf(szBuf, SERIAL_BUFFER_SIZE, "I%d#", nValue * 1000); // value is in ms
	nErr = roofCommand(szBuf, szResp, 'I', SERIAL_BUFFER_SIZE);
	return nErr;
}


int CRTIRoR::getRainTimerValue(int &nValue)
{
    int nErr = RTI_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = roofCommand("N#", szResp, 'N', SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    nValue = atoi(szResp);
#ifdef RTI_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRTIRoR::getRainTimerValue] nValue =  %d\n", timestamp, nValue);
    fflush(Logfile);
#endif
    return nErr;

}

int CRTIRoR::setRainTimerValue(const int &nValue)
{
    int nErr = RTI_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "N%d#", nValue);
    nErr = roofCommand(szBuf, szResp, 'N', SERIAL_BUFFER_SIZE);
    return nErr;
}


int CRTIRoR::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = RTI_OK;
    std::string sSegment;
    if(!pszResp) {
#ifdef RTI_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::setDefaultDir] pszResp is NULL\n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    if(!strlen(pszResp)) {
#ifdef RTI_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIRoR::setDefaultDir] pszResp is enpty\n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

