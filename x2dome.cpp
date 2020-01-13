#include "x2dome.h"


X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXForMounts,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyXForMounts;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_nBattRequest = 0;
    
    m_RTIRoR.setSerxPointer(pSerX);
    m_RTIRoR.setSleeprPinter(pSleeper);

    if (m_pIniUtil)
    {   
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)					
{
    int nErr;
    char szPort[SERIAL_BUFFER_SIZE];

    X2MutexLocker ml(GetMutex());

    // get serial port device name
    portNameOnToCharPtr(szPort,SERIAL_BUFFER_SIZE);
    nErr = m_RTIRoR.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
        // nErr = ERR_COMMOPENING;
    }
    else
        m_bLinked = true;

	return nErr;
}

int X2Dome::terminateLink(void)					
{
    X2MutexLocker ml(GetMutex());

    m_RTIRoR.Disconnect();
	m_bLinked = false;

    return SB_OK;
}

 bool X2Dome::isLinked(void) const				
{
    return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    
    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    double dShutterBattery, dShutterCutOff;
    bool nReverseDir;
    int n_nbStepPerRev;
    int nRainSensorStatus = NOT_RAINING;
    int nSSpeed;
    int nSAcc;
	int nWatchdog;
    int nRainTimer;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("RTI-RoR.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());

    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);
    // set controls state depending on the connection state
    if(m_bLinked) {
        dx->setEnabled("needReverse",true);
        nErr = m_RTIRoR.getDefaultDir(nReverseDir);
        if(nReverseDir)
            dx->setChecked("needReverse",false);
        else
            dx->setChecked("needReverse",true);

        // read values from dome controller
        n_nbStepPerRev = m_RTIRoR.getNbTicksPerRev();
        snprintf(szTmpBuf,16,"%d",n_nbStepPerRev);
        dx->setText("ticksPerStroke", szTmpBuf);

        dx->setEnabled("shutterSpeed",true);
        m_RTIRoR.getShutterSpeed(nSSpeed);
        dx->setPropertyInt("shutterSpeed","value", nSSpeed);

        dx->setEnabled("shutterAcceleration",true);
        m_RTIRoR.getShutterAcceleration(nSAcc);
        dx->setPropertyInt("shutterAcceleration","value", nSAcc);

		dx->setEnabled("shutterWatchdog",true);
		m_RTIRoR.getSutterWatchdogTimerValue(nWatchdog);
		dx->setPropertyInt("shutterWatchdog", "value", nWatchdog);

        dx->setEnabled("rainCheckInterval",true);
        m_RTIRoR.getRainTimerValue(nRainTimer);
        dx->setPropertyInt("rainCheckInterval", "value", nRainTimer);

        dx->setEnabled("lowShutBatCutOff",true);


        m_RTIRoR.getBatteryLevels(dShutterBattery, dShutterCutOff);
        dx->setPropertyDouble("lowShutBatCutOff","value", dShutterCutOff);
        snprintf(szTmpBuf,16,"%2.2f V",dShutterBattery);
        dx->setText("shutterBatteryLevel", szTmpBuf);

        nErr = m_RTIRoR.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            dx->setText("rainStatus", "--");
        else {
            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "<html><head/><body><p><span style=\" color:#%s;\">%s</span></p></body></html>", nRainSensorStatus==NOT_RAINING?"00ff00":"ff0000", nRainSensorStatus==NOT_RAINING ? "Not raining" : "Raining");
            dx->setText("rainStatus", szTmpBuf);
        }
    }
    else {
		dx->setEnabled("pushButton",false);
		dx->setEnabled("ticksPerStroke",false);
        dx->setEnabled("needReverse",false);
        dx->setEnabled("ticksPerRev",false);
        dx->setEnabled("shutterSpeed",false);
        dx->setEnabled("shutterAcceleration",false);
		dx->setEnabled("shutterWatchdog",false);
        dx->setEnabled("rainCheckInterval",false);
        dx->setEnabled("lowShutBatCutOff",false);
        dx->setPropertyString("shutterBatteryLevel","text", "--");
        dx->setPropertyString("rainStatus","text", "--");
    }

    m_nBattRequest = 0;
    

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        dx->propertyInt("shutterSpeed", "value", nSSpeed);
        dx->propertyInt("shutterAcceleration", "value", nSAcc);
		dx->propertyInt("shutterWatchdog", "value", nWatchdog);
        dx->propertyInt("rainCheckInterval", "value", nRainTimer);
        dx->propertyDouble("lowShutBatCutOff", "value", dShutterCutOff);
        nReverseDir = dx->isChecked("needReverse");
        if(m_bLinked) {
            m_RTIRoR.setDefaultDir(!nReverseDir);
            m_RTIRoR.setRainTimerValue(nRainTimer);
            m_RTIRoR.setShutterSpeed(nSSpeed);
            m_RTIRoR.setShutterAcceleration(nSAcc);
            m_RTIRoR.setSutterWatchdogTimerValue(nWatchdog);
            m_RTIRoR.setBatteryCutOff(dShutterCutOff);
		}

    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr;
    double dShutterBattery, dShutterCutOff;
    char szTmpBuf[SERIAL_BUFFER_SIZE];    
    int nRainSensorStatus = NOT_RAINING;

    if (!strcmp(pszEvent, "on_timer"))
    {
        // don't ask to often
        if (!(m_nBattRequest%4)) {
            m_RTIRoR.getBatteryLevels(dShutterBattery, dShutterCutOff);
            if(dShutterBattery>=0.0f)
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "%2.2f V", dShutterBattery);
            else
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "--");
            uiex->setText("shutterBatteryLevel", szTmpBuf);
        }
        m_nBattRequest++;
        nErr = m_RTIRoR.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            uiex->setText("rainStatus", "--");
        else {

            snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "<html><head/><body><p><span style=\" color:#%s;\">%s</span></p></body></html>", nRainSensorStatus==NOT_RAINING?"00ff00":"ff0000", nRainSensorStatus==NOT_RAINING ? "Not raining" : "Raining");
            uiex->setText("rainStatus", szTmpBuf);
        }
    }
}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const					
{
	str = "RTIRoR";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const					
{
    str = "RTIRoR";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
    str = "RTIRoR Dome Rotation Kit";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)					
{

    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
		X2MutexLocker ml(GetMutex());
        m_RTIRoR.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    str = "RTIRoR Dome Rotation Kit";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const	
{
    str = "RTIRoR Dome Rotation Kit X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    *pdAz = m_RTIRoR.getCurrentAz();
    *pdEl = m_RTIRoR.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_RTIRoR.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{
    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    m_RTIRoR.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_RTIRoR.openShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

    nErr = m_RTIRoR.closeShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    *pbComplete = true;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;
    
	X2MutexLocker ml(GetMutex());

	nErr = m_RTIRoR.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_RTIRoR.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    *pbComplete = true;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    *pbComplete = true;
    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_RTIRoR.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    if(!m_bLinked)
        return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	m_RTIRoR.syncDome(dAz, dEl);

    return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[SERIAL_BUFFER_SIZE];

    portNameOnToCharPtr(szPortName, SERIAL_BUFFER_SIZE);

    str = szPortName;

}

void X2Dome::setPortName(const char* szPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, szPort);
    
}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}



