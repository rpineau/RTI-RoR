#include "x2dome.h"


X2Dome::X2Dome(const char* pszSelection, const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyX,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			    pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyX			            = pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bSettingPanID = false;
    m_bSettingNetwork = false;
        
    m_RTIRoR.setSerxPointer(pSerX);

    if (m_pIniUtil)
    {
        m_bLogRainStatus = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_LOG_RAIN_STATUS, false);
        m_RTIRoR.enableRainStatusFile(m_bLogRainStatus);
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyX)
		delete m_pTheSkyX;
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

    // get serial port device name, and IP and port if the connection is over TCP.
    portNameOnToCharPtr(szPort,SERIAL_BUFFER_SIZE);
    nErr = m_RTIRoR.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
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
    else if (!strcmp(pszName, DomeHasHighlyRelaibleOpenCloseSensors_Name))
        *ppVal = dynamic_cast<DomeHasHighlyRelaibleOpenCloseSensors*>(this);

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
    std::stringstream sTmpBuf;
    std::string fName;
    double dRoofBattery =0 , dRoofCutOff = 0;
    bool nReverseDir = false;
    int nRainSensorStatus = NOT_RAINING;
    int nSSpeed = 0;
    int nSAcc = 0;
    double  batShutCutOff = 0;
    std::string sDummy;
    bool bUseDHCP = false;
    std::string sIpAddress;
    std::string sSubnetMask;
    std::string sGatewayIP;
    
    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("RTI-RoR.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());

    // disbale these for now, these will be for the number of shuter and sequence for openging/closing
    dx->setEnabled("checkBox_3",false);
    dx->setEnabled("comboBox_2",false);
    
    
    if(m_bLogRainStatus) {
        dx->setChecked("checkBox",true);
        m_RTIRoR.getRainStatusFileName(fName);
        dx->setPropertyString("filePath","text", fName.c_str());
    }
    else {
        dx->setChecked("checkBox",false);
        dx->setPropertyString("filePath","text", "");
    }

    if(m_bLinked) {
        dx->setEnabled("needReverse",true);
        nErr = m_RTIRoR.getDefaultDir(nReverseDir);
        if(nReverseDir)
            dx->setChecked("needReverse",false);
        else
            dx->setChecked("needReverse",true);

        // read values from dome controller
        dx->setEnabled("shutterSpeed",true);
        nErr = m_RTIRoR.getRoofSpeed(nSSpeed);
        dx->setPropertyInt("shutterSpeed","value", nSSpeed);

        dx->setEnabled("shutterAcceleration",true);
        m_RTIRoR.getRoofAcceleration(nSAcc);
        dx->setPropertyInt("shutterAcceleration","value", nSAcc);

        dx->setEnabled("pushButton_4", true);

        dx->setEnabled("lowShutBatCutOff",true);


        m_RTIRoR.getBatteryLevels(dRoofBattery, dRoofCutOff);
        dx->setPropertyDouble("lowShutBatCutOff","value", dRoofCutOff);
        std::stringstream().swap(sTmpBuf);
        if(dRoofBattery>=0.0f)
            sTmpBuf << std::fixed << std::setprecision(2) << dRoofBattery << " V";
        else
            sTmpBuf << "--";
        dx->setPropertyString("shutterBatteryLevel","text", sTmpBuf.str().c_str());

        nErr = m_RTIRoR.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            dx->setPropertyString("rainStatus","text", "--");
        else {
            std::stringstream().swap(sTmpBuf);
            sTmpBuf << (nRainSensorStatus==NOT_RAINING ? "<html><head/><body><p><span style=\" color:#00FF00;\">Not raining</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Raining</span></p></body></html>");
            dx->setPropertyString("rainStatus","text", sTmpBuf.str().c_str());
        }

        nErr = m_RTIRoR.getMACAddress(sDummy);
        if(nErr)
            sDummy = "";
        dx->setPropertyString("MACAddress", "text", sDummy.c_str());

        nErr = m_RTIRoR.getUseDHCP(bUseDHCP);
        dx->setChecked("checkBox_2", bUseDHCP);
        if(bUseDHCP) {
            dx->setEnabled("IPAddress", false);
            dx->setEnabled("SubnetMask", false);
            dx->setEnabled("GatewayIP", false);
        }
        else { // not using dhcp so the field are editable
            dx->setEnabled("IPAddress", true);
            dx->setEnabled("SubnetMask", true);
            dx->setEnabled("GatewayIP", true);
        }

        nErr = m_RTIRoR.getIpAddress(sIpAddress);
        if(nErr)
            sIpAddress = "";
        dx->setPropertyString("IPAddress", "text", sIpAddress.c_str());

        nErr = m_RTIRoR.getSubnetMask(sSubnetMask);
        if(nErr)
            sSubnetMask = "";
        dx->setPropertyString("SubnetMask", "text", sSubnetMask.c_str());

        nErr = m_RTIRoR.getIPGateway(sGatewayIP);
        if(nErr)
            sGatewayIP = "";
        dx->setPropertyString("GatewayIP", "text", sGatewayIP.c_str());
        
        dx->setEnabled("pushButton",true);
    }
    else {
        dx->setEnabled("needReverse", false);
        dx->setEnabled("shutterSpeed", false);
        dx->setEnabled("shutterAcceleration", false);
        dx->setEnabled("lowShutBatCutOff", false);
        dx->setPropertyString("domeBatteryLevel", "text", "--");
        dx->setPropertyString("shutterBatteryLevel", "text", "--");
        dx->setEnabled("pushButton_2", false);
        dx->setEnabled("pushButton", false);
        dx->setEnabled("pushButton_3", false);
        dx->setEnabled("pushButton_4", false);
        dx->setPropertyString("rainStatus","text", "--");
    
        dx->setEnabled("checkBox_2", false);
        dx->setPropertyString("MACAddress", "text", "");
        dx->setEnabled("IPAddress", false);
        dx->setPropertyString("IPAddress", "text", "");
        dx->setEnabled("SubnetMask", false);
        dx->setPropertyString("SubnetMask", "text", "");
        dx->setEnabled("GatewayIP", false);
        dx->setPropertyString("GatewayIP", "text", "");
        dx->setEnabled("pushButton_5", false);
    }
 
    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK) {
        dx->propertyInt("shutterSpeed", "value", nSSpeed);
        dx->propertyInt("shutterAcceleration", "value", nSAcc);
        dx->propertyDouble("lowShutBatCutOff", "value", batShutCutOff);
        nReverseDir = dx->isChecked("needReverse");
        m_bLogRainStatus = dx->isChecked("checkBox");
        m_RTIRoR.enableRainStatusFile(m_bLogRainStatus);

        if(m_bLinked) {
            m_RTIRoR.setDefaultDir(!nReverseDir);
            m_RTIRoR.setRoofSpeed(nSSpeed);
            m_RTIRoR.setRoofAcceleration(nSAcc);
        }

        // save the values to persistent storage
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_LOG_RAIN_STATUS, m_bLogRainStatus);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr;
    double dRoofBattery, dRoofCutOff;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    std::stringstream sTmpBuf;
    std::stringstream sErrorMessage;
    std::string fName;
    std::string sDummy;
    int nRainSensorStatus = NOT_RAINING;
    int nSpeed;
    int nAcc;
    bool bUseDHCP;

    if(!m_bLinked)
        return;

    if (!strcmp(pszEvent, "on_timer"))
    {
            if( m_bLinked) {
                uiex->setEnabled("shutterSpeed",true);
                m_RTIRoR.getRoofSpeed(nSpeed);
                uiex->setPropertyInt("shutterSpeed","value", nSpeed);

                uiex->setEnabled("shutterAcceleration",true);
                m_RTIRoR.getRoofAcceleration(nAcc);
                uiex->setPropertyInt("shutterAcceleration","value", nAcc);

                uiex->setEnabled("pushButton_4", true);
            }
            else {
                uiex->setPropertyInt("shutterSpeed","value", 0);
                uiex->setPropertyInt("shutterAcceleration","value", 0);
                uiex->setPropertyInt("shutterWatchdog", "value", 0);
                uiex->setEnabled("shutterSpeed",false);
                uiex->setEnabled("shutterAcceleration",false);
                uiex->setEnabled("pushButton_4", false);
            }

        }
        if(m_bLinked) {
            if(m_bSettingNetwork) {
                if(m_SetNetworkTimer.GetElapsedSeconds()>5) {
                    nErr = m_RTIRoR.getUseDHCP(bUseDHCP);
                    if(bUseDHCP) {
                        uiex->setEnabled("IPAddress", false);
                        uiex->setEnabled("SubnetMask", false);
                        uiex->setEnabled("GatewayIP", false);
                    }
                    else { // not using dhcp so the field are editable
                        uiex->setEnabled("IPAddress", true);
                        uiex->setEnabled("SubnetMask", true);
                        uiex->setEnabled("GatewayIP", true);
                    }
                    m_RTIRoR.getIpAddress(sDummy);
                    uiex->setPropertyString("IPAddress", "text", sDummy.c_str());
                    m_RTIRoR.getSubnetMask(sDummy);
                    uiex->setPropertyString("SubnetMask", "text", sDummy.c_str());
                    m_RTIRoR.getIPGateway(sDummy);
                    uiex->setPropertyString("GatewayIP", "text", sDummy.c_str());
                }
            }
    }

    else if (!strcmp(pszEvent, "on_pushButton_4_clicked")) {
        m_RTIRoR.restoreRoofMotorSettings();
        // read values from dome controller
        m_RTIRoR.getRoofSpeed(nSpeed);
        uiex->setPropertyInt("shutterSpeed","value", nSpeed);

        m_RTIRoR.getRoofAcceleration(nAcc);
        uiex->setPropertyInt("shutterAcceleration","value", nAcc);
    }

    else if (!strcmp(pszEvent, "on_checkBox_stateChanged"))
    {
        m_bLogRainStatus = uiex->isChecked("checkBox");
        if(m_bLogRainStatus) {
            m_RTIRoR.getRainStatusFileName(fName);
            uiex->setPropertyString("filePath","text", fName.c_str());
        }
        else {
            uiex->setPropertyString("filePath","text", "");
        }
    }

   else if (!strcmp(pszEvent, "on_pushButton_5_clicked")) {
        if(uiex->isChecked("checkBox_2")) {
            m_RTIRoR.setUseDHCP(true);
        }
        else {
            m_RTIRoR.setUseDHCP(false);
            uiex->propertyString("IPAddress", "text", szTmpBuf, SERIAL_BUFFER_SIZE);
            m_RTIRoR.setIpAddress(std::string(szTmpBuf));

            uiex->propertyString("SubnetMask", "text", szTmpBuf, SERIAL_BUFFER_SIZE);
            m_RTIRoR.setSubnetMask(std::string(szTmpBuf));

            uiex->propertyString("GatewayIP", "text", szTmpBuf, SERIAL_BUFFER_SIZE);
            m_RTIRoR.setIPGateway(std::string(szTmpBuf));
        }
        m_RTIRoR.reconfigureNetwork();
        m_bSettingNetwork = true;
        m_SetNetworkTimer.Reset();
    }

    else if (!strcmp(pszEvent, "on_checkBox_2_stateChanged")) {
        if(uiex->isChecked("checkBox_2")) {
            uiex->setEnabled("IPAddress", false);
            uiex->setEnabled("SubnetMask", false);
            uiex->setEnabled("GatewayIP", false);
        }
        else {
            uiex->setEnabled("IPAddress", true);
            uiex->setEnabled("SubnetMask", true);
            uiex->setEnabled("GatewayIP", true);
        }
    }
    else {
        m_RTIRoR.getBatteryLevels( dRoofBattery, dRoofCutOff);
        if(dRoofBattery>=0.0f)
            sTmpBuf << std::fixed << std::setprecision(2) << dRoofBattery << " V";
        else
            sTmpBuf << "--";
        uiex->setPropertyString("shutterBatteryLevel","text", sTmpBuf.str().c_str());
        nErr = m_RTIRoR.getRainSensorStatus(nRainSensorStatus);
        if(nErr)
            uiex->setPropertyString("rainStatus","text", "--");
        else {
            std::stringstream().swap(sTmpBuf);
            sTmpBuf << (nRainSensorStatus==NOT_RAINING ? "<html><head/><body><p><span style=\" color:#00FF00;\">Not raining</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Raining</span></p></body></html>");
            uiex->setPropertyString("rainStatus","text", sTmpBuf.str().c_str());
        }
    }

}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const
{
	str = "RTI-Dome";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const
{
    str = "RTI-Dome";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
    str = "RTI-Dome Dome Controller";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)
{

    if(m_bLinked) {
        std::string sFirmware;
        float fVersion;
		X2MutexLocker ml(GetMutex());
        m_RTIRoR.getFirmwareVersion(sFirmware, fVersion);
        str = sFirmware.c_str();

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    str = "RTI-Dome Dome Controller";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = "RTI-Dome Dome Controller X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return PLUGIN_VERSION;
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
    int nStatus;
    m_RTIRoR.getRoofStatus(nStatus);
    *pdAz = 0;
    if(nStatus == OPEN) {
        *pdEl =90;
    }
    else {
        *pdEl = 0;
    }
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
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

    nErr = m_RTIRoR.openRoof();
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, nErr);

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_RTIRoR.closeRoof();
    if(nErr)
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, nErr);

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    return ERR_COMMANDNOTSUPPORTED;
}

int X2Dome::dapiUnpark(void)
{
    return ERR_COMMANDNOTSUPPORTED;
}

int X2Dome::dapiFindHome(void)
{
    return ERR_COMMANDNOTSUPPORTED;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
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
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, nErr);

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
        return MAKE_ERR_CODE(PLUGIN_ID, DriverRootInterface::DT_DOME, nErr);

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    return ERR_COMMANDNOTSUPPORTED;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    return ERR_COMMANDNOTSUPPORTED;
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

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);

}



