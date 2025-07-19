// Alpaca API functions
//
//  Created by Rodolphe Pineau on 2024/04/16
//  Copyright © 2024 Rodolphe Pineau. All rights reserved.
//

#pragma message "Alpaca server enabled"
#include <atomic>
#include <vector>
#include <functional>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
// Alpaca REST server
#include <UUID.h>
#include <aWOT.h>

#define ALPACA_DISCOVERY_PORT 32227
#define ALPACA_SERVER_PORT 80
#define ALPACA_VAR_BUF_LEN 256
#define ALPACA_OK 0
#define DISCOVERY_ERROR -1

enum AlpacaShutterStates { A_OPEN=0, A_CLOSED, A_OPENING, A_CLOSING,  A_ERROR};
uint32_t nTransactionID;
UUID uuid;
String sAlpacaDiscovery = "alpacadiscovery1";
String sRedirectURL;
volatile bool bAlpacaConnected = false;
class DomeAlpacaDiscoveryServer
{
public:
	DomeAlpacaDiscoveryServer(int port=ALPACA_DISCOVERY_PORT);
	void startServer();
	int checkForRequest();
private:
	EthernetUDP *discoveryServer;
	int m_UDPPort;
};
// ALPACA discovery server
DomeAlpacaDiscoveryServer::DomeAlpacaDiscoveryServer(int port)
{
	m_UDPPort = port;
	discoveryServer = nullptr;
}

void DomeAlpacaDiscoveryServer::startServer()
{
	discoveryServer = new EthernetUDP();
	if(!discoveryServer) {
		discoveryServer = nullptr;
		return;
	}
	discoveryServer->begin(m_UDPPort);
	DBPrintln("Alpaca discovery server started on port " + String(m_UDPPort));
}

int DomeAlpacaDiscoveryServer::checkForRequest()
{
	if(!discoveryServer)
		return -1;
	String sDiscoveryResponse = "{\"AlpacaPort\":"+String(ALPACA_SERVER_PORT)+"}";
	String sDiscoveryRequest;
	char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
	int packetSize = discoveryServer->parsePacket();
	if (packetSize) {
		DBPrintln("Alpaca discovery server request");
		memset(packetBuffer,0,sizeof(packetBuffer));
		discoveryServer->read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		// do stuff
		sDiscoveryRequest = String(packetBuffer);
		DBPrintln("Alpaca discovery server sDiscoveryRequest : " + sDiscoveryRequest);
		if(sDiscoveryRequest.indexOf(sAlpacaDiscovery)==-1) {
			DBPrintln("Alpaca discovery server request error");
			return DISCOVERY_ERROR; // wrong type of discovery message
		}
		DBPrintln("Alpaca discovery server sending response : " + sDiscoveryResponse);
		// send discovery reponse
		discoveryServer->beginPacket(discoveryServer->remoteIP(), discoveryServer->remotePort());
		discoveryServer->write(sDiscoveryResponse.c_str());
		discoveryServer->endPacket();
	}
	return ALPACA_OK;
}


void formDataToJson(Request &req, JsonDocument &FormData)
{
	char name[ALPACA_VAR_BUF_LEN];
	char value[ALPACA_VAR_BUF_LEN];
	String sName;
	String sValue;
	memset(name,0,ALPACA_VAR_BUF_LEN);
	memset(value,0,ALPACA_VAR_BUF_LEN);
	while(req.form(name, ALPACA_VAR_BUF_LEN-1, value, ALPACA_VAR_BUF_LEN-1)){
		sName  = String(name);
		sName.toLowerCase();
		sValue = String(value);
		sValue.toLowerCase();
		DBPrintln("name : " + sName);
		DBPrintln("value : " + sValue);
		if(isDigit(value[0]) ) {
			if(sValue.indexOf('.') == -1) {
				// int
				FormData[sName]=sValue.toInt();
			} else {
				// double
				FormData[sName]=sValue.toDouble();
			}
		}
		else {
			// string
			// check for boolean
			if(sValue == "true") {
				FormData[sName]=true;
			}
			else if(sValue == "false") {
				FormData[sName]=false;
			}
			else {
				FormData[sName]=sValue;
			}
		}
	}
}


void  getQueryGetVariables(String sQueryString, std::vector<std::vector<String>> &svParameters)
{
	int nErr;
	int nIndex = 0;
	int nCurIndex = 0;
	String sEntry;
	std::vector<String> svKV;
	std::vector<String> svFields;
	DBPrintln("getQueryGetVariables");
	// url parameters are separate by '&'
	while(true) {
		nIndex = sQueryString.indexOf('&',nCurIndex);
		if(nIndex == -1) {
			svFields.push_back(sQueryString.substring(nCurIndex));
			break;
		}
		svFields.push_back(sQueryString.substring(nCurIndex,nIndex));
		nCurIndex = nIndex+1;
	}
	if(svFields.size()) {
		// now split each field in key,value pair with '=' as the separator
		for(String &sTmp : svFields) {
			sTmp.toLowerCase();
			nIndex = sTmp.indexOf('=');
			svKV.push_back(sTmp.substring(0,nIndex));
			svKV.push_back(sTmp.substring(nIndex+1));
			svParameters.push_back(svKV);
			svKV.clear();
		}
	}
	return;
}

bool getIDs(Request &req, JsonDocument &AlpacaResp, JsonDocument &FormData)
{
	char ClientID[64];
	char ClientTransactionID[64];
	String sClientId;
	String sClientTransactionId;
	std::vector<std::vector<String>> svParameters;
	bool bParamOk = true;
	DBPrintln("getIDs");
	AlpacaResp["ServerTransactionID"] = nTransactionID;
	if(req.method() == Request::GET) {
		// the req.query being case sensitive will not work here.
		getQueryGetVariables(String(req.query()), svParameters);
		for( std::vector<String> &svParamEntry : svParameters ) {

			if(svParamEntry.at(0).equals("clientid"))
				sClientId = svParamEntry.at(1);
			if(svParamEntry.at(0).equals("clienttransactionid"))
				sClientTransactionId = svParamEntry.at(1);
		}

		if(sClientId.length())
			AlpacaResp["ClientID"] = sClientId.toInt()<0?0:sClientId.toInt();
		if(sClientTransactionId.length())
			AlpacaResp["ClientTransactionID"] = sClientTransactionId.toInt()<0?0:sClientTransactionId.toInt();
	}
	else { // this is a PUT, therefore there should be some form data
		formDataToJson(req, FormData);
		if(FormData.size()==0){
			bParamOk = false;
		}
		else {
			if(FormData["clientid"].is<unsigned long>()) {
				serializeJson(FormData["clientid"], sClientId);
				sClientId.trim();
				AlpacaResp["ClientID"] = sClientId.toInt()<0?0:sClientId.toInt();
			}
			if(FormData["clienttransactionid"].is<unsigned long>()) {
				serializeJson(FormData["clienttransactionid"], sClientTransactionId);
				sClientTransactionId.trim();
				AlpacaResp["ClientTransactionID"] = sClientTransactionId.toInt()<0?0:sClientTransactionId.toInt();
			}
		}
#ifdef DEBUG
		String sTmp;
		serializeJson(FormData, sTmp);
		DBPrintln("FormData : " + sTmp);
		DBPrintln("FormData.size() : " + String(FormData.size()));
#endif
	}

	DBPrintln("bParamOk : " + String(bParamOk?"Ok":"Error"));
	DBPrintln("sClientId : " + sClientId);
	DBPrintln("sClientTransactionId : " + sClientTransactionId);
	return bParamOk;
}

void redirectToSetup(Request &req, Response &res)
{
	res.set("Location", sRedirectURL.c_str());
    res.sendStatus(302);
}

void getApiVersion(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getApiVersion ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["Value"][0] = 1;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getDescription(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getDescription ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["Value"]["ServerName"]= "RTIDome Alpaca";
	AlpacaResp["Value"]["Manufacturer"]= "RTI-Zone";
	AlpacaResp["Value"]["ManufacturerVersion"]= VERSION;
	AlpacaResp["Value"]["Location"]= "Earth";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getConfiguredDevice(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getConfiguredDevice ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["Value"][0] ["DeviceName"]= "RTIDome";
	AlpacaResp["Value"][0] ["DeviceType"]= "dome";
	AlpacaResp["Value"][0] ["DeviceNumber"]= 0;
	AlpacaResp["Value"][0] ["UniqueID"]= uuid;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doAction(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sAction;
	String sParameters;
	DBPrintln("[ ********** doAction ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	serializeJson(FormData["action"], sAction);
	serializeJson(FormData["parameters"], sParameters);
#ifdef DEBUG
	DBPrintln("sAction : " + sAction);
	DBPrintln("sParameters : " + sParameters);
#endif

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = "Ok";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doCommandBlind(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sClientId;
	String sClientTransactionId;
	DBPrintln("[ ********** doCommandBlind ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doCommandBool(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sClientId;
	String sClientTransactionId;
	DBPrintln("[ ********** doCommandBool ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doCommandString(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sClientId;
	String sClientTransactionId;
	DBPrintln("[ ********** doCommandString ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = "Ok";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getConnected(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getConected ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = bAlpacaConnected;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void setConnected(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sClientId;
	String sClientTransactionId;
	String sParameter;
	String sTmp;
	DBPrintln("[ ********** setConected ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(!FormData["connected"].is<bool>()) {
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters, missing 'Connected'";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	bAlpacaConnected = FormData["connected"];
	DBPrintln("bAlpacaConnected : " + (bAlpacaConnected?String("true"):String("false")));
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getDeviceDescription(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getDeviceDescription ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"]= "RTI-Zone dome controller";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getDriverInfo(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getDriverInfo ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"]= "RTI-Zone Dome controller";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getDriverVersion(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getDriverVersion ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"]= String(VERSION);
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getInterfaceVersion(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getInterfaceVersion ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"]= 1;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getName(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getName ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"]= "RTI-Zone Dome controller";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getSupportedActions(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getSupportedActions ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["Value"].add("EthernetReconfigure");
	AlpacaResp["Value"].add("Calibrate");
	AlpacaResp["Value"].add("RestoreMotorDefault");
	AlpacaResp["Value"].add("GetRoofAcceleration");
	AlpacaResp["Value"].add("SetRoofAcceleration");
	AlpacaResp["Value"].add("GetMacAddress");
	AlpacaResp["Value"].add("GetIpAddress");
	AlpacaResp["Value"].add("SetIpAddress");
	AlpacaResp["Value"].add("RoofVolts");
	AlpacaResp["Value"].add("GetRainAction");
	AlpacaResp["Value"].add("SetRainAction");
	AlpacaResp["Value"].add("isShutterPresent");
	AlpacaResp["Value"].add("GetSubnet");
	AlpacaResp["Value"].add("SetSubnet");
	AlpacaResp["Value"].add("GetPanID");
	AlpacaResp["Value"].add("SetPanID");
	AlpacaResp["Value"].add("GetRoofSpeed");
	AlpacaResp["Value"].add("SetRoofSpeed");
	AlpacaResp["Value"].add("GetStepPerRev");
	AlpacaResp["Value"].add("SetStepPerRev");
	AlpacaResp["Value"].add("GetIpGateway");
	AlpacaResp["Value"].add("SetIpGateway");
	AlpacaResp["Value"].add("GetDhcp");
	AlpacaResp["Value"].add("SetDhcp");
	AlpacaResp["Value"].add("GetRoofReverse");
	AlpacaResp["Value"].add("SetRoofReverse");
	AlpacaResp["Value"].add("GetRainStatus");
	AlpacaResp["Value"].add("RestoreMotorDefaultShutter");
	AlpacaResp["Value"].add("GetShutterAcceleration");
	AlpacaResp["Value"].add("SetShutterAcceleration");
	AlpacaResp["Value"].add("ShutterHello");
	AlpacaResp["Value"].add("GetShutterPanID");
	AlpacaResp["Value"].add("SetShutterPanID");
	AlpacaResp["Value"].add("GetShutterSpeed");
	AlpacaResp["Value"].add("SetShutterSpeed");
	AlpacaResp["Value"].add("GetShutterReverse");
	AlpacaResp["Value"].add("SetShutterReverse");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getAltitude(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sTmpString = String(STATE_ROOF);
	DBPrintln("[ ********** getAltitude ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = 0.0;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void geAtHome(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** geAtHome ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	/*if(String(Roof->GetHomeStatus() == CLOSED)) {
		AlpacaResp["Value"] = true;
	}
	else {
		AlpacaResp["Value"] = false;
	}
	*/
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void geAtPark(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** geAtPark ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(bParked) {
		AlpacaResp["Value"] = true;
	}
	else {
		AlpacaResp["Value"] = false;
	}
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getAzimuth(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getAzimuth ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	// AlpacaResp["Value"] = Roof->GetAzimuth();
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canfindhome(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canfindhome ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canPark(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canPark ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSetAltitude(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSetAltitude ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = false;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSetAzimuth(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSetAzimuth ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSetPark(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSetPark ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSetShutter(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSetShutter ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSlave(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSlave ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = false;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void canSyncAzimuth(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSyncAzimuth ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = true;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getShutterStatus(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sTmpString = String(STATE_ROOF);
	DBPrintln("[ ********** getShutterStatus ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
/*	
	shutterClient.print(sTmpString + "#");
	switch (RemoteShutter.state) {
		case OPEN:
			AlpacaResp["Value"] = A_OPEN;
			break;
		case CLOSED:
			AlpacaResp["Value"] = A_CLOSED;
			break;
		case ERROR:
			AlpacaResp["Value"] = A_ERROR;
			break;
		case OPENING:
		case BOTTOM_OPEN:
		case BOTTOM_OPENING:
		case FINISHING_OPEN:
			AlpacaResp["Value"] = A_OPENING;
			break;
		case CLOSING:
		case BOTTOM_CLOSED:
		case BOTTOM_CLOSING:
		case FINISHING_CLOSE:
			AlpacaResp["Value"] = A_CLOSING;
			break;
		default:
			AlpacaResp["Value"] = A_ERROR;
			break;
	}
	*/
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}
void getSlaved(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** canSlave ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	AlpacaResp["Value"] = false;
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void setSlaved(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** Slaved ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0x400;
	AlpacaResp["ErrorMessage"] = "Invalid parameters, missing 'Connected'";
	AlpacaResp["Value"] = false;
	serializeJson(AlpacaResp, sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void getSlewing(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** getSlewing ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	DBPrintln("Seekmode : " + String(Roof->GetSeekMode()));
	if(Roof->GetSeekMode() != NOT_MOVING) {
		AlpacaResp["Value"] = true;
	}
	else {
		AlpacaResp["Value"] = false;
	}

	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doAbort(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** doAbort ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	Abort(); // this is in the RoREth-esp32.ino

	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doCloseShutter(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sTmpString = String(CLOSE_ROOF);
	DBPrintln("[ ********** doCloseShutter ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	// shutterClient.print(sTmpString+ "#");
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doFindHome(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** doFindHome ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(bLowShutterVoltage) {
		AlpacaResp["ErrorNumber"] = 0x408;
		AlpacaResp["ErrorMessage"] = "Low shutter voltage, staying at park position";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	// Roof->StartHoming();
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doOpenShutter(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sTmpString = String(OPEN_ROOF);
	DBPrintln("[ ********** doOpenShutter ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(bLowShutterVoltage) {
		AlpacaResp["ErrorNumber"] = 0x408;
		AlpacaResp["ErrorMessage"] = "Low shutter voltage, staying at closed position";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

	}
	else {
		AlpacaResp["ErrorNumber"] = 0;
		AlpacaResp["ErrorMessage"] = "";
		// shutterClient.print(sTmpString+ "#");
	}
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doPark(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	double fParkPos;
	DBPrintln("[ ********** doPark ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	// fParkPos = Roof->GetParkAzimuth();
	// Roof->GoToAzimuth(fParkPos);
	bParked = true;
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void setPark(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	double fParkPos;
	DBPrintln("[ ********** setPark ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}


	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
	// fParkPos = Roof->GetAzimuth();
	// Roof->SetParkAzimuth(fParkPos);
}

void doAltitudeSlew(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	DBPrintln("[ ********** doAltitudeSlew ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(!FormData["altitude"].is<double>()) {
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid value";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}
	// in case we implement this one day.
	AlpacaResp["ErrorNumber"] = 0x400;
	AlpacaResp["ErrorMessage"] = "Not implemented";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doGoTo(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	double dNewPos;
	DBPrintln("[ ********** doGoTo ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(bLowShutterVoltage) {
		AlpacaResp["ErrorNumber"] = 0x408;
		AlpacaResp["ErrorMessage"] = "Low shutter voltage, staying at park position";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(!FormData["azimuth"].is<double>()) {
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	dNewPos = FormData["azimuth"];
	if(dNewPos < 0 || dNewPos>360) {
		AlpacaResp["ErrorNumber"] = 1025;
		AlpacaResp["ErrorMessage"] = "Invalid azimuth";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	// Roof->GoToAzimuth(dNewPos);
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}

void doSyncAzimuth(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	double dNewPos;
	DBPrintln("[ ********** doSyncAzimuth ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	res.set("Content-Type", "application/json");
	if(!bParamsOk){
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid parameters";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	if(!FormData["azimuth"].is<double>()) {
		AlpacaResp["ErrorNumber"] = 1025;
		AlpacaResp["ErrorMessage"] = "Invalid azimuth";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	dNewPos = FormData["azimuth"];
	if(dNewPos<0 || dNewPos > 360) {
		AlpacaResp["ErrorNumber"] = 0x401;
		AlpacaResp["ErrorMessage"] = "Invalid Azimuth";
		serializeJson(AlpacaResp, sResp);
		res.write((uint8_t*)(sResp.c_str()),sResp.length());

		return;
	}

	// Roof->SyncPosition(dNewPos);
	AlpacaResp["ErrorNumber"] = 0;
	AlpacaResp["ErrorMessage"] = "";
	serializeJson(AlpacaResp, sResp);
	DBPrintln("sResp : " + sResp);
	res.write((uint8_t*)(sResp.c_str()),sResp.length());
}


void doSetup(Request &req, Response &res)
{
	JsonDocument AlpacaResp;
	JsonDocument FormData;
	bool bParamsOk = false;
	String sResp;
	String sHTML;
	res.set("Content-Type", "text/html");
	DBPrintln("[ ********** doSetup ********** ]");
	bParamsOk = getIDs(req, AlpacaResp, FormData);
	sHTML = "<!DOCTYPE html>\n<html>\n";
	sHTML += "<head>";
	sHTML += "<title>RTI Dome Setup</title>\n";
	sHTML += "</head>\n";
	sHTML += "<body>\n";
	sHTML += "<H1>RTI Dome Setup</H1>\n";
	// display passed data
	if(FormData.size()!=0){
		sHTML += "<p>data passed : </p>\n";
		sHTML += "<p>"+sResp+"</p>\n";
	}

	sHTML += "</body>\n</html>\n";
	res.print(sHTML);
}


class DomeAlpacaServer
{
public :
	DomeAlpacaServer(int port=ALPACA_SERVER_PORT);
	void startServer();
	void checkForRequest();
	void setRoofPtr(RoofClass *pRoof);

	static void myCallback(Request &req, Response &res);

private :
	EthernetServer *mRestServer;
	Application  *m_AlpacaRestServer;
	int m_nRestPort;
};

DomeAlpacaServer::DomeAlpacaServer(int port)
{
	m_nRestPort = port;
	mRestServer = nullptr;
	m_AlpacaRestServer = nullptr;
	nTransactionID = 0;
}

void DomeAlpacaServer::startServer()
{
	mRestServer = new EthernetServer(m_nRestPort);
	m_AlpacaRestServer = new Application();
	DBPrintln("m_AlpacaRestServer starting");
	DBPrintln("m_AlpacaRestServer UUID : " + String(uuid.toCharArray()));
	mRestServer->begin();
	sRedirectURL = String("http://")+ sLocalIPAdress + String(":") + String(ALPACA_SERVER_PORT) + String("/setup/v1/dome/0/setup");
	DBPrintln("Redirect URL for setup : " + sRedirectURL);
	DBPrintln("m_AlpacaRestServer mapping endpoints");
	m_AlpacaRestServer->use("/", &redirectToSetup);
	m_AlpacaRestServer->use("/setup", &redirectToSetup);
	m_AlpacaRestServer->get("/management/apiversions", &getApiVersion);
	m_AlpacaRestServer->get("/management/v1/configureddevices", &getConfiguredDevice);
	m_AlpacaRestServer->get("/management/v1/description", &getDescription);
	m_AlpacaRestServer->use("/setup/v1/dome/0/setup", &doSetup);
	m_AlpacaRestServer->put("/api/v1/dome/0/action", &doAction);
	m_AlpacaRestServer->put("/api/v1/dome/0/commandblind", &doCommandBlind);
	m_AlpacaRestServer->put("/api/v1/dome/0/commandbool", &doCommandBool);
	m_AlpacaRestServer->put("/api/v1/dome/0/commandstring", &doCommandString);
	m_AlpacaRestServer->get("/api/v1/dome/0/connected", &getConnected);
	m_AlpacaRestServer->put("/api/v1/dome/0/connected", &setConnected);
	m_AlpacaRestServer->get("/api/v1/dome/0/description", &getDeviceDescription);
	m_AlpacaRestServer->get("/api/v1/dome/0/driverinfo", &getDriverInfo);
	m_AlpacaRestServer->get("/api/v1/dome/0/driverversion", &getDriverVersion);
	m_AlpacaRestServer->get("/api/v1/dome/0/interfaceversion", &getInterfaceVersion);
	m_AlpacaRestServer->get("/api/v1/dome/0/name", &getName);
	m_AlpacaRestServer->get("/api/v1/dome/0/supportedactions", &getSupportedActions);
	m_AlpacaRestServer->get("/api/v1/dome/0/altitude", &getAltitude);
	m_AlpacaRestServer->get("/api/v1/dome/0/athome", &geAtHome);
	m_AlpacaRestServer->get("/api/v1/dome/0/atpark", &geAtPark);
	m_AlpacaRestServer->get("/api/v1/dome/0/azimuth", &getAzimuth);
	m_AlpacaRestServer->get("/api/v1/dome/0/canfindhome", &canfindhome);
	m_AlpacaRestServer->get("/api/v1/dome/0/canpark", &canPark);
	m_AlpacaRestServer->get("/api/v1/dome/0/cansetaltitude", &canSetAltitude);
	m_AlpacaRestServer->get("/api/v1/dome/0/cansetazimuth", &canSetAzimuth);
	m_AlpacaRestServer->get("/api/v1/dome/0/cansetpark", &canSetPark);
	m_AlpacaRestServer->get("/api/v1/dome/0/cansetshutter", &canSetShutter);
	m_AlpacaRestServer->get("/api/v1/dome/0/canslave", &canSlave);
	m_AlpacaRestServer->get("/api/v1/dome/0/cansyncazimuth", &canSyncAzimuth);
	m_AlpacaRestServer->get("/api/v1/dome/0/shutterstatus", &getShutterStatus);
	m_AlpacaRestServer->get("/api/v1/dome/0/slaved", &getSlaved);
	m_AlpacaRestServer->put("/api/v1/dome/0/slaved", &setSlaved);
	m_AlpacaRestServer->get("/api/v1/dome/0/slewing", &getSlewing);
	m_AlpacaRestServer->put("/api/v1/dome/0/abortslew", &doAbort);
	m_AlpacaRestServer->put("/api/v1/dome/0/closeshutter", &doCloseShutter);
	m_AlpacaRestServer->put("/api/v1/dome/0/findhome", &doFindHome);
	m_AlpacaRestServer->put("/api/v1/dome/0/openshutter", &doOpenShutter);
	m_AlpacaRestServer->put("/api/v1/dome/0/park", &doPark);
	m_AlpacaRestServer->put("/api/v1/dome/0/setpark", &setPark);
	m_AlpacaRestServer->put("/api/v1/dome/0/slewtoaltitude", &doAltitudeSlew);
	m_AlpacaRestServer->put("/api/v1/dome/0/slewtoazimuth", &doGoTo);
	m_AlpacaRestServer->put("/api/v1/dome/0/synctoazimuth", &doSyncAzimuth);

	DBPrintln("m_AlpacaRestServer started");
}


void DomeAlpacaServer::checkForRequest()
{
	// process incoming connections one at a time
	EthernetClient client = mRestServer->accept();
	if (client.connected()) {
		m_AlpacaRestServer->process(&client);
		client.stop();
		nTransactionID++;
  }
}
