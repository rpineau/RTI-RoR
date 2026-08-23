//
// RTI-Zone RoR firmware.
// for ESP32
//
// Copyright © 2025 Rodolphe Pineau. All rights reserved.
//


// Uncomment #define DEBUG to enable printing debug messages on serial port defined as DebugPort

#include "Arduino.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "config.h"

bool firstLoop = true;
#include "RoofClass.h"

#pragma message "Ethernet enabled"
// include and some defines for ethernet connection
#include <Network.h>

#ifdef USE_OTA_UPDATE
#pragma message "OTA Update enable"
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#endif

byte MAC_Address[6];

IPConfig ServerConfig;
volatile bool ethernetPresent;
NetworkServer *RoR_Server = nullptr;
NetworkClient domeClient;
int nbNetworkClient = 0;
String networkBuffer = "";
String sLocalIPAdress = "";
// OTA update stuff
#ifdef USE_OTA_UPDATE
WebServer httpServer(OTA_PORT);
HTTPUpdateServer httpUpdater;
#endif

String computerBuffer = "";

bool bParked = false; // use to the run check doesn't continuously try to park

RoofClass *Roof = NULL;

// global variable for conditon status
volatile bool bIsSafe;

const char ERR_NO_DATA = -1;

#include "ror_commands.h"
enum CmdSource {SERIAL_CMD, NETWORK_CMD};
// function prototypes
void configureEthernet();
bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnetMask, bool bReconfigure);
void checkForNewTCPClient();

void openIntHandler();
void closeIntHandler();
void conditionIntHandler();
void buttonOpenHandler();
void buttonCloseHandler();
void resetChip(int);
void CheckForCommands();
void CheckForCondition();
void ReceiveNetwork(NetworkClient client);
void ReceiveComputer();
void ProcessCommand(int nSource);
void Abort();

#ifdef USE_ALPACA
#include "AlpacaAPI.h"
DomeAlpacaServer *AlpacaServer;
RoRAlpacaDiscoveryServer *AlpacaDiscoveryServer;
#endif

void MotorTask(void *);
esp_task_wdt_config_t twdt_config = {
	.timeout_ms = 1000000,
	.idle_core_mask = 0,    // Bitmask of cores
	.trigger_panic = false,
};

//
// Setup and main loops
//
void setup()
{
	ethernetPresent = false;
	bIsSafe = false;
	nbNetworkClient = 0;

#ifdef DEBUG
#ifndef DEBUG_TO_COMPUTER
	DebugPort.begin(115200, SERIAL_8N1, 16, 17); // pins 16 rx2, 17 tx2, 115200 bps, 8 bits no parity 1 stop bit
	//DebugPort.begin(115200);
	delay(1000);
	DBPrintln("========== RTI-Zone controller booting ==========");
#endif
#endif

	digitalWrite(ETHERNET_RESET, 0);
	pinMode(ETHERNET_RESET, OUTPUT);
	Computer.begin(115200);
	//Computer.begin(115200, SERIAL_8N1, 16, 17); // pins 16 rx2, 17 tx2, 115200 bps, 8 bits no parity 1 stop bit


	Roof = new RoofClass();
	Roof->motorStop();

	configureEthernet();
	esp_task_wdt_deinit();
	esp_task_wdt_init(&twdt_config);
	esp_task_wdt_add(NULL);
	disableCore0WDT();
	disableCore1WDT();
	xTaskCreatePinnedToCore(MotorTask, "MotorTask", 10000, NULL, 16, NULL,  0);

	RoR_Server = new NetworkServer(CMD_SERVER_PORT);
	RoR_Server->begin();
#ifdef USE_ALPACA
	AlpacaDiscoveryServer = new RoRAlpacaDiscoveryServer();
	AlpacaDiscoveryServer->startServer();
	AlpacaServer = new DomeAlpacaServer();
	AlpacaServer->startServer();
#endif
	DBPrintln("========== Ready ==========");
}

//
// These tasks take care of all communications and commands
//


void loop()
{
	if(firstLoop) {
		firstLoop = false;
		Computer.println("========== Rotator is Ready ==========");
	}

	if(ethernetPresent) {
		checkForNewTCPClient();
		AlpacaDiscoveryServer->checkForRequest();
		AlpacaServer->checkForRequest();
	}

	CheckForCommands();
	CheckForCondition();

#ifdef USE_OTA_UPDATE
	httpServer.handleClient();
#endif

	taskYIELD();
	esp_task_wdt_reset();
}

//
// This task does all the motor controls
//
void MotorTask(void *)
{
	DBPrintln("========== Motor task starting ==========");
	DBPrintln("========== Motor task Attaching interrupt handler ==========");
	attachInterrupt(digitalPinToInterrupt(CLOSE_PIN), closeIntHandler, FALLING);
	attachInterrupt(digitalPinToInterrupt(OPEN_PIN), openIntHandler, FALLING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_CLOSE), buttonCloseHandler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON_OPEN), buttonOpenHandler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(COND_SENSOR_PIN), conditionIntHandler, CHANGE);

	esp_task_wdt_add(NULL);
	DBPrintln("========== Motor task ready ==========");
	for(;;) {
		Roof->Run();
		taskYIELD();
		esp_task_wdt_reset();
	}
}


//
//
//
void configureEthernet()
{
        DBPrintln("========== Configuring Ethernet ==========");
        Roof->getIpConfig(ServerConfig);
        ethernetPresent =  initEthernet(ServerConfig.bUseDHCP,
										ServerConfig.ip,
										ServerConfig.dns,
										ServerConfig.gateway,
										ServerConfig.subnetMask);
}

#ifdef DEBUG
void onEvent(arduino_event_id_t event, arduino_event_info_t info)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      DBPrintln("ETH Started");
      //set eth hostname here
      DBPrintln("esp32-eth0");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      DBPrintln("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      DBPrintln("ETH Got IP: '" + String(esp_netif_get_desc(info.got_ip.esp_netif)) +"'");
      DBPrintln(ETH);
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      DBPrintln("ETH Lost IP");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      DBPrintln("ETH Disconnected");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      DBPrintln("ETH Stopped");
      break;
    default:
      break;
  }
}
#endif
bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnetMask)
{
	bool bDhcpOk;
	int nTimeout = 0;
#ifdef DEBUG
	Network.onEvent(onEvent); // this is just for debugging
#endif
	DBPrintln("========== Init Ethernet ==========");
	// resetChip(ETHERNET_RESET);
	SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI);
	// network configuration
	if(!ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI)) {
		DBPrintln("NO HARDWARE !!!");
		return false;
	}
	nbNetworkClient = 0;
	// set an ip so we can get the link status
	RoR_Ethernet.config(ip, gateway, subnetMask);
	while(!RoR_Ethernet.linkUp() ) {
		vTaskDelay(250 / portTICK_PERIOD_MS);
		nTimeout++;
		if(nTimeout == 120) { // 30 seconds timeout, 250ms per loop, 120 loops = 30 seconds
			return false;
		}
	}

	RoR_Ethernet.macAddress(MAC_Address);
	RoR_Ethernet.setHostname("RTI-RoR");

	DBPrintln("========== Setting IP config ==========");
	// try DHCP if set
	if(bUseDHCP) {
		bDhcpOk = RoR_Ethernet.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0)); // all value set to the default 0 means use dhcp.
		if(bDhcpOk) {
			nTimeout = 0;
			while(RoR_Ethernet.localIP() == IPAddress(0,0,0,0) ) {
				vTaskDelay(250 / portTICK_PERIOD_MS);
				nTimeout++;
				if(nTimeout == 120) { // 30 seconds timeout, 250ms per loop, 120 loops = 30 seconds
					break;
				}
			}
		}
	}
	else {
		RoR_Ethernet.config(ip, gateway, subnetMask);
		RoR_Ethernet.dnsIP(0,dns);
	}

	if(RoR_Ethernet.localIP() == IPAddress(0,0,0,0)) {
			RoR_Ethernet.config(ip, gateway, subnetMask); // use defaults
			vTaskDelay(250 / portTICK_PERIOD_MS);
	}

	RoR_Ethernet.setDefault();

	DBPrintln("========== Checking hardware status ==========");
	DBPrintln("W5500 Ok.");
	DBPrintln("W5500 IP = " + RoofClass::IpAddress2String(RoR_Ethernet.localIP()));
#ifdef DEBUG
	char macBuffer[20];
	snprintf(macBuffer,20,"%02x:%02x:%02x:%02x:%02x:%02x",
		MAC_Address[0],
		MAC_Address[1],
		MAC_Address[2],
		MAC_Address[3],
		MAC_Address[4],
		MAC_Address[5]);
	DBPrintln("Dome MAC : " + String(macBuffer));
#endif

	DBPrintln("Server ready");

	sLocalIPAdress = RoofClass::IpAddress2String(RoR_Ethernet.localIP());
	return true;
}


void checkForNewTCPClient()
{
	if(!RoR_Server)
		return;

	NetworkClient newClient = RoR_Server->accept();
	if(newClient) {
		DBPrintln("new client");
		if(nbNetworkClient > 0) { // we only accept 1 client
			newClient.write("Already in use#");
			newClient.flush();
			newClient.stop();
			DBPrintln("new client rejected");
		}
		else {
			nbNetworkClient++;
			domeClient = newClient;
			DBPrintln("new client accepted");
			DBPrintln("nb client = " + String(nbEthernetClient));
		}
	}

	if((nbNetworkClient>0) && !domeClient.connected()) {
		DBPrintln("client disconnected");
		domeClient.stop();
		nbNetworkClient--;
		DBPrintln("nb client = " + String(nbNetworkClient));
	}
}

void IRAM_ATTR openIntHandler()
{
   if(Roof)
	   Roof->openInterrupt();
}

void IRAM_ATTR closeIntHandler()
{
   if(Roof)
	   Roof->closedInterrupt();
}

void IRAM_ATTR conditionIntHandler()
{
   if(Roof)
	   Roof->conditionInterrupt();
}

void IRAM_ATTR buttonOpenHandler()
{
   if(Roof)
	   Roof->ButtonOpenCheck();
}

void IRAM_ATTR buttonCloseHandler()
{
   if(Roof)
	   Roof->ButtonCloseCheck();
}


// reset chip with /reset connected to nPin
void resetChip(int nPin)
{
	digitalWrite(nPin, 0);
	delay(2);
	digitalWrite(nPin, 1);
	delay(10);
}

void CheckForCommands()
{
	ReceiveComputer();
	if(ethernetPresent ) {
		ReceiveNetwork(domeClient);
	}
}

void CheckForCondition()
{
	String shutterMessage;

	int nPosition, nParkPos;
	if(bIsSafe != Roof->GetConditionStatus()) { // was there a state change ?
		bIsSafe = Roof->GetConditionStatus();
	}
	if(!bIsSafe) {
		// emergency close
		// need to make sure mount is parked.
		// might need to leave this to the app control for now.
	}
}


void ReceiveNetwork(NetworkClient client)
{
	char networkCharacter;

	if(!client.connected()) {
		return;
	}

	if(client.available() < 1)
		return; // no data

	while(client.available()>0) {
		networkCharacter = client.read();
		if (networkCharacter != ERR_NO_DATA) {
			if (networkCharacter == '\r' || networkCharacter == '\n' || networkCharacter == '#') {
				// End of message
				if (networkBuffer.length() > 0) {
					ProcessCommand(NETWORK_CMD);
					networkBuffer = "";
					return; // we'll read the next command on the next loop.
				}
			}
			else {
				networkBuffer += String(networkCharacter);
			}
		}
	}
}

// All comms are terminated with '#' but the '\r' and '\n' are for debugging
void ReceiveComputer()
{
	char computerCharacter;

	if(!Computer)
		return;

	if(Computer.available() < 1)
		return; // no data

	while(Computer.available() > 0 ) {
		computerCharacter = Computer.read();
		if (computerCharacter != ERR_NO_DATA) {
			if (computerCharacter == '\r' || computerCharacter == '\n' || computerCharacter == '#') {
				// End of message
				if (computerBuffer.length() > 0) {
					ProcessCommand(SERIAL_CMD);
					computerBuffer = "";
					return; // we'll read the next command on the next loop.
				}
			}
			else {
				computerBuffer += String(computerCharacter);
			}
		}
	}
}

void ProcessCommand(int nSource)
{
	double fTmp;
	char command;
	String value;

	String serialMessage, sTmpString;
	bool hasValue = false;

	// Split the buffer into command char and value if present
	// Command character
	switch(nSource) {
		case SERIAL_CMD:
			command = computerBuffer.charAt(0);
			// Payload
			value = computerBuffer.substring(1);
			break;
		case NETWORK_CMD:
			command = networkBuffer.charAt(0);
			// Payload
			value = networkBuffer.substring(1);
			break;
	}

	// payload has data
	if (value.length() > 0)
		hasValue = true;

	serialMessage = "";

	DBPrintln("\nProcessCommand");
	DBPrintln("Command = \"" + String(command) +"\"");
	DBPrintln("Value = \"" + String(value) +"\"");
	DBPrintln("nSource = " + String(nSource));


	switch (command) {
		case ABORT:
			sTmpString = String(ABORT);
			serialMessage = sTmpString;
			Abort();
			break;

		case CALIBRATE_ROOF:
			Roof->StartCalibrating();
			serialMessage = String(CALIBRATE_ROOF);
			break;


		case COND_ROOF:
			serialMessage = String(COND_ROOF) + String(bIsSafe ? "1" : "0");
			break;

		case ETH_RECONFIG :
			if(nbNetworkClient > 0) {
				domeClient.stop();
				nbNetworkClient--;
			}
			DBPrintln("Rebooting for Ethernet reconfiguration");
			vTaskDelay(500 / portTICK_PERIOD_MS);
			ESP.restart();
			break;

		case ETH_MAC_ADDRESS:
			char macBuffer[20];
			snprintf(macBuffer,20,"%02x:%02x:%02x:%02x:%02x:%02x",
					MAC_Address[0],
					MAC_Address[1],
					MAC_Address[2],
					MAC_Address[3],
					MAC_Address[4],
					MAC_Address[5]);

			serialMessage = String(ETH_MAC_ADDRESS) + String(macBuffer);
			break;

		case IP_DHCP:
			if (hasValue) {
				Roof->setDHCPFlag(value.toInt() == 0 ? false : true);
			}
			serialMessage = String(IP_DHCP) + String( Roof->getDHCPFlag()? "1" : "0");
			break;

		case IP_ADDRESS:
			if (hasValue) {
				Roof->setIPAddress(value);
				Roof->getIpConfig(ServerConfig);
			}
			if(!ServerConfig.bUseDHCP)
				serialMessage = String(IP_ADDRESS) + String(Roof->getIPAddress());
			else {
				serialMessage = String(IP_ADDRESS) + String(RoofClass::IpAddress2String(RoR_Ethernet.localIP()));
			}
			break;

		case IP_SUBNET:
			if (hasValue) {
				Roof->setIPSubnetMask(value);
				Roof->getIpConfig(ServerConfig);
			}
			if(!ServerConfig.bUseDHCP)
				serialMessage = String(IP_SUBNET) + String(Roof->getIPSubnetMask());
			else {
				serialMessage = String(IP_SUBNET) + String(RoofClass::IpAddress2String(RoR_Ethernet.subnetMask()));
			}
			break;

		case IP_GATEWAY:
			if (hasValue) {
				Roof->setIPGateway(value);
				Roof->getIpConfig(ServerConfig);
			}
			if(!ServerConfig.bUseDHCP)
				serialMessage = String(IP_GATEWAY) + String(Roof->getIPGateway());
			else {
				serialMessage = String(IP_GATEWAY) + String(RoofClass::IpAddress2String(RoR_Ethernet.gatewayIP()));
			}
			break;

		case ACCELERATION_ROOF:
			if (hasValue) {
				Roof->SetAcceleration(value.toInt());
			}
			serialMessage = String(ACCELERATION_ROOF) + String(Roof->GetAcceleration());
			break;

		case CLOSE_ROOF:
			sTmpString = String(CLOSE_ROOF);
			serialMessage = sTmpString;
			Roof->Close();
			break;

		case ROOF_RESTORE_MOTOR_DEFAULT :
			Roof->restoreDefaultMotorSettings();
			serialMessage = String(ROOF_RESTORE_MOTOR_DEFAULT);
			break;

		case OPEN_ROOF:
			serialMessage = String(OPEN_ROOF);
			Roof->Open();
			break;

		case REVERSED_ROOF:
			if (hasValue)
				Roof->SetReversed(value.toInt());
			serialMessage = String(REVERSED_ROOF) + String(Roof->GetReversed());
			break;

		case SPEED_ROOF:
			if (hasValue)
				Roof->SetMaxSpeed(value.toInt());
			serialMessage = String(SPEED_ROOF) + String(Roof->GetMaxSpeed());
			break;

		case STATE_ROOF:
			sTmpString = String(STATE_ROOF);
			serialMessage = sTmpString + Roof->getRoofState();
			break;

		case STEPSPER_ROOF:
			if (hasValue)
				Roof->SetStepsPerStroke(value.toInt());
			serialMessage = String(STEPSPER_ROOF) + String(Roof->GetStepsPerStroke());
			break;

		case VERSION_ROOF:
			serialMessage = String(VERSION_ROOF) + VERSION;
			break;

		default:
			serialMessage = "Unknown command:" + String(command);
			break;
	}


	// Send messages if they aren't empty.
	if (serialMessage.length() > 0) {
		serialMessage += "#";
		switch(nSource) {
			case SERIAL_CMD:
				if(Computer) {
					Computer.write(serialMessage .c_str(), serialMessage.length());
				}
				break;
			case NETWORK_CMD:
				if(domeClient.connected()) {
					DBPrintln("Network serialMessage = " + serialMessage);
					domeClient.write(serialMessage .c_str(), serialMessage.length());
					domeClient.flush();
				}
				break;
		}
	}
}


void Abort()
{
	String shutterMessage;
	if(Roof) {
		Roof->motorStop();
	}
}
