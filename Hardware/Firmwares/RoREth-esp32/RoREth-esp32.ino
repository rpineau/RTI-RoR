//
// RTI-Zone Dome Roof firmware.
// for ESP32
//
// Copyright © 2025 Rodolphe Pineau. All rights reserved.
//


// Uncomment #define DEBUG to enable printing debug messages on serial port defined as DebugPort

#include "Arduino.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include <atomic>
#define DEBUG   // enable debug to serial port defined as DebugPort

#ifdef DEBUG
#pragma message "Debug messages enabled"
#define DebugPort Serial1    //  Rx2,Tx2 =  Serial1
#define DBPrint(x) if(DebugPort) DebugPort.print(x)
#define DBPrintln(x) if(DebugPort) DebugPort.println(x)
#define DBPrintHex(x) if(DebugPort) DebugPort.print(x, HEX)
#else
#pragma message "Debug messages disabled"
#define DBPrint(x)
#define DBPrintln(x)
#define DBPrintHex(x)
#endif // DEBUG

#define VERSION "2.645"
#define MAX_TIMEOUT 10

#define USE_EXT_EEPROM
#define USE_ETHERNET
#define USE_ALPACA

#define Computer Serial     // USB = Serial


// FreeRTOS stuff
#define MOTOR_EVENT_BIT	( 1 << 0 )
EventGroupHandle_t xEventGroup;

#include "RoofClass.h"

#ifdef USE_ETHERNET
#pragma message "Ethernet enabled"
// include and some defines for ethernet connection
#include <SPI.h>    // ESP32 :  SCK: GPIO18, SDO/TX: GPIO23, SDI: GPIO19, CS: GPIO5, Reset : GPIO29, Int : GPIO0
#include <Ethernet.h>
#include "EtherMac.h"
#define ETHERNET_CS     5
#define ETHERNET_INT	0
#define ETHERNET_RESET  4
#define CMD_SERVER_PORT 2323
#define domeEthernet Ethernet
uint32_t uidBuffer[4];  // Board unique ID
byte MAC_Address[6];    // Mac address, uses part of the unique ID
IPConfig ServerConfig;
std::atomic<bool> ethernetPresent;
EthernetServer *domeServer = nullptr;
EthernetClient domeClient;
int nbEthernetClient = 0;
String networkBuffer = "";
String sLocalIPAdress = "";
#endif // USE_ETHERNET

String computerBuffer = "";

bool bParked = false; // use to the run check doesn't continuously try to park

RoofClass *Roof = NULL;

static const unsigned long pingInterval = 5000; // 5 seconds, can't be changed with command

// Once booting is done and XBee is ready, broadcast a hello message
// so a shutter knows you're around if it is already running. If not,
// the shutter will send a hello when it boots.
std::atomic<bool> bSentHello;


std::atomic<bool> bShutterPresent;
// global variable for conditon status
std::atomic<bool> bIsBadCondition;
// global variable for shutter voltage state
std::atomic<bool> bLowShutterVoltage;

const char ERR_NO_DATA = -1;

#include "ror_commands.h"
enum CmdSource {SERIAL_CMD, NETWORK_CMD};
// function prototypes
#ifdef USE_ETHERNET
void configureEthernet();
bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, bool bReconfigure);
void checkForNewTCPClient();
#endif // USE_ETHERNET
void openIntHandler();
void closeIntHandler();
void conditionIntHandler();
void buttonHandler();
void resetChip(int);
void StartWirelessConfig();
void ConfigXBee();
void setPANID(String);
void SendHello();
void requestShutterData();
void CheckForCommands();
void CheckForCondition();
#ifdef USE_ETHERNET
void ReceiveNetwork(EthernetClient client);
#endif // USE_ETHERNET
void ReceiveComputer();
void ProcessCommand(int nSource);
void Abort();

#ifdef USE_ALPACA
#include "AlpacaAPI.h"
DomeAlpacaServer *AlpacaServer;
DomeAlpacaDiscoveryServer *AlpacaDiscoveryServer;
#endif

void MotorTask(void *);
esp_task_wdt_config_t twdt_config =
    {
        .timeout_ms = 1000000,
        .idle_core_mask = 0,    // Bitmask of cores
        .trigger_panic = false,
    };
TaskHandle_t motorTaskHandle = nullptr;
//
// Setup and main loops
//
void setup()
{
	ethernetPresent = false;
	bSentHello = false;
	bShutterPresent = false;
	bIsBadCondition = false;
	bLowShutterVoltage = false;

	nbEthernetClient = 0;

	xEventGroup = xEventGroupCreate();
#ifdef DEBUG
	DebugPort.begin(115200, SERIAL_8N1, 16, 17); // pins 16 rx2, 17 tx2, 115200 bps, 8 bits no parity 1 stop bit
	//DebugPort.begin(115200);
	delay(1000);
	DBPrintln("========== RTI-Zone controller booting ==========");
#endif

#ifdef USE_ETHERNET
	digitalWrite(ETHERNET_RESET, 0);
	pinMode(ETHERNET_RESET, OUTPUT);
#endif // USE_ETHERNET

#ifdef USE_ETHERNET
	getMacAddress(MAC_Address, uidBuffer);
	DBPrintln("MAC : " + String(MAC_Address[0], HEX) + String(":") +
					String(MAC_Address[1], HEX) + String(":") +
					String(MAC_Address[2], HEX) + String(":") +
					String(MAC_Address[3], HEX) + String(":") +
					String(MAC_Address[4], HEX) + String(":") +
					String(MAC_Address[5], HEX) );
#endif // USE_ETHERNET

	Computer.begin(115200);
	//Computer.begin(115200, SERIAL_8N1, 16, 17); // pins 16 rx2, 17 tx2, 115200 bps, 8 bits no parity 1 stop bit


	Roof = new RoofClass();
	Roof->motorStop();
	Roof->Stop();
	Roof->EnableMotor(false);
	xEventGroupClearBits(xEventGroup, MOTOR_EVENT_BIT);

#ifdef USE_ETHERNET
	configureEthernet();
#endif // USE_ETHERNET
	rtc_wdt_protect_off();
	esp_task_wdt_deinit();
	esp_task_wdt_init(&twdt_config);
	esp_task_wdt_add(NULL);
	disableCore0WDT();
	disableCore1WDT();
	xTaskCreatePinnedToCore(MotorTask, "MotorTask", 10000, NULL, 16, &motorTaskHandle,  0);

	domeServer = new EthernetServer(CMD_SERVER_PORT);
	domeServer->begin();
#ifdef USE_ALPACA
	AlpacaDiscoveryServer = new DomeAlpacaDiscoveryServer();
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
	const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

#ifdef USE_ETHERNET
	if(ethernetPresent) {
		checkForNewTCPClient();
		AlpacaDiscoveryServer->checkForRequest();
		AlpacaServer->checkForRequest();
	}
#endif //USE_ETHERNET

	CheckForCommands();
	CheckForCondition();
	vTaskDelay(xDelay);
	// taskYIELD();
	esp_task_wdt_reset();
}

//
// This task does all the motor controls
//
void MotorTask(void *)
{
	EventBits_t uxBits;
	const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;

	DBPrintln("========== Motor task starting ==========");
	DBPrintln("========== Motor task Attaching interrupt handler ==========");
	attachInterrupt(digitalPinToInterrupt(CLOSE_PIN), closeIntHandler, FALLING);
	attachInterrupt(digitalPinToInterrupt(OPEN_PIN), openIntHandler, FALLING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_CLOSE), buttonHandler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BUTTON_OPEN), buttonHandler, CHANGE);
	attachInterrupt(digitalPinToInterrupt(COND_SENSOR_PIN), conditionIntHandler, CHANGE);

	esp_task_wdt_add(NULL);
	DBPrintln("========== Motor task ready ==========");

	for(;;) {
		uxBits = xEventGroupWaitBits(
            xEventGroup,
            MOTOR_EVENT_BIT,
            pdTRUE,        // MOTOR_EVENT_BIT should be cleared before returning.
            pdFALSE,
            xTicksToWait ); // Wait a maximum of 100ms for the bit to be set. */

		if( ( uxBits & MOTOR_EVENT_BIT ) != 0 ) {
			Roof->Run();
			if(Roof->isRunning()) {
				xEventGroupSetBits(xEventGroup, MOTOR_EVENT_BIT);
			}
			else {
				xEventGroupClearBits(xEventGroup, MOTOR_EVENT_BIT);
			}
		}
		else {
			// timeout
		}
		vTaskDelay(xDelay);
		// taskYIELD();
		esp_task_wdt_reset();
	}
}

//
//
//
#ifdef USE_ETHERNET
void configureEthernet()
{
        DBPrintln("========== Configuring Ethernet ==========");
        Roof->getIpConfig(ServerConfig);
        ethernetPresent =  initEthernet(ServerConfig.bUseDHCP,
										ServerConfig.ip,
										ServerConfig.dns,
										ServerConfig.gateway,
										ServerConfig.subnet);
}


bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
	bool bDhcpOk;
	int nTimeout = 0;
	DBPrintln("========== Init Ethernet ==========");
	resetChip(ETHERNET_RESET);
	// network configuration
	Ethernet.init(ETHERNET_CS);
	nbEthernetClient = 0;
	// set an ip so we can get the link status
	domeEthernet.begin(MAC_Address, "192.168.0.1", "1.1.1.1", "192.168.0.254", "255.255.255.0");
	while(domeEthernet.linkStatus() == LinkOFF ) {
		delay(250);
		nTimeout++;
		if(nTimeout == 10) {
			return false;
		}
	}
	DBPrintln("========== Setting IP config ==========");
	// try DHCP if set
	if(bUseDHCP) {
		bDhcpOk = domeEthernet.begin(MAC_Address, 10000, 4000); // short timeout
		if(!bDhcpOk) {
			DBPrintln("DHCP Failed!");
			if(domeEthernet.linkStatus() == LinkON ) {
				domeEthernet.begin(MAC_Address, ip, dns, gateway, subnet);
			}
			else {
				DBPrintln("No cable");
				return false;
			}
		}
	}
	else {
		domeEthernet.begin(MAC_Address, ip, dns, gateway, subnet);
	}

	DBPrintln("========== Checking hardware status ==========");
	if(domeEthernet.hardwareStatus() == EthernetNoHardware) {
		 DBPrintln("NO HARDWARE !!!");
		return false;
	}
	DBPrintln("W5500 Ok.");
	DBPrintln("W5500 IP = " + RoofClass::IpAddress2String(Ethernet.localIP()));
	Ethernet.setRetransmissionCount(3);

	DBPrintln("Server ready");
	return true;
}


void checkForNewTCPClient()
{
	if(ServerConfig.bUseDHCP)
		domeEthernet.maintain();

	if(!domeServer)
		return;

	EthernetClient newClient = domeServer->accept();
	if(newClient) {
		DBPrintln("new client");
		if(nbEthernetClient > 0) { // we only accept 1 client
			newClient.write("Already in use#");
			newClient.flush();
			newClient.stop();
			DBPrintln("new client rejected");
		}
		else {
			nbEthernetClient++;
			domeClient = newClient;
			DBPrintln("new client accepted");
			DBPrintln("nb client = " + String(nbEthernetClient));
		}
	}

	if((nbEthernetClient>0) && !domeClient.connected()) {
		DBPrintln("client disconnected");
		domeClient.stop();
		nbEthernetClient--;
		DBPrintln("nb client = " + String(nbEthernetClient));
	}
}
#endif // USE_ETHERNET

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

void IRAM_ATTR buttonHandler()
{
   if(Roof)
	   Roof->ButtonCheck();
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

#ifdef USE_ETHERNET
	if(ethernetPresent ) {
		ReceiveNetwork(domeClient);
	}
#endif // USE_ETHERNET
}

void CheckForCondition()
{
	String shutterMessage;

	int nPosition, nParkPos;
	if(bIsBadCondition != Roof->GetConditionStatus()) { // was there a state change ?
		bIsBadCondition = Roof->GetConditionStatus();
	}
}


#ifdef USE_ETHERNET
void ReceiveNetwork(EthernetClient client)
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
#endif // USE_ETHERNET

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
#ifdef USE_ETHERNET
		case NETWORK_CMD:
			command = networkBuffer.charAt(0);
			// Payload
			value = networkBuffer.substring(1);
			break;
#endif
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
			xEventGroupSetBits(xEventGroup, MOTOR_EVENT_BIT);
			serialMessage = String(CALIBRATE_ROOF);
			break;


		case COND_ROOF:
			serialMessage = String(COND_ROOF) + String(bIsBadCondition ? "1" : "0");
			break;

#ifdef USE_ETHERNET
		case ETH_RECONFIG :
			if(nbEthernetClient > 0) {
				domeClient.stop();
				nbEthernetClient--;
			}
			DBPrintln("Rebooting for Ethernet reconfiguration");
			delay(500);
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
				serialMessage = String(IP_ADDRESS) + String(RoofClass::IpAddress2String(domeEthernet.localIP()));
			}
			break;

		case IP_SUBNET:
			if (hasValue) {
				Roof->setIPSubnet(value);
				Roof->getIpConfig(ServerConfig);
			}
			if(!ServerConfig.bUseDHCP)
				serialMessage = String(IP_SUBNET) + String(Roof->getIPSubnet());
			else {
				serialMessage = String(IP_SUBNET) + String(RoofClass::IpAddress2String(domeEthernet.subnetMask()));
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
				serialMessage = String(IP_GATEWAY) + String(RoofClass::IpAddress2String(domeEthernet.gatewayIP()));
			}
			break;
#endif // USE_ETHERNET


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
			xEventGroupSetBits(xEventGroup, MOTOR_EVENT_BIT);
			break;

		case ROOF_RESTORE_MOTOR_DEFAULT :
			Roof->restoreDefaultMotorSettings();
			serialMessage = String(ROOF_RESTORE_MOTOR_DEFAULT);
			break;

		case OPEN_ROOF:
			serialMessage = String(OPEN_ROOF);
			if(Roof->GetVoltsAreLow())
				serialMessage += "L";
			else {
				Roof->Open();
				xEventGroupSetBits(xEventGroup, MOTOR_EVENT_BIT);
			}
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

		case VOLTS_ROOF:
			if (hasValue) {
				Roof->SetLowVoltageCutoff(value.toInt());
			}
			serialMessage = String(VOLTS_ROOF) + String(Roof->GetVoltString());
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
	#ifdef USE_ETHERNET
			case NETWORK_CMD:
				if(domeClient.connected()) {
					DBPrintln("Network serialMessage = " + serialMessage);
					domeClient.write(serialMessage .c_str(), serialMessage.length());
					domeClient.flush();
				}
				break;
	#endif
		}
	}
}


void Abort()
{
	String shutterMessage;
	if(Roof) {
		Roof->Stop();
		xEventGroupClearBits(xEventGroup, MOTOR_EVENT_BIT);
	}
}
