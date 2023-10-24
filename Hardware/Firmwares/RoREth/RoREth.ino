//
// RTI-Zone RoR firmware.
// 

// Debug printing, uncomment #define DEBUG to enable
#define DEBUG
#ifdef DEBUG
#pragma message "Debug messages enabled"
#define DebugPort Serial    // Programming port
#define DBPrint(x) if(DebugPort) DebugPort.print(x)
#define DBPrintln(x) if(DebugPort) DebugPort.println(x)
#define DBPrintHex(x) if(DebugPort) DebugPort.print(x, HEX)
#else
#pragma message "Debug messages disabled"
#define DBPrint(x)
#define DBPrintln(x)
#define DBPrintHex(x)
#endif // DEBUG


#define MAX_TIMEOUT 10
#define ERR_NO_DATA -1
#define OK  0

#define VERSION "2.645"

#define USE_EXT_EEPROM
#define USE_ETHERNET
//#define USE_ALPACA

#ifdef USE_ETHERNET
#pragma message "Ethernet enabled"
#ifdef USE_ALPACA
#pragma message "Alpaca server enabled"
#endif
// include and some defines for ethernet connection
#include <SPI.h>
#include <Ethernet.h>
#include "EtherMac.h"
#endif // USE_ETHERNET

#pragma message "RP2040 Serial2"
#define Computer Serial2     // USB FTDI

#define FTDI_RESET  28

#include "RoofClass.h"

#ifdef USE_ETHERNET
#define ETHERNET_CS     17
#define ETHERNET_RESET  20
uint32_t uidBuffer[4];  // DUE unique ID
byte MAC_Address[6];    // Mac address, uses part of the unique ID

#define SERVER_PORT 2323
EthernetServer domeServer(SERVER_PORT);
#ifdef USE_ALPACA
#include "RTI-DomeAlpacaServer.h"
#define ALPACA_SERVER_PORT 11111
RTIDomeAlpacaServer AlpacaServer(ALPACA_SERVER_PORT);
#endif // USE_ALPACA
EthernetClient domeClient;
int nbEthernetClient;
String networkBuffer;
#endif // USE_ETHERNET

String computerBuffer;
volatile bool core0Ready = false;

bool bParked = false; // use to the rin check doesn't continuously try to park/close

RoofClass *Roof = NULL;

// global variable for rain status
volatile bool bIsRaining = false;
// global variable for shutter voltage state
volatile bool bLowShutterVoltage = false;


#ifdef USE_ETHERNET
// global variable for the IP config and to check if we detect the ethernet card
bool ethernetPresent;
IPConfig ServerConfig;
#endif // USE_ETHERNET

// Roof commands
const char ABORT_MOVE_CMD               = 'a'; // Tell everything to STOP!
const char ETH_RECONFIG                 = 'b'; // reconfigure ethernet
const char CALIBRATE_ROTATOR_CMD        = 'c'; // Calibrate the dome
const char RESTORE_MOTOR_DEFAULT        = 'd'; // restore default values for motor control.
const char ETH_MAC_ADDRESS              = 'f'; // get the MAC address.
const char IP_ADDRESS                   = 'j'; // get/set the IP address
const char VOLTS_ROTATOR_CMD            = 'k'; // Get volts and get/set cutoff
const char SLEW_STATUS_GET             = 'm'; // Get Slewing status/direction
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
const char POSITION_ROOF_GET		    = 'P'; // Get step position
const char SHUTTER_PANID_GET            = 'Q'; // get and set the XBEE PAN ID
const char SPEED_ROOF_CMD            = 'R'; // Get/Set step rate (speed)
const char STEPSPER_ROOF_CMD         = 'T'; // Get/Set steps per stroke
const char VERSION_ROOF_GET          = 'V'; // Get version string
const char REVERSED_ROOF_CMD         = 'Y'; // Get/Set stepper reversed status
const char SOUTH_WALL_PRESENT        =  'Z'; // enable/disable south wall operations

// function prototypes
#ifdef USE_ETHERNET
void configureEthernet();
bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet);
void checkForNewTCPClient();
#endif // USE_ETHERNET
void checkInterruptTimer();
void rainIntHandler();
void buttonHandler();
void resetChip(int);
void resetFTDI(int);
void CheckForCommands();
void CheckForRain();
void PingShutter();
#ifdef USE_ETHERNET
void ReceiveNetwork(EthernetClient);
#endif // USE_ETHERNET
void ReceiveComputer();
void ProcessCommand();

void setup()
{
    core0Ready = false;

    digitalWrite(FTDI_RESET, 0);
    pinMode(FTDI_RESET, OUTPUT);

#ifdef USE_ETHERNET
    digitalWrite(ETHERNET_RESET, 0);
    pinMode(ETHERNET_RESET, OUTPUT);
#endif // USE_ETHERNET

    resetFTDI(FTDI_RESET);

#ifdef DEBUG
    DebugPort.begin(115200);
    DBPrintln("\n\n========== RTI-Zome RoR controller booting ==========\n\n");
#endif
#ifdef USE_ETHERNET
    getMacAddress(MAC_Address, uidBuffer);
#ifdef DEBUG
    DBPrintln("MAC : " + String(MAC_Address[0], HEX) + String(":") +
                    String(MAC_Address[1], HEX) + String(":") +
                    String(MAC_Address[2], HEX) + String(":") +
                    String(MAC_Address[3], HEX) + String(":") +
                    String(MAC_Address[4], HEX) + String(":") +
                    String(MAC_Address[5], HEX) );
#endif
#endif // USE_ETHERNET

    Computer.begin(115200);

    Roof = new RoofClass();
    Roof->motorStop();
    Roof->EnableMotor(false);

#ifdef USE_ETHERNET
    configureEthernet();
#endif // USE_ETHERNET
    core0Ready = true;
    DBPrintln("========== Core 0 ready ==========");
}

void setup1()
{
    while(!core0Ready)
        delay(100);

    DBPrintln("========== Core 1 starting ==========");

    DBPrintln("========== Core 1 Attaching interrupt handler ==========");
    attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainIntHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_OPEN), buttonHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_CLOSE), buttonHandler, CHANGE);

    DBPrintln("========== Core 1 ready ==========");
}

void loop()
{

#ifdef USE_ETHERNET
    if(ethernetPresent)
        checkForNewTCPClient();
#endif // USE_ETHERNET

    Roof->Run();
    CheckForCommands();
    CheckForRain();
}

void loop1()
{   // all stepper motor code runs on core 1
    Roof->Run();
}

#ifdef USE_ETHERNET
void configureEthernet()
{
    Roof->getIpConfig(ServerConfig);
    ethernetPresent =  initEthernet(ServerConfig.bUseDHCP,
                                    ServerConfig.ip,
                                    ServerConfig.dns,
                                    ServerConfig.gateway,
                                    ServerConfig.subnet);
}


bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
    int dhcpOk;
#ifdef DEBUG
    IPAddress aTmp;
#endif

    resetChip(ETHERNET_RESET);
    // network configuration
    nbEthernetClient = 0;
    Ethernet.init(ETHERNET_CS);

    // try DHCP if set
    if(bUseDHCP) {
        dhcpOk = Ethernet.begin(MAC_Address, 10000, 4000); // short timeout
        if(!dhcpOk) {
            DBPrintln("DHCP Failed!");
            if(Ethernet.linkStatus() == LinkON )
                Ethernet.begin(MAC_Address, ip, dns, gateway, subnet);
            else {
                DBPrintln("No cable");
                return false;
            }
        }
    }
    else {
        Ethernet.begin(MAC_Address, ip, dns, gateway, subnet);
    }

    if(Ethernet.hardwareStatus() == EthernetNoHardware) {
         DBPrintln("NO HARDWARE !!!");
        return false;
    }
#ifdef DEBUG
    aTmp = Ethernet.localIP();
    DBPrintln("IP = " + String(aTmp[0]) + String(".") +
                        String(aTmp[1]) + String(".") +
                        String(aTmp[2]) + String(".") +
                        String(aTmp[3]) );
#endif

    Ethernet.setRetransmissionCount(3);

    DBPrintln("Server ready, calling begin()");
    domeServer.begin();
#ifdef USE_ALPACA
    AlpacaServer.startServer();
#endif // USE_ALPACA
    return true;
}


void checkForNewTCPClient()
{
    if(ServerConfig.bUseDHCP)
        Ethernet.maintain();

    EthernetClient newClient = domeServer.accept();
    if(newClient) {
        DBPrintln("new client");
        if(nbEthernetClient > 0) { // we only accept 1 client
            newClient.print("Already in use#");
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
        configureEthernet();
    }
}
#endif // USE_ETHERNET

void rainIntHandler()
{
    if(Roof)
      Roof->rainInterrupt();
}

void handleClosedInterrupt()
{
    if(Roof)
      Roof->ClosedInterrupt();
}

void buttonHandler()
{
    if(Roof)
      Roof->DoButtons();
}

// reset chip with /reset connected to nPin
void resetChip(int nPin)
{
    digitalWrite(nPin, 0);
    delay(2);
    digitalWrite(nPin, 1);
    delay(10);
}

//reset FTDI FT232 usb to serial chip
void resetFTDI(int nPin)
{
    digitalWrite(nPin,0);
    delay(1000);
    digitalWrite(nPin,1);
}


void CheckForCommands()
{
    ReceiveComputer();

#ifdef USE_ETHERNET
    if(ethernetPresent )
        ReceiveNetwork(domeClient);
#endif // USE_ETHERNET
}

void CheckForRain()
{

    if(bIsRaining != Roof->GetRainStatus()) { // was there a state change ?
        bIsRaining = Roof->GetRainStatus();
    }
    if (bIsRaining) {
        if (Roof->GetState() == OPEN)
            Roof->Close();
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
                    ProcessCommand(true);
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

// All comms are terminated with '#' but the '\r' and '\n' are for XBee config
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
                    ProcessCommand(false);
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

void ProcessCommand(bool bFromNetwork)
{
    float fTmp;
    char command;
    String value;

    String serialMessage, sTmpString;
    bool hasValue = false;

    // Split the buffer into command char and value if present
    // Command character
    if(bFromNetwork) {
#ifdef USE_ETHERNET
        command = networkBuffer.charAt(0);
        // Payload
        value = networkBuffer.substring(1);
#endif // USE_ETHERNET
    }
    else {
        command = computerBuffer.charAt(0);
        // Payload
        value = computerBuffer.substring(1);
    }
    // payload has data
    if (value.length() > 0)
        hasValue = true;

    serialMessage = "";

    DBPrintln("\nProcessCommand");
    DBPrintln("Command = \"" + String(command) +"\"");
    DBPrintln("Value = \"" + String(value) +"\"");
    DBPrintln("bFromNetwork = \"" + String(bFromNetwork?"Yes":"No") +"\"");


    switch (command) {
        case ABORT_MOVE_CMD:
            sTmpString = String(ABORT_MOVE_CMD);
            serialMessage = sTmpString;
            Roof->motorStop();
            DBPrintln(serialMessage);
            break;

        case VOLTS_ROTATOR_CMD:
            if (hasValue) {
                Roof->SetVoltsFromString(value);
            }
            serialMessage = String(VOLTS_ROTATOR_CMD) + String(Roof->GetVoltString());
            DBPrintln(serialMessage);
            break;

        case RAIN_ROOF_GET:
            serialMessage = String(RAIN_ROOF_GET) + String(bIsRaining ? "1" : "0");
            DBPrintln(serialMessage);
            break;

#ifdef USE_ETHERNET
        case ETH_RECONFIG :
            if(nbEthernetClient > 0) {
                domeClient.stop();
                nbEthernetClient--;
            }
            configureEthernet();
            serialMessage = String(ETH_RECONFIG)  + String(ethernetPresent?"1":"0");
            DBPrintln(serialMessage);
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
            DBPrintln(serialMessage);
            break;

        case IP_DHCP:
            if (hasValue) {
                Roof->setDHCPFlag(value.toInt() == 0 ? false : true);
            }
            serialMessage = String(IP_DHCP) + String( Roof->getDHCPFlag()? "1" : "0");
            DBPrintln(serialMessage);
            break;

        case IP_ADDRESS:
            if (hasValue) {
                Roof->setIPAddress(value);
                Roof->getIpConfig(ServerConfig);
            }
            if(!ServerConfig.bUseDHCP)
                serialMessage = String(IP_ADDRESS) + String(Roof->getIPAddress());
            else {
                serialMessage = String(IP_ADDRESS) + String(Roof->IpAddress2String(Ethernet.localIP()));
            }
            DBPrintln(serialMessage);
            break;

        case IP_SUBNET:
            if (hasValue) {
                Roof->setIPSubnet(value);
                Roof->getIpConfig(ServerConfig);
            }
            if(!ServerConfig.bUseDHCP)
                serialMessage = String(IP_SUBNET) + String(Roof->getIPSubnet());
            else {
                serialMessage = String(IP_SUBNET) + String(Roof->IpAddress2String(Ethernet.subnetMask()));
            }
            DBPrintln(serialMessage);
            break;

        case IP_GATEWAY:
            if (hasValue) {
                Roof->setIPGateway(value);
                Roof->getIpConfig(ServerConfig);
            }
            if(!ServerConfig.bUseDHCP)
                serialMessage = String(IP_GATEWAY) + String(Roof->getIPGateway());
            else {
                serialMessage = String(IP_GATEWAY) + String(Roof->IpAddress2String(Ethernet.gatewayIP()));
            }
            DBPrintln(serialMessage);
            break;
#endif // USE_ETHERNET

        case ACCELERATION_ROOF_CMD:
            if (hasValue) {
                Roof->SetAcceleration(value.toInt());
            }
            serialMessage = String(ACCELERATION_ROOF_CMD) + String(Roof->GetAcceleration());
            DBPrintln(serialMessage);
            break;

        case CLOSE_ROOF_CMD:
            if (Roof->GetState() != CLOSED) {
                Roof->Close();
            }
            serialMessage = String(STATE_ROOF_GET) + String(Roof->GetState());
             DBPrintln(serialMessage);
           break;

        case SHUTTER_RESTORE_MOTOR_DEFAULT :
            Roof->restoreDefaultMotorSettings();
            serialMessage = String(RESTORE_MOTOR_DEFAULT);
            DBPrintln(serialMessage);
            break;

        case SLEW_ROTATOR_GET:
            serialMessage = String(SLEW_ROTATOR_GET) + String(Rotator->GetDirection());
            break;

        case OPEN_ROOF_CMD:
            if (bIsRaining) {
                serialMessage = "OR"; // (O)pen command (R)ain cancel
            }
            else if (Roof->GetVoltsAreLow()) {
                serialMessage = "OL"; // (O)pen command (L)ow voltage cancel
            }
            else {
                serialMessage = String(OPEN_ROOF_CMD); // (O)pen command
                if (Roof->GetState() != OPEN)
                    Roof->Open();
            }
            DBPrintln(serialMessage);
            break;

        case REVERSED_ROOF_CMD:
            if (hasValue) {
                Roof->SetReversed(value.equals("1"));
            }
            serialMessage = String(REVERSED_ROOF_CMD) + String(Roof->GetReversed());
            DBPrintln(serialMessage);
            break;

        case SPEED_ROOF_CMD:
            if (hasValue) {
                DBPrintln("Set speed to " + value);
                if (value.toInt() > 0) Roof->SetMaxSpeed(value.toInt());
            }
            serialMessage = String(SPEED_ROOF_CMD) + String(Roof->GetMaxSpeed());
            DBPrintln(serialMessage);
            break;

        case STATE_ROOF_GET:
            serialMessage = String(STATE_ROOF_GET) + String(Roof->GetState());
            DBPrintln(serialMessage);
            break;

        case STEPSPER_ROOF_CMD:
            if (hasValue) {
                if (value.toInt() > 0) {
                    Roof->SetStepsPerStroke(value.toInt());
                }
            }
            else {
                DBPrintln("Get Steps " + String(Roof->GetStepsPerStroke()));
            }
            serialMessage = String(STEPSPER_ROOF_CMD) + String(Roof->GetStepsPerStroke());
            DBPrintln(serialMessage);
            break;

        case VERSION_ROOF_GET:
            serialMessage = String(VERSION_ROOF_GET) + VERSION;
            DBPrintln(serialMessage);
            break;

        default:
            serialMessage = "Unknown command:" + String(command);
            DBPrintln(serialMessage);
            break;
    }
        

    // Send messages if they aren't empty.
    if (serialMessage.length() > 0) {
        if(!bFromNetwork) {
            if(Computer)
                Computer.print(serialMessage + "#");
            }
#ifdef USE_ETHERNET
        else if(domeClient.connected()) {
                DBPrintln("Network serialMessage = " + serialMessage);
                domeClient.print(serialMessage + "#");
                domeClient.flush();
        }
#endif // USE_ETHERNET
    }
}




