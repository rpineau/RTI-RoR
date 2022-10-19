//
// RTI-Zone Roof firmware.

#define MAX_TIMEOUT 10
#define ERR_NO_DATA -1
#define OK  0

#define VERSION "2.645"

#define USE_EXT_EEPROM
#define USE_ETHERNET

#ifdef USE_ETHERNET
#pragma message "Ethernet enabled"
#define USE_ALPACA
#pragma message "Alpaca server enabled"
// include and some defines for ethernet connection
#include <SPI.h>
#include <Ethernet.h>
#include "EtherMac.h"
#endif // USE_ETHERNET

#define Computer Serial2     // USB FTDI
#define FTDI_RESET  23
#define DebugPort Serial    // programing port

#include "RoofClass.h"

#ifdef USE_ETHERNET
#define ETHERNET_CS     52
#define ETHERNET_RESET  53
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
RoofClass *Roof = NULL;
static const unsigned long resetInterruptInterval = 43200000; // 12 hours
StopWatch ResetInterruptWatchdog;

// global variable for rain status
volatile bool bIsRaining = false;

#ifdef USE_ETHERNET
// global variable for the IP config and to check if we detect the ethernet card
bool ethernetPresent;
IPConfig ServerConfig;
#endif // USE_ETHERNET

// Rotator commands
const char ABORT_MOVE_CMD               = 'a'; // Tell everything to STOP!
const char ETH_RECONFIG                 = 'b'; // reconfigure ethernet
const char CALIBRATE_ROOF_CMD           = 'c'; // Calibrate the roof
const char RESTORE_MOTOR_DEFAULT        = 'd'; // restore default values for motor control.
const char ETH_MAC_ADDRESS              = 'f'; // get the MAC address.
const char IP_ADDRESS                   = 'j'; // get/set the IP address
const char RAIN_ROOF_ACTION             = 'n'; // Get/Set action when rain sensor triggered (do nothing, home, park)
const char IP_SUBNET                    = 'p'; // get/set the ip subnet
const char PANID_GET                    = 'q'; // get and set the XBEE PAN ID
const char STEPSPER_ROOF_CMD            = 't'; // Get/set Steps per open/close
const char IP_GATEWAY                   = 'u'; // get/set default gateway IP
const char VERSION_GET                  = 'v'; // Get Firmware Version
const char IP_DHCP                      = 'w'; // get/set DHCP mode
const char REVERSED_ROOF_CMD            = 'y'; // Get/Set stepper reversed status
const char RAIN_ROOF_GET                = 'F'; // Get rain status (from client) or tell roof it's raining (from Rotator)

const char CLOSE_ROOF_CMD               = 'C'; // Close roof
const char ACCELERATION_ROOF_CMD        = 'E'; // Get/Set stepper acceleration
const char HELLO_CMD                    = 'H'; // Let roof know we're here
const char WATCHDOG_INTERVAL_SET        = 'I'; // Tell roof when to trigger the watchdog for communication loss with rotator
const char VOLTS_ROOF_CMD               = 'K'; // Get volts and set cutoff voltage (close if bellow)
const char STATE_ROOF_GET               = 'M'; // Get roof state
const char OPEN_ROOF_CMD                = 'O'; // Open the roof
const char POSITION_ROOF_GET		    = 'P'; // Get step position
const char SPEED_ROOF_CMD               = 'R'; // Get/Set step rate (speed)
const char VERSION_ROOF_GET             = 'V'; // Get version string


// function prototypes
void checkInterruptTimer();
#ifdef USE_ETHERNET
void configureEthernet();
bool initEthernet(bool bUseDHCP, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet);
void checkForNewTCPClient();
#endif // USE_ETHERNET
void homeIntHandler();
void rainIntHandler();
void buttonHandler();
void resetChip(int);
void resetFTDI(int);
void CheckForCommands();
void CheckForRain();
#ifdef USE_ETHERNET
void ReceiveNetwork(EthernetClient);
#endif // USE_ETHERNET
void ReceiveComputer();
void ProcessCommand();

void setup()
{
    digitalWrite(FTDI_RESET, 0);
    pinMode(FTDI_RESET, OUTPUT);

#ifdef USE_ETHERNET
    digitalWrite(ETHERNET_RESET, 0);
    pinMode(ETHERNET_RESET, OUTPUT);
#endif // USE_ETHERNET

    resetFTDI(FTDI_RESET);

#ifdef DEBUG
    DebugPort.begin(115200);
    DBPrintln("\n\n========== RTI-Zome controller booting ==========\n\n");
#endif
#ifdef USE_ETHERNET
    getMacAddress(MAC_Address, uidBuffer);
#endif // USE_ETHERNET

    Computer.begin(115200);
    Roof = new RoofClass();
    Roof->motorStop();
    Roof->EnableMotor(false);
    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(HOME_PIN), homeIntHandler, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainIntHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_CW), buttonHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_CCW), buttonHandler, CHANGE);
    ResetInterruptWatchdog.reset();
    interrupts();
#ifdef USE_ETHERNET
    configureEthernet();
#endif // USE_ETHERNET
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
    checkInterruptTimer();
}

// reset intterupt as they seem to stop working after a while
void checkInterruptTimer()
{
    if(ResetInterruptWatchdog.elapsed() > resetInterruptInterval ) {
        if(Roof->GetSeekMode() == HOMING_NONE) { // reset interrupt only if not doing anything
            noInterrupts();
            detachInterrupt(digitalPinToInterrupt(HOME_PIN));
            detachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN));
            detachInterrupt(digitalPinToInterrupt(BUTTON_CW));
            detachInterrupt(digitalPinToInterrupt(BUTTON_CCW));
            // re-attach interrupts
            attachInterrupt(digitalPinToInterrupt(HOME_PIN), homeIntHandler, FALLING);
            attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainIntHandler, CHANGE);
            attachInterrupt(digitalPinToInterrupt(BUTTON_CW), buttonHandler, CHANGE);
            attachInterrupt(digitalPinToInterrupt(BUTTON_CCW), buttonHandler, CHANGE);
            ResetInterruptWatchdog.reset();
            interrupts();
        }
    }
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
        dhcpOk = Ethernet.begin(MAC_Address);
        if(!dhcpOk) {
            DBPrintln("DHCP Failed!");
            Ethernet.begin(MAC_Address, ip, dns, gateway, subnet);
        } else {

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

void homeIntHandler()
{
    if(Roof)
        Roof->homeInterrupt();
}

void rainIntHandler()
{
    if(Roof)
        Roof->rainInterrupt();
}

void buttonHandler()
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
        if (Roof->GetRainAction() == HOME)
            Roof->StartHoming();

        if (Roof->GetRainAction() == PARK)
            Roof->GoToAzimuth(Roof->GetParkAzimuth());
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
            Roof->Stop();
            break;

        case ACCELERATION_ROOF_CMD:
            if (hasValue) {
                Roof->SetAcceleration(value.toInt());
            }
            serialMessage = String(ACCELERATION_ROOF_CMD) + String(Roof->GetAcceleration());
            break;

        case CALIBRATE_ROOF_CMD:
            Roof->StartCalibrating();
            serialMessage = String(CALIBRATE_ROOF_CMD);
            break;


        case RAIN_ROOF_ACTION:
            if (hasValue) {
                Roof->SetRainAction(value.toInt());
            }
            serialMessage = String(RAIN_ROOF_ACTION) + String(Roof->GetRainAction());
            break;

        case SPEED_ROOF_CMD:
            if (hasValue)
                Roof->SetMaxSpeed(value.toInt());
            serialMessage = String(SPEED_ROOF_CMD) + String(Roof->GetMaxSpeed());
            break;

        case REVERSED_ROOF_CMD:
            if (hasValue)
                Roof->SetReversed(value.toInt());
            serialMessage = String(REVERSED_ROOF_CMD) + String(Roof->GetReversed());
            break;

        case RESTORE_MOTOR_DEFAULT:
            Roof->restoreDefaultMotorSettings();
            serialMessage = String(RESTORE_MOTOR_DEFAULT);
            break;

        case STEPSPER_ROOF_CMD:
            if (hasValue)
                Roof->SetStepsPerRotation(value.toInt());
            serialMessage = String(STEPSPER_ROOF_CMD) + String(Roof->GetStepsPerRotation());
            break;

        case VERSION_GET:
            serialMessage = String(VERSION_GET) + VERSION;
            break;

        case VOLTS_ROOF_CMD:
            if (hasValue) {
                Roof->SetLowVoltageCutoff(value.toInt());
            }
            serialMessage = String(VOLTS_ROOF_CMD) + String(Roof->GetVoltString());
            break;

        case RAIN_ROOF_GET:
            serialMessage = String(RAIN_ROOF_GET) + String(bIsRaining ? "1" : "0");
            break;

#ifdef USE_ETHERNET
        case ETH_RECONFIG :
            if(nbEthernetClient > 0) {
                domeClient.stop();
                nbEthernetClient--;
            }
            configureEthernet();
            serialMessage = String(ETH_RECONFIG)  + String(ethernetPresent?"1":"0");
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
                serialMessage = String(IP_ADDRESS) + String(Roof->IpAddress2String(Ethernet.localIP()));
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
                serialMessage = String(IP_SUBNET) + String(Roof->IpAddress2String(Ethernet.subnetMask()));
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
                serialMessage = String(IP_GATEWAY) + String(Roof->IpAddress2String(Ethernet.gatewayIP()));
            }
            break;
#endif // USE_ETHERNET


        default:
            serialMessage = "Unknown command:" + String(command);
            break;
    }


    // Send messages if they aren't empty.
    if (serialMessage.length() > 0) {
        if(!bFromNetwork) {
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





