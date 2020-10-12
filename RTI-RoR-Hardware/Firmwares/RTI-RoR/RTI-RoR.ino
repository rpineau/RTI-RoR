// Roll off roof code based on the NexDome code.

#include "RoofClass.h"

#define ERR_NO_DATA -1

#define Computer Serial
String serialBuffer;

const String version = "1.00";

// actual commands that deal with the roof
const char CALIBRATE                = 'Z';
const char ABORT_CMD                = 'A';
const char ACCELERATION_ROOF_CMD    = 'E'; // Get/Set stepper acceleration
const char CLOSE_ROOF_CMD           = 'C'; // Close shutter
const char ELEVATION_ROOF_CMD       = 'G'; // Get/Set altitude
const char OPEN_ROOF_CMD            = 'O'; // Open the shutter
const char POSITION_AT_PARK         = 'P'; // Get At Park status
const char WATCHDOG_INTERVAL_SET    = 'I'; // Tell us how long between checks in seconds

const char RAIN_ROOF_GET            = 'F'; // Rotator telling us if it's raining or not
const char RAIN_ROOF_CMD            = 'N'; // Get or Set Rain Check Interval
const char RAIN_ROOF_TWICE_CMD      = 'J'; // Get/Set Rain check requires to hits

const char SPEED_ROOF_CMD           = 'R'; // Get/Set step rate (speed)
const char REVERSED_ROOF_CMD        = 'Y'; // Get/Set stepper reversed status
const char STATE_ROOF_GET           = 'M'; // Get shutter state
const char SET_POSITION             = 'S';  // set the current position (aka sync)
const char STEPSPER_ROOF_CMD        = 'T'; // Get/Set steps per stroke
const char VERSION_ROOF_GET         = 'V'; // Get version string
const char VOLTS_ROOF_CMD           = 'K'; // Get volts and get/set cutoff
const char VOLTSCLOSE_ROOF_CMD     = 'B';


RoofClass Roof;

bool    bManualMode = false;
bool bIsRaining = false;
unsigned long voltUpdateInterval = 5000;

enum HomeStatuses { NEVER_HOMED, HOMED, ATHOME };
StopWatch Rainchecktimer;

void setup()
{
    Computer.begin(115200);
    watchdogTimer.reset();
    Rainchecktimer.reset();
    bIsRaining = Roof.GetRainStatus();
    startTimer(TC1, 0, TC3_IRQn, 20000);
    Roof.EnableOutputs(false);
}

void loop()
{
    if (Computer.available() > 0) {
        watchdogTimer.reset();
        ReceiveSerial();
    }

    if(watchdogTimer.elapsed() >= Roof.watchdogInterval) {
        // we lost communication with the computer.. close everything.
        if (Roof.GetState() != CLOSED && Roof.GetState() != CLOSING) {
            DBPrintln("watchdogTimer triggered.. closing");
            DBPrintln("watchdogTimer.elapsed() = " + String(watchdogTimer.elapsed()));
            DBPrintln("Roof.watchdogInterval = " + String(Roof.watchdogInterval));
            Roof.Close();
            }
    }

    if(Roof.DoButtons()) {
        bManualMode = true;
    }

    if(bManualMode)
        watchdogTimer.reset();

    CheckForRain();
    Roof.Run();
}

/*
 * As demonstrated by RCArduino and modified by BKM:
 * pick clock that provides the least error for specified frequency.
 */
uint8_t pickClock(uint32_t frequency, uint32_t& retRC)
{
    /*
        Timer       Definition
        TIMER_CLOCK1    MCK/2
        TIMER_CLOCK2    MCK/8
        TIMER_CLOCK3    MCK/32
        TIMER_CLOCK4    MCK/128
    */
    struct {
        uint8_t flag;
        uint8_t divisor;
    } clockConfig[] = {
        { TC_CMR_TCCLKS_TIMER_CLOCK1, 2 },
        { TC_CMR_TCCLKS_TIMER_CLOCK2, 8 },
        { TC_CMR_TCCLKS_TIMER_CLOCK3, 32 },
        { TC_CMR_TCCLKS_TIMER_CLOCK4, 128 }
    };
    float ticks;
    float error;
    int clkId = 3;
    int bestClock = 3;
    float bestError = 1.0;
    do
    {
        ticks = (float) VARIANT_MCK / (float) frequency / (float) clockConfig[clkId].divisor;
        error = abs(ticks - round(ticks));
        if (abs(error) < bestError)
        {
            bestClock = clkId;
            bestError = error;
        }
    } while (clkId-- > 0);
    ticks = (float) VARIANT_MCK / (float) frequency / (float) clockConfig[bestClock].divisor;
    retRC = (uint32_t) round(ticks);
    return clockConfig[bestClock].flag;
}


void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
    uint32_t rc = 0;
    uint8_t clock;
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    clock = pickClock(frequency, rc);

    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);
    TC_SetRA(tc, channel, rc/2); //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;

    NVIC_EnableIRQ(irq);
}


// DUE stepper callback
void TC3_Handler()
{
    TC_GetStatus(TC1, 0);
    stepper.run();
}


void CheckForRain()
{
    // Only check periodically
    // Disable by setting rain check interval to 0;
    if(Roof.GetRainCheckInterval() == 0)
        return;
    if(Rainchecktimer.elapsed() >= (Roof.GetRainCheckInterval() * 1000) ) {
        bIsRaining = Roof.GetRainStatus();
        if (bIsRaining &&  Roof.GetState() != CLOSED && Roof.GetState() != CLOSING) {
            DBPrintln("Rain sensor triggered.. closing");
            Roof.Close();
        }
        Rainchecktimer.reset();
    }
}


void ReceiveSerial()
{
    char character = Computer.read();

    if (character != ERR_NO_DATA) {
        if (character == '\r' || character == '\n' || character == '#') {
            // End of message
            if (serialBuffer.length() > 0) {
                ProcessMessages(serialBuffer);
                serialBuffer = "";
            }
        }
        else {
            serialBuffer += String(character);
        }
    }
}


void ProcessMessages(String buffer)
{
    // float localFloat;
    int32_t local32;
    int16_t local16;

    String value, computerMessage="";
    char command;


    if (buffer.equals("OK")) {
        DBPrintln("Buffer == OK");
        return;
    }

    command = buffer.charAt(0);
    value = buffer.substring(1); // Payload if the command has data.
    DBPrintln("<<< Command:" + String(command) + " Value:" + value);

    switch (command) {
        case ACCELERATION_ROOF_CMD:
            if (value.length() > 0) {
                DBPrintln("Set acceleration to " + value);
                local32 = value.toInt();
                Roof.SetAcceleration(local32);
            }
            computerMessage = String(ACCELERATION_ROOF_CMD) + String(Roof.GetAcceleration());
            DBPrintln("Acceleration is " + String(Roof.GetAcceleration()));
            break;

        case CALIBRATE:
            DBPrintln("Starting Calibration");
            Roof.Calibrate();
            computerMessage = String(CALIBRATE);
            break;

        case ELEVATION_ROOF_CMD:
            if (value.length() > 0) {
                Roof.GotoPosition((unsigned long)value.toInt());
                DBPrintln("GoTo " + value);
            }
            else {
                DBPrintln("Current Position " + String(Roof.GetPostion()));
            }
            computerMessage = String(ELEVATION_ROOF_CMD) + String(Roof.GetPostion());
            break;

        case CLOSE_ROOF_CMD:
            DBPrintln("Received CLOSE Shutter Command");
            if (!Roof.GetAtPark()) {
                computerMessage = "CP"; // (C)closecommand not at (P)ark  cancel
                DBPrintln("Not At Park");
            }
            else {
                if (Roof.GetState() != CLOSED) {
                    DBPrintln("Received CLOSE .. calling Roof.Close();");
                    Roof.Close();
                    }
                computerMessage = String(CLOSE_ROOF_CMD);
            }
            bManualMode = false;
            break;

        case OPEN_ROOF_CMD:
            DBPrintln("Received OPEN Shutter Command");
            if (bIsRaining) {
                computerMessage = "OR"; // (O)pen command (R)ain cancel
                DBPrintln("Raining");
            }
            else if (Roof.GetVoltsAreLow()) {
                computerMessage = "OL"; // (O)pen command (L)ow voltage cancel
                DBPrintln("Voltage Low");
            }
            else if (!Roof.GetAtPark()) {
                computerMessage = "OP"; // (O)pen command not at (P)ark  cancel
                DBPrintln("Not At Park");
            }
            else {
                if (Roof.GetState() != OPEN) {
                    DBPrintln("Received OPEN .. calling Roof.Open();");
                    Roof.Open();
                    }
                computerMessage = String(OPEN_ROOF_CMD);
            }
            bManualMode = false;
            break;

        case SET_POSITION:
            if (value.length() > 0) {
                Roof.SetRoofCurrentPosition((long)value.toInt());
                DBPrintln("Set Pos to " + value );
            }
            else {
                DBPrintln("Current Position " + String(Roof.GetPostion()));
            }
            computerMessage = String(ELEVATION_ROOF_CMD) + String(Roof.GetPostion());
            break;

        case POSITION_AT_PARK:
            computerMessage = String(POSITION_AT_PARK) + String(Roof.GetAtPark()?"1":"0");
            DBPrintln(computerMessage);
            break;

        case WATCHDOG_INTERVAL_SET:
            if (value.length() > 0) {
                Roof.SetWatchdogInterval((unsigned long)value.toInt());
                DBPrintln("Watchdog interval set to " + value + "ms");
            }
            else {
                DBPrintln("Rain check interval " + String(Roof.watchdogInterval));
            }
            computerMessage = String(WATCHDOG_INTERVAL_SET) + String(Roof.watchdogInterval);
            break;

        case RAIN_ROOF_GET:
            computerMessage = String(RAIN_ROOF_GET) + String(bIsRaining ? "1" : "0");
            break;

        case RAIN_ROOF_CMD :
            if (value.length() > 0) {
                Roof.SetRainCheckInterval(value.toInt());
            }
            computerMessage = String(RAIN_ROOF_CMD) + String(Roof.GetRainCheckInterval());
            break;

        case RAIN_ROOF_TWICE_CMD:
            if (value.length() > 0) {
                Roof.SetCheckRainTwice(value.equals("1"));
            }
            computerMessage = String(RAIN_ROOF_TWICE_CMD) + String(Roof.GetRainCheckTwice());
            break;


        case REVERSED_ROOF_CMD:
            if (value.length() > 0) {
                Roof.SetReversed(value.equals("1"));
                DBPrintln("Set Reversed to " + value);
            }
            computerMessage = String(REVERSED_ROOF_CMD) + String(Roof.GetReversed());
            DBPrintln(computerMessage);
            break;

        case SPEED_ROOF_CMD:
            if (value.length() > 0) {
                local32 = value.toInt();
                DBPrintln("Set speed to " + value);
                if (local32 > 0) Roof.SetMaxSpeed(value.toInt());
            }
            computerMessage = String(SPEED_ROOF_CMD) + String(Roof.GetMaxSpeed());
            DBPrintln(computerMessage);
            break;

        case STATE_ROOF_GET:
            computerMessage = String(STATE_ROOF_GET) + String(Roof.GetState());
            DBPrintln(computerMessage);
            break;

        case STEPSPER_ROOF_CMD:
            if (value.length() > 0) {
                local32 = value.toInt();
                if (local32 > 0) {
                    Roof.SetStepsPerStroke(local32);
                }
            }
            else {
                DBPrintln("Get Steps " + String(Roof.GetStepsPerStroke()));
            }
            computerMessage = String(STEPSPER_ROOF_CMD) + String(Roof.GetStepsPerStroke());
            break;

        case VERSION_ROOF_GET:
            computerMessage = String(VERSION_ROOF_GET)  + version;
            DBPrintln(computerMessage);
            break;

        case VOLTS_ROOF_CMD:
            if (value.length() > 0) {
                Roof.SetVoltsFromString(value);
                DBPrintln("Set volts to " + value);
            }
            computerMessage = String(VOLTS_ROOF_CMD) + Roof.GetVoltString();
            DBPrintln(computerMessage);
            break;

        case VOLTSCLOSE_ROOF_CMD:
            if (value.length() > 0) {
                DBPrintln("Close on low voltage value inn" + String(value));
                Roof.SetVoltsClose(value.toInt());
            }
            computerMessage = String(VOLTSCLOSE_ROOF_CMD) + String(Roof.GetVoltsClose());
            DBPrintln("Close on low voltage " + String(Roof.GetVoltsClose()));
            break;

        default:
            DBPrintln("Unknown command " + String(command));
            break;
    }

    if (computerMessage.length() > 0) {
        DBPrintln(">>> Sending " + computerMessage);
        Computer.print(computerMessage +"#");
    }
}
