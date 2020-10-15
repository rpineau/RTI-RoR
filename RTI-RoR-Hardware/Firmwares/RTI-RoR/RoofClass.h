// Roll off roof code based on the NexDome code.

#include <AccelStepper.h>
#include <DueFlashStorage.h>
#include "StopWatch.h"

#define Computer Serial

// #define DEBUG
#ifdef DEBUG
#define DBPrintln(x) DebugPort.println(x)
#else
#define DBPrintln(x)
#endif // DEBUG


typedef struct _Configuration {
        int             signature;
        uint64_t        stepsPerStroke;
        unsigned long   acceleration;
        unsigned long   maxSpeed;
        uint8_t         reversed;
        uint16_t        cutoffVolts;
        unsigned long   rainCheckInterval;
        bool            rainCheckTwice;
        byte            voltsClose;
        unsigned long   watchdogInterval;
} Configuration;

DueFlashStorage dueFlashStorage;
Configuration config;



#define     STEPPER_ENABLE_PIN      10
#define     STEPPER_DIRECTION_PIN   11
#define     STEPPER_STEP_PIN        12
#define     CLOSED_PIN               2
#define     OPENED_PIN               3
#define     BUTTON_OPEN              5
#define     BUTTON_CLOSE             6
#define     RAIN_SENSOR_PIN          7  // Digital Input from RG11
#define     AT_PARK_PIN             59  // due
#define     AT_PARK_PIN_LOOP        58  // due
#define     VOLTAGE_MONITOR_PIN     A0


#define AD_REF  3.3

#define     EEPROM_SIGNATURE 2640

#define MIN_WATCHDOG_INTERVAL 60000
#define MAX_WATCHDOG_INTERVAL 300000

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN);

// need to make this global so we can access it in the interrupt
enum ShutterStates { OPEN, CLOSED, OPENING, CLOSING, ERROR };
volatile ShutterStates  shutterState = ERROR;

StopWatch watchdogTimer;
StopWatch debounceTimer;

volatile bool       atPark = false;

/*
 * As demonstrated by RCArduino and modified by BKM:
 * pick clock that provides the least error for specified frequency.
 * https://github.com/SomeRandomGuy/DueTimer
 * https://github.com/ivanseidel/DueTimer
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

void stopTimer(Tc *tc, uint32_t channel, IRQn_Type irq)
{
    NVIC_DisableIRQ(irq);
    TC_Stop(tc, channel);
}

// DUE stepper callback
void TC3_Handler()
{
    TC_GetStatus(TC1, 0);
    stepper.run();
}


class RoofClass
{
public:

    // Constructor
    RoofClass();

    bool        m_bWasRunning = false;

    unsigned long watchdogInterval = 90000; // 1.5 minutes


    // Getters
    int32_t         GetAcceleration();
    unsigned long   GetPostion();
    int             GetEndSwitchStatus();
    unsigned long   GetMaxSpeed();
    bool            GetAtPark();
    bool            GetReversed();
    short           GetState();
    unsigned long   GetStepsPerStroke();
    bool        GetVoltsAreLow();
    String      GetVoltString();
    unsigned long           GetRainCheckInterval();
    bool        GetRainCheckTwice();
    bool        GetRainStatus();

    // Setters
    void        SetAcceleration(const unsigned long);
    void        SetMaxSpeed(const unsigned long);
    void        SetReversed(const bool);
    void        SetStepsPerStroke(const unsigned long);
    void        SetVoltsFromString(const String);
    void        SetRainCheckInterval(const unsigned long);
    void        SetCheckRainTwice(const bool);
    void        SetRoofCurrentPosition(long);
    // Movers
    bool        DoButtons();
    void        Open();
    void        Close();
    void        GotoPosition(const unsigned long);
    void        SetWatchdogInterval(const unsigned long);
    byte        GetVoltsClose();
    void        SetVoltsClose(const byte);

    void        EnableMotor(const bool);
    void        Run();
    void        Stop();
    static void motorStop();
    void        motorMoveTo(const long newPosition);
    void        motorMoveRelative(const long amount);
    void        stopInterrupt();
    void        Abort();
    void        Calibrate();
    void        LoadFromEEProm();
    void        SaveToEEProm();

    static void     ClosedInterrupt();
    static void     OpenInterrupt();

private:

    Configuration m_Config;

    float           m_fAdcConvert;
    uint16_t        m_nVolts;
    StopWatch       m_batteryCheckTimer;
    unsigned long   m_nBatteryCheckInterval = 0; // we want to check battery immedialtelly

    uint8_t         m_nLastButtonPressed;


    int         MeasureVoltage();
    void        SetDefaultConfig();
    bool        isAtPark();
    bool        isCalibrating;
};


RoofClass::RoofClass()
{
    int sw1, sw2;
    m_fAdcConvert = 5.0 * (AD_REF / 1024.0) * 100;
    LoadFromEEProm();

    pinMode(CLOSED_PIN, INPUT);
    pinMode(OPENED_PIN, INPUT);

    pinMode(AT_PARK_PIN, INPUT_PULLUP);
    pinMode(AT_PARK_PIN_LOOP, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIRECTION_PIN, OUTPUT);
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    pinMode(BUTTON_OPEN, INPUT_PULLUP);
    pinMode(BUTTON_CLOSE, INPUT_PULLUP);
    pinMode(VOLTAGE_MONITOR_PIN, INPUT);
    pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP);


    LoadFromEEProm();
    SetAcceleration(m_Config.acceleration);
    SetMaxSpeed(m_Config.maxSpeed);
    stepper.setEnablePin(STEPPER_ENABLE_PIN);
    EnableMotor(false);
    GetEndSwitchStatus();
    // set interrupts
    attachInterrupt(digitalPinToInterrupt(CLOSED_PIN), ClosedInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(OPENED_PIN), OpenInterrupt, CHANGE);

    // reset all timers
    m_batteryCheckTimer.reset();
    watchdogTimer.reset();


    // get initial sensor states
    GetState();

    // if the AT_PARK is wired, AT_PARK_PIN will read 0
    digitalWrite(AT_PARK_PIN_LOOP, LOW);
    atPark = digitalRead(AT_PARK_PIN);

    isCalibrating = false;
}

//
// Interrupts
//
void RoofClass::ClosedInterrupt()
{
    int sw1, sw2;

    sw1 = digitalRead(CLOSED_PIN);
    sw2 = digitalRead(OPENED_PIN);

    if (sw1 == HIGH && sw2 == LOW && shutterState == CLOSING) {
        shutterState = CLOSED;
    }

    if(shutterState == CLOSED)
        RoofClass::motorStop();
}

void RoofClass::OpenInterrupt()
{
    int sw1, sw2;

    sw1 = digitalRead(CLOSED_PIN);
    sw2 = digitalRead(OPENED_PIN);

    if (sw1 == LOW && sw2 == HIGH && shutterState == OPENING) {
        shutterState = OPEN;
    }

    if(shutterState == OPEN) {
        RoofClass::motorStop();
        }
}

// EEPROM
void RoofClass::SetDefaultConfig()
{
    memset(&m_Config, 0, sizeof(Configuration));
    m_Config.signature = EEPROM_SIGNATURE;

    m_Config.stepsPerStroke = 885000;
    m_Config.acceleration = 7000;
    m_Config.maxSpeed = 5000;
    m_Config.reversed  = false;
    m_Config.cutoffVolts = 1150;
    m_Config.voltsClose = 0;
    m_Config.watchdogInterval = 90000;
}

void RoofClass::LoadFromEEProm()
{

    byte* data = dueFlashStorage.readAddress(0);
    memcpy(&m_Config, data, sizeof(Configuration));

    if (m_Config.signature != EEPROM_SIGNATURE) {
        SetDefaultConfig();
        SaveToEEProm();
        return;
    }

    if(m_Config.watchdogInterval > MAX_WATCHDOG_INTERVAL)
        m_Config.watchdogInterval = MAX_WATCHDOG_INTERVAL;
    if(m_Config.watchdogInterval < MIN_WATCHDOG_INTERVAL)
        m_Config.watchdogInterval = MIN_WATCHDOG_INTERVAL;
}

void RoofClass::SaveToEEProm()
{
    byte data[sizeof(Configuration)];
    memcpy(data, &m_Config, sizeof(Configuration));
    dueFlashStorage.write(0, data, sizeof(Configuration));
}

// INPUTS
bool RoofClass::DoButtons()
{
    int PRESSED = LOW;
    bool bButtonUsed = false;
    static int whichButtonPressed = 0, lastButtonPressed = 0;


    if (digitalRead(BUTTON_OPEN) == PRESSED && whichButtonPressed == 0 && GetEndSwitchStatus() != OPEN) {
        if(!isAtPark()) {
            DBPrintln("Open Button used but not at park. Doing noting");
        }
        else if(GetRainStatus() == true) { // it's raining
            DBPrintln("Open Button used but it's raining. Doing noting");
        }
        else {
            DBPrintln("Button Open Shutter");
            watchdogTimer.reset();
            whichButtonPressed = BUTTON_OPEN;
            shutterState = OPENING;
            GotoPosition(m_Config.stepsPerStroke);
            lastButtonPressed = BUTTON_OPEN;
        }
        bButtonUsed = true;
    }
    else if (digitalRead(BUTTON_CLOSE) == PRESSED && whichButtonPressed == 0 && GetEndSwitchStatus() != CLOSED) {
        if(!isAtPark()) {
            DBPrintln("Close Button used but not at park. Doing noting");
        }
        else {
            DBPrintln("Button Close Shutter");
            watchdogTimer.reset();
            whichButtonPressed = BUTTON_CLOSE;
            shutterState = CLOSING;
            GotoPosition(0);
            lastButtonPressed = BUTTON_CLOSE;
        }
        bButtonUsed = true;
    }

    if (digitalRead(whichButtonPressed) == !PRESSED && lastButtonPressed > 0) {
        motorStop();
        lastButtonPressed = whichButtonPressed = 0;
        bButtonUsed = false;
    }

    return bButtonUsed;
}

int RoofClass::MeasureVoltage()
{
    int adc;
    float calc;

    adc = analogRead(VOLTAGE_MONITOR_PIN);
    DBPrintln("ADC returns " + String(adc));
    calc = adc * m_fAdcConvert;
    return int(calc);
}

// Getters
int32_t RoofClass::GetAcceleration()
{
    return m_Config.acceleration;
}

int RoofClass::GetEndSwitchStatus()
{
    int result= ERROR;

    if (digitalRead(CLOSED_PIN) == HIGH)
        result = CLOSED;

    if (digitalRead(OPENED_PIN) == HIGH)
        result = OPEN;
    return result;
}

unsigned long RoofClass::GetPostion()
{
    return stepper.currentPosition();
}

void RoofClass::SetRoofCurrentPosition(long nPos)
{
    stepper.setCurrentPosition(nPos);
}

uint32_t RoofClass::GetMaxSpeed()
{
    return stepper.maxSpeed();
}

bool RoofClass::GetAtPark()
{
    return isAtPark();
}

bool RoofClass::isAtPark()
{
    if (digitalRead(AT_PARK_PIN) == LOW)
        atPark = true;
    else
        atPark = false;

    return atPark;
}

bool RoofClass::GetReversed()
{
    return m_Config.reversed;
}

short RoofClass::GetState()
{
    int sw1, sw2;
    DBPrintln("[GetState] 1- shutterState = " + String(shutterState));

    sw1 = digitalRead(CLOSED_PIN);
    sw2 = digitalRead(OPENED_PIN);

    if (sw1 == HIGH && sw2 == HIGH) {
        shutterState = ERROR;
        return (short)shutterState;
        }

    if(shutterState == OPENING || shutterState == CLOSING)
        return (short)shutterState;

    if (sw1 == HIGH && sw2 == LOW)
        shutterState = CLOSED;
    else if (sw1 == LOW && sw2 == HIGH)
        shutterState = OPEN;
    else if (sw1 == LOW && sw2 == LOW)
        shutterState = ERROR;

    if (shutterState == CLOSED) {
            stepper.setCurrentPosition(0);
    }
    else if (shutterState == OPEN) {
            stepper.setCurrentPosition(m_Config.stepsPerStroke);
    }

    DBPrintln("[GetState] 2- shutterState = " + String(shutterState));

    return (short)shutterState;
}

uint32_t RoofClass::GetStepsPerStroke()
{
    return m_Config.stepsPerStroke;
}

inline bool RoofClass::GetVoltsAreLow()
{
    bool low = (m_nVolts <= m_Config.cutoffVolts);
    return low;
}

String RoofClass::GetVoltString()
{
    return String(m_nVolts) + "," + String(m_Config.cutoffVolts);
}

// Setters
void RoofClass::EnableMotor(const bool newState)
{
    if (newState == false) {
        digitalWrite(STEPPER_ENABLE_PIN, HIGH);
        DBPrintln("Outputs disabled");
#if defined ARDUINO_DUE
        stopInterrupt();
#endif
    }
    else {
        digitalWrite(STEPPER_ENABLE_PIN, LOW);
        DBPrintln("Outputs Enabled");
    }
}

void RoofClass::SetAcceleration(const unsigned long accel)
{
     m_Config.acceleration = accel;
    stepper.setAcceleration(accel);
    SaveToEEProm();
}

void RoofClass::SetMaxSpeed(const unsigned long speed)
{
    m_Config.maxSpeed = speed;
    stepper.setMaxSpeed(speed);
    SaveToEEProm();
}

void RoofClass::SetReversed(const bool reversed)
{
    m_Config.reversed = reversed;
    stepper.setPinsInverted(reversed, reversed, reversed);
    SaveToEEProm();
}

void RoofClass::SetStepsPerStroke(const uint32_t newSteps)
{
    m_Config.stepsPerStroke = newSteps;
    SaveToEEProm();
}

void RoofClass::SetVoltsFromString(const String value)
{
    m_Config.cutoffVolts = value.toInt();
    SaveToEEProm();
}

inline unsigned long RoofClass::GetRainCheckInterval()
{
    return m_Config.rainCheckInterval;
}

inline void RoofClass::SetRainCheckInterval(const unsigned long interval)
{
    m_Config.rainCheckInterval = interval;
    SaveToEEProm();
}

inline void RoofClass::SetCheckRainTwice(const bool state)
{
     m_Config.rainCheckTwice = state;
    SaveToEEProm();
}

inline bool RoofClass::GetRainCheckTwice()
{
    return m_Config.rainCheckTwice;
}


bool RoofClass::GetRainStatus()
{
    static int rainCount = 0;
    bool isRaining = false;

    if (!m_Config.rainCheckTwice)
        rainCount = 1;

    if (digitalRead(RAIN_SENSOR_PIN) == HIGH) {
        rainCount = 0;
    }
    else {
        if (digitalRead(RAIN_SENSOR_PIN) == LOW) {
            if (rainCount == 1) {
                isRaining = true;
            }
            else {
                rainCount = 1;
            }
        }
    }

    if(isRaining)
        DBPrintln("[GetRainStatus] it's raining" );
    else
        DBPrintln("[GetRainStatus] it's not raining" );

    return isRaining;
}


// Movers
void RoofClass::Open()
{
    shutterState = OPENING;
    DBPrintln("shutterState = OPENING");
    GotoPosition(m_Config.stepsPerStroke);
}

void RoofClass::Close()
{
    shutterState = CLOSING;
    DBPrintln("shutterState = CLOSING");
    GotoPosition(0);
}

void RoofClass::GotoPosition(const unsigned long newPos)
{
    uint64_t currentPos = stepper.currentPosition();
    bool doMove = false;

    // Check if this actually changes position, then move if necessary.
    if (newPos > currentPos) {
    DBPrintln("shutterState = OPENING");
        shutterState = OPENING;
        doMove = true;
    }
    else if (newPos < currentPos) {
    DBPrintln("shutterState = CLOSING");
        shutterState = CLOSING;
        doMove = true;
    }

    if (doMove) {
        EnableMotor(true);
        motorMoveTo(newPos);
    }
}

inline void RoofClass::SetWatchdogInterval(const unsigned long newInterval)
{
    if(newInterval > MAX_WATCHDOG_INTERVAL)
        m_Config.watchdogInterval = MAX_WATCHDOG_INTERVAL;
    else    if(newInterval < MIN_WATCHDOG_INTERVAL)
        m_Config.watchdogInterval = MIN_WATCHDOG_INTERVAL;
    else
        m_Config.watchdogInterval = newInterval;
    SaveToEEProm();
}

inline void RoofClass::SetVoltsClose(const byte value)
{
    m_Config.voltsClose = value;
    SaveToEEProm();
}

inline byte RoofClass::GetVoltsClose()
{
    return  m_Config.voltsClose;
}

void RoofClass::Run()
{
    static bool hitSwitch = false, firstBatteryCheck = true, doSync = true;
    int open_min, open_max;


    if (digitalRead(CLOSED_PIN) == HIGH && shutterState != OPENING && hitSwitch == false) {
            hitSwitch = true;
            doSync = true;
            shutterState = CLOSED;
            motorStop();
            DBPrintln("Hit closed switch");
            DBPrintln("shutterState = CLOSED");
    }

    if (digitalRead(OPENED_PIN) == HIGH && shutterState != CLOSING && hitSwitch == false) {
            hitSwitch = true;
            shutterState = OPEN;
            motorStop();
            DBPrintln("Hit opened switch");
            DBPrintln("shutterState = OPEN");
    }

    if (stepper.isRunning()) {
        m_bWasRunning = true;
    }

    if (stepper.isRunning() == false && digitalRead(OPENED_PIN) != LOW && digitalRead(CLOSED_PIN) != LOW) {
        shutterState = ERROR;
        DBPrintln("shutterState = ERROR");
    }

    if (m_batteryCheckTimer.elapsed() >= m_nBatteryCheckInterval) {
        DBPrintln("Measuring Battery");
        m_nVolts = MeasureVoltage();
        if (firstBatteryCheck) {
            m_batteryCheckTimer.reset();
            m_nBatteryCheckInterval  = 5000;
            firstBatteryCheck = false;
        }
        else {
            m_batteryCheckTimer.reset();
            m_nBatteryCheckInterval = 120000;
        }
    }

    if (stepper.isRunning())
        return;

    if (doSync && digitalRead(CLOSED_PIN) == HIGH) {
            stepper.setCurrentPosition(0);
            doSync = false;
            DBPrintln("Stopped at closed position");
    }

    if (m_bWasRunning) { // this  only runs once after stopping.
        DBPrintln("WasRunning " + String(shutterState) + " Hitswitch " + String(hitSwitch));
        m_nLastButtonPressed = 0;
        m_bWasRunning = false;
        hitSwitch = false;
        EnableMotor(false);
        if (shutterState == OPEN) {// re-adjust the step per stroke
            open_min = int(stepper.currentPosition() * 0.9);
            open_max = int(stepper.currentPosition() * 1.1);
            if (m_Config.stepsPerStroke < open_min || m_Config.stepsPerStroke > open_max ) {
                SetStepsPerStroke(stepper.currentPosition());
                DBPrintln("new m_Config.stepsPerStroke = " + String(GetStepsPerStroke()));
            }
            else if (isCalibrating) {
                SetStepsPerStroke(stepper.currentPosition());
                DBPrintln("new m_Config.stepsPerStroke = " + String(GetStepsPerStroke()));
                isCalibrating = false;
            }
        }
    }
}


void RoofClass::Abort()
{
    motorStop();
    shutterState = ERROR;
}

void RoofClass::Calibrate()
{
    if (shutterState == CLOSED) {
        SetRoofCurrentPosition(0);
        stepper.setCurrentPosition(0);
        m_Config.stepsPerStroke = 0xFFFFFFFFFFFFFFFF;
        Open();
        isCalibrating = true;
    }
}


void RoofClass::motorStop()
{
    stepper.stop();

}

void RoofClass::stopInterrupt()
{
    DBPrintln("Stopping interrupt");
    // stop interrupt timer
    stopTimer(TC1, 0, TC3_IRQn);
}

void RoofClass::motorMoveTo(const long newPosition)
{
    EnableMotor(true);
    stepper.moveTo(newPosition);

    DBPrintln("Starting interrupt");
    int nFreq;
    nFreq = m_Config.maxSpeed *3 >20000 ? 20000 : m_Config.maxSpeed*3;
    // start interrupt timer
    // AccelStepper run() is called under a timer interrupt
    startTimer(TC1, 0, TC3_IRQn, nFreq);
}

void RoofClass::motorMoveRelative(const long amount)
{

    EnableMotor(true);
    stepper.move(amount);

    DBPrintln("Starting interrupt");
    int nFreq;
    nFreq = m_Config.maxSpeed *3 >20000 ? 20000 : m_Config.maxSpeed*3;
    // start interrupt timer
    // AccelStepper run() is called under a timer interrupt
    startTimer(TC1, 0, TC3_IRQn, nFreq);

}

