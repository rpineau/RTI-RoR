
#define STEPPER_ENABLE_PIN			10
#define 	STEPPER_DIRECTION_PIN 	11
#define 	STEPPER_STEP_PIN			12

enum  	Direction { DIRECTION_CCW = -1, DIRECTION_CW = 1 };

volatile bool bStepperRunning;
int nStepState;
unsigned int nCurrentSpeed;
unsigned int nMaxSpeed;
unsigned int nAcceleration;
unsigned long nCurrentPosition;
unsigned long nTargetPos;
unsigned long nPosToDecelerate;
unsigned int nDir;
unsigned int nTargetSpeed;
bool bReversed;

class Stepper {

public :
	Stepper();
	~Stepper();

	void enable(bool bEnable);
	void moveTo(unsigned long nPos);
	void stop();
	unsigned long getCurrentPosition();
	void setCurrentPosition(unsigned long nPos);

	unsigned int getMaxSpeed();
	void setMaxSpeed(unsigned int nSpeed);

	void setAcceleration(unsigned int nMaxSpeed);
	unsigned int getAcceletation();
	bool isRunning();

private :
	//set stepper interrupt
	void	setStepperInterrupt();
	void	stopStepperInterrupt();

};


Stepper::Stepper()
{
	bStepperRunning = false;
	nCurrentPosition = 0;
	nTargetPos = 0;
	nPosToDecelerate = 0;
	nCurrentSpeed = 0;
	nMaxSpeed = 0;
	nAcceleration = 0;
	bStepperRunning = 0;
	nDir = DIRECTION_CCW;
	bReversed = false;
	nTargetSpeed = 0;

}

Stepper::~Stepper()
{
}

void Stepper::moveTo(unsigned long nPos)
{
	nTargetPos = nPos;
	if( nPos == nCurrentPosition)
		return;
	if(nPos > nCurrentPosition)
		nDir = bReversed?DIRECTION_CW:DIRECTION_CCW;
	else
		nDir = bReversed?DIRECTION_CCW:DIRECTION_CW;
	// compute when to start decelerating
	// nPosToDecelerate = ????
	digitalWrite(STEPPER_DIRECTION_PIN, nDir);
	nTargetSpeed = nMaxSpeed;
	bStepperRunning = true;
	enable(true);
	setStepperInterrupt();
}

void Stepper::stop()
{
	// start decelleration

	cli();
	nTargetSpeed = 0;
	OCR1A = int((16000000.0 / (8.0 * nMaxSpeed*2)) - 1);
	sei();
}

void Stepper::enable(bool bEnable)
{
	digitalWrite(STEPPER_ENABLE_PIN, bEnable?LOW:HIGH);
}

unsigned long Stepper::getCurrentPosition()
{
	return nCurrentPosition;
}

void Stepper::setCurrentPosition(unsigned long nPos)
{
	nCurrentPosition = nPos;
}

unsigned int Stepper::getMaxSpeed()
{
	return nMaxSpeed;
}

void Stepper::setMaxSpeed(unsigned int nSpeed)
{
	nMaxSpeed = nSpeed;
}

void Stepper::setAcceleration(unsigned int nAcceleration)
{
	nAcceleration = nAcceleration;
}

unsigned int Stepper::getAcceletation()
{
	return nAcceleration;
}

bool Stepper::isRunning()
{
	return bStepperRunning;
}

void Stepper::setStepperInterrupt()
{

  cli();//stop interrupts

  //set timer1 interrupt
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  // set timer count
  OCR1A = int((16000000.0 / (8.0 * nAcceleration*2)) - 1); // speed *2 as the sampling rate
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}

void Stepper::stopStepperInterrupt()
{
  cli();//stop interrupts
    // stop timer compare interrupt
  TIMSK1 &= ~(1 << OCIE1A);
  sei();//allow interrupts

}

ISR(TIMER1_COMPA_vect) { //Interrupt at freq of speed

	int nStep;

	if(!bStepperRunning)
		return;

	if(nStepState) {
		digitalWrite(STEPPER_STEP_PIN, HIGH);
		//  update position
		nStep += nDir;
		nCurrentPosition += nStep;
	}
	else {
		digitalWrite(STEPPER_STEP_PIN, LOW);
	}
	nStepState = !nStepState;

	if(nCurrentPosition == nPosToDecelerate)
		nTargetSpeed=0; // time to stop

	if(nCurrentSpeed < nTargetSpeed) {
		if(nCurrentSpeed == nMaxSpeed) {
			OCR1A = int((16000000.0 / (8.0 * nMaxSpeed*2)) - 1); // speed *2 as the sampling rate
		}
		else {
			// accelerate
			nCurrentSpeed++;
		}
	}
	else if (nCurrentSpeed > nTargetSpeed) {
		// decelerate
		nCurrentSpeed--;
		OCR1A = int((16000000.0 / (8.0 * nMaxSpeed*2)) - 1); // speed *2 as the sampling rate
	}
	else if(nCurrentSpeed < 0) {
		nCurrentSpeed = 0;
		bStepperRunning = false;
		// we're done moving
		// stop timer compare interrupt
		TIMSK1 &= ~(1 << OCIE1A);
	}

}
