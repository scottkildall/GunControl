/********************************
// Gun Control
*********************************/

//#define QUICK_TEST
//#define TRACK_STILL_ON

#include <avr/io.h>
#include <stdlib.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include "cmucam.h"
#include "global.h"
#include "timer.h"
#include "uart.h"
#include "rprintf.h"
#include "buffer.h"
#include "AVRADC.h"

/********************************
// Build define flags:
*********************************/
//#define USE_DEBUG

/**********************************
//function prototypes
***********************************/
void Initialize(void);

void AutoMode(void);
void ManualMode(void);

u08 CheckAlwaysSwitch(void);
u08 CheckSensor(void);
u08 CheckPositionSwitch(void);

void DoServoTrack(void);

void ForwardOne(void);
void Forward(u08 steps);

void ReverseOne(void);
void Reverse(u08 steps);

void Position(u08 p);
	
void TrackForward(void);
void TrackReverse(void);
void TrackStill(void);

void StartTracking(void);
void StopTracking(void);
void ResetTracking(void);

void CenterStepper(void);
void Rest(void);

void FlashLED(void);

/**********************************
//defines
***********************************/
#define O_DIR		(DDRB)
#define O_PORT		(PORTB)		// all outputs on Port B

#define I_DIR		(DDRC)


#define PinHigh( pin, port ) 	{ port |= (_BV(pin)); }
#define PinLow( pin, port )	{ port &= ~(_BV(pin)); }

#define L1		(PB3)	//(PB4)
#define L2		(PB2)	// (PB1)
#define L3		(PB1)	//(PB2)
#define L4		(PB4)	//(PB3)

#define POWER_LED	(PB0)

#define SW_ALWAYS	(PC4)	// GENERIC SWITCH -- always on, bypass sensor
#define SW_SENSOR	(PC1)	// SENSOR
#define SW_POSITION	(PC2)	// REED SWITCH for stepper positioning
#define JOYSTICK_INPUT		(PC3)	// ADC 3
#define JOYSTICK_CHANNEL	(3)	// PC3

#define	STEPPER_SERVO_STEPS	(1)	// how many steps per servo position
#define	STEPPER_AUTO_DELAY	(20)
#define STEPPER_STARTUP_DELAY	(100)

#define STEPPER_STILL_STEPS	(15)

#define STEPPER_RANDOM_RANGE	(6)
#define STEPPER_MIN_STEPS	(10)

#define	STEPPER_MANUAL_DELAY	(20)

#define	STEPPER_COUNT	(50)	// 7.2 degrees/step = 50 steps for full revolution

#define JOYSTICK_RIGHT_VAL	(140)	// less than this, go to right
#define	JOYSTICK_LEFT_VAL	(240)	// more than this, go to left

// gets shifted by range from 0-3 for appropriate pin outs on stepper
#define HIGH_NIB_MASK	(240)

#define SERVO_CENTER (40)

//#define SENSOR_TIMEOUT_MS (10000)

#define FORWARD (1)
#define REVERSE (0)

#define TRACK_STILL_LOOPS	(10)	// how many times of no movement on servo until
					// the gun will waver a bit
u08	gSP	= 	0;
u08	gStepperDelay;
u08	gStepperMask[4];

u08	gForward = TRUE;
u08 	gTrackStillCount = 0;

long	gStepperPos;	// Used only in auto mode -- corresponds to servo readings
int	gLastServoPos;

u08	gTrackStillCalled1 = FALSE;
u08	gTrackStillCalled2 = FALSE;
u16	gResetTimer = 0;	

u08	gDirection = FORWARD;
	
/***********************************
//functions
************************************/
int main(void)
{
	Initialize();

#ifdef QUICK_TEST
	FlashLED();
	FlashLED();
		
	while( TRUE )
	{
		if( CheckAlwaysSwitch() )
		{
			if( CheckPositionSwitch() == FALSE )
			{
				PinLow( POWER_LED, O_PORT );
			}
			else
			{
				PinHigh( POWER_LED, O_PORT );
			}
		}
		else
		{	if( CheckSensor() == FALSE )
			{
				PinLow( POWER_LED, O_PORT );
			}
			else
			{
				PinHigh( POWER_LED, O_PORT );
			}
		}
	}
#endif	
	
	AutoMode();
								
	/*while(TRUE)
	{
		if( CheckAlwaysSwitch() )
			AutoMode();	
		else
			ManualMode();
	}*/
				
	return 6;
}


void Initialize(void)
{	
	// Initialize output pins, turn power led on	
	O_DIR = _BV(L1) | _BV(L2) | _BV(L3) | _BV(L4) | _BV(POWER_LED);
	PinHigh( POWER_LED, O_PORT );
	
	// Initialize input pins
	I_DIR = 0x00;
	
	gStepperMask[0] = 3;	// 1 1 0 0
	gStepperMask[1] = 6;	// 0 1 1 0
	gStepperMask[2] = 12;	// 0 0 1 1
	gStepperMask[3] = 9;	// 1 0 0 1
	
	gStepperPos = SERVO_CENTER;
	
	uartInit();
	uartSetBaudRate(38400);
	
	// make all rprintf statements use uart for output
	rprintfInit(uartSendByte);
	
	// initialize the timer system
	timerInit();
	timer1SetPrescaler(TIMER_CLK_DIV1024);

	// wait for hardware to power up
	timerPause(100);
	
	srand(6123143);
}

void AutoMode(void)
{
	gStepperDelay = STEPPER_AUTO_DELAY;
	
	ADCDisable();
	CenterStepper();
	timerPause(100);
	
	// Wait for sensor to activate sequence
	/*while( CheckSensor() == FALSE )
	{
		// back to manual mode
		if( CheckAlwaysSwitch() == FALSE )
			return;
	}*/
	
	StartTracking();
							
	//while( CheckAlwaysSwitch() )
	while( TRUE )
		DoServoTrack();
	
	StopTracking();			
}


void ManualMode(void)
{
	u08	adcVal;
	
	//PinLow( CAMERA_POWER, O_PORT );
	
	gStepperDelay	= STEPPER_MANUAL_DELAY;
	
	ADCEnable();
	
	ADCSetResolution(kADC8bitResolution);	
		
	while( CheckAlwaysSwitch() == FALSE )
	{
		ADCSetChannel(JOYSTICK_CHANNEL);
		adcVal = (u08)ADCGetValue();
		
		if( adcVal < JOYSTICK_RIGHT_VAL )
			ForwardOne();
		else if( adcVal > JOYSTICK_LEFT_VAL )
			ReverseOne();
		else
			Rest();
	}
}

void ForwardOne()
{
	gSP++;
	Position(gSP % 4);
	
	gStepperPos++;
}

void Forward(u08 steps)
{	
	u08 i;
	
	for( i = 0; i < steps; i++ )
		ForwardOne();
}

void ReverseOne()
{
	gSP--;
	Position(gSP % 4);

	gStepperPos--;
}

void Reverse(u08 steps)
{
	u08 i;
	
	for( i = 0; i < steps; i++ )
		ReverseOne();
}

void Position(u08 p)
{
	// optimize later by masking with new pinouts
	//O_PORT = gStepperMask[p] | (O_PORT & HIGH_NIB_MASK);
	
	switch( p )
	{
		case 0:
		{
			PinLow(L4, O_PORT);
			PinLow(L3, O_PORT);
			PinHigh(L2, O_PORT);
			PinHigh(L1, O_PORT);
			break;
		}
		case 1:
		{
			PinLow(L4, O_PORT);
			PinLow(L1, O_PORT);
			PinHigh(L3, O_PORT);
			PinHigh(L2, O_PORT);
			break;
		}
		case 2:
		{
			PinLow(L1, O_PORT);
			PinLow(L2, O_PORT);
			PinHigh(L3, O_PORT);
			PinHigh(L4, O_PORT);
			break;
		}
		case 3:
		{
			PinLow(L3, O_PORT);
			PinLow(L2, O_PORT);
			PinHigh(L4, O_PORT);
			PinHigh(L1, O_PORT);
			break;
		}
	}
	
	timerPause(gStepperDelay);
}
	
void Rest(void)
{
	PinLow(L1, O_PORT);
	PinLow(L2, O_PORT);
	PinLow(L3, O_PORT);
	PinLow(L4, O_PORT);
}

void CenterStepper()
{
	// should go forward at startup
	if( gStepperPos >= SERVO_CENTER )
	{
		gStepperDelay = STEPPER_STARTUP_DELAY;
		ForwardOne();
		gStepperDelay = STEPPER_AUTO_DELAY;
				
		while( CheckPositionSwitch() == FALSE )
			ForwardOne();
	}
	else
	{
		gStepperDelay = STEPPER_STARTUP_DELAY;
		ReverseOne();
		gStepperDelay = STEPPER_AUTO_DELAY;
				
		while( CheckPositionSwitch() == FALSE )
			ReverseOne();
	}
	
	gStepperPos = SERVO_CENTER;
	
}


u08 CheckAlwaysSwitch(void)
{
	if( bit_is_set( PINC, SW_ALWAYS) )
		return TRUE;
	else
		return FALSE;
}

u08 CheckSensor(void)
{
	//if(bit_is_set(PINC, SW_SENSOR_BYPASS))
	//	return TRUE;
	/*else*/ if(bit_is_set(PINC, SW_SENSOR))
		return TRUE;
	else
		return FALSE;
}

u08 CheckPositionSwitch(void)
{
	if( bit_is_set( PINC, SW_POSITION) )
		return TRUE;
	else
		return FALSE;
}

void StartTracking(void)
{
	int result;
	
	// Pay attention to a global timer
	cmuResetTimer();	
	
	uartFlushReceiveBuffer();
	timerPause(300);
			
	// Initialize the camera and begin tracking	
	result = cmuCamInit();
	result= cmuCamStartTracking();
			
	// Can probably get rid of this
	uartFlushReceiveBuffer();
	timerPause(300);
}
	
void StopTracking(void)
{
	cmuCamStopTracking();
}

void ResetTracking(void)
{
	StopTracking();
	timerPause(300);
	CenterStepper();
	StartTracking();
}
	
void DoServoTrack(void)
{
	int	result;
	int	servoPos;	
	int	i;
							
	result = cmuCamGetServoPosition( &servoPos );

	if( result != NPACKET )
	{
		FlashLED();
	}	
	else
	{	
		/*if( gForward )
		{
			gStepperDelay = STEPPER_STARTUP_DELAY;
			ForwardOne();
			gStepperDelay = STEPPER_AUTO_DELAY;
			Forward(STEPPER_AUTO_STEPS);
		
			gForward = FALSE;
		}
		else
		{
			gStepperDelay = STEPPER_STARTUP_DELAY;
			ReverseOne();
			gStepperDelay = STEPPER_AUTO_DELAY;
			Reverse(STEPPER_AUTO_STEPS);
			
			gForward = TRUE;
		}*/
		
		if( servoPos == SERVO_CENTER && gStepperPos != SERVO_CENTER)
			CenterStepper();
		else if( (servoPos - gStepperPos) > 0 )
			TrackForward();
		else if( (servoPos - gStepperPos) < 0)
			TrackReverse();
				
		gStepperPos = servoPos;
		
		if( servoPos != gLastServoPos )
		{
			gResetTimer += cmuGetSec();
			cmuResetTimer();
			gTrackStillCalled1 = FALSE;
			gTrackStillCalled2 = FALSE;
		}
		
	
		gLastServoPos = servoPos;
		
		// 60 second timeout until we ALWAYS reset
		if( cmuGetSec() > 8 || gResetTimer > 60 )
		{
			TrackStill();
			ResetTracking();
			
			gTrackStillCalled1 = FALSE;
			gTrackStillCalled2 = FALSE;
			
			cmuResetTimer();
			gResetTimer = 0;
		}
		
		if( cmuGetSec() > 3 )
		{
			if( gTrackStillCalled1 == FALSE )
			{
				TrackStill();
				gTrackStillCalled1 = TRUE;
			}
		}
		
		else if( cmuGetSec() > 5 )
		{
			if( gTrackStillCalled2 == FALSE )
			{
				TrackStill();
				gTrackStillCalled2 = TRUE;
			}
		}
	}
	
}

void TrackForward(void)
{
	gStepperDelay = STEPPER_STARTUP_DELAY;
	ForwardOne();
	gStepperDelay = STEPPER_AUTO_DELAY;
	Forward(STEPPER_SERVO_STEPS);
}

void TrackReverse(void)
{
	gStepperDelay = STEPPER_STARTUP_DELAY;
	ReverseOne();
	gStepperDelay = STEPPER_AUTO_DELAY;
	Reverse(STEPPER_SERVO_STEPS);
}

void TrackStill(void)
{
	int i;
	u16 randNum;
	u16 steps;
	u16 direction;
	
	
	randNum = (u32)rand();	
	steps = randNum % STEPPER_RANDOM_RANGE;
	
	
	if( gDirection == FORWARD )
	{
		gStepperDelay = STEPPER_STARTUP_DELAY;
		ForwardOne();
		gStepperDelay = STEPPER_AUTO_DELAY;
	
		Forward(steps + STEPPER_MIN_STEPS);	// was STEPPER_STILL_STEPS
		
		gDirection = REVERSE;
	}
	else
	{
		gStepperDelay = STEPPER_STARTUP_DELAY;
		ReverseOne();
		gStepperDelay = STEPPER_AUTO_DELAY;
		
		Reverse(steps + STEPPER_MIN_STEPS);	// was STEPPER_STILL_STEPS
		
		gDirection = FORWARD;
	}
}

void FlashLED(void)
{
	PinLow( POWER_LED, O_PORT );
	timerPause(500);	
	
	PinHigh( POWER_LED, O_PORT );
	timerPause(500);	
}

// OLD SERVO DEBUG CODE

	// grab some servo data
	/*i = 0;
	for( i = 0; i < 10; i++ )
	{	
		servoErr[i] = cmuCamGetServoPosition(&servoPos[i], (resultStr1 + (i*64)) );	
				
		if( servoErr[i] != NPACKET )
			missedPackets++;
	}*/


	/*	
			timerPause(500);
			PinLow( DEBUG_LED, O_PORT );
			
			rprintfCRLF();
			rprintf( "missed packets =\r" );
			rprintfNum(10, 4, FALSE, ' ',   (long)missedPackets ); 
			rprintfCRLF();
			
			
			rprintf( "Dumping servo information\r" );		
			
			for( i = 0; i < 10; i++ )
			{
				rprintfNum(10, 4, FALSE, ' ',   (long)servoErr[i] ); 
				rprintfNum(10, 4, FALSE, ' ',   (long)servoPos[i]); 
				rprintfCRLF();
				rprintfStr( resultStr1 + (i*64) );
				rprintf( "\r" );
			}
	*/		
	