// INCLUDES
#include "AVRADC.H"
#include <avr/io.h>

// DEFINES
#define kADCMaxChannelNum  (7)
#define kADMUXChannelMask (240)
#define kADCDefaultResolution (kADC10bitResolution)

// INTERNAL FUNCTION DELCARATIONS
void ADCStartConversion(void);
void ADCWaitConversion(void);

// FUNCTION IMPLEMENTATION
void ADCEnable(void)
{
	// Set the enable bit
	ADCSRA = ADCSRA | (1 << ADEN);

	// We always use AREF as the reference voltage
	ADMUX = ADMUX & (~((1 << REFS1) + (1 << REFS0))) ;

	ADCSetResolution( kADCDefaultResolution );

	// Workaround -- 1st ADCGetValue call always returns zero. Hide that bug!
	ADCGetValue();
}

void ADCDisable(void)
{
	// Clear the enable bit
	ADCSRA = ADCSRA & (~(1 << ADEN));
}

void ADCSetResolution( int resolutionType )
{
	if( resolutionType == kADC10bitResolution  )
		ADMUX = ADMUX & (~(1 << ADLAR));
	else
		ADMUX = ADMUX | (1 << ADLAR);
}

u16 ADCGetValue(void)
{
	u16 val = 0;
	u16 val2 = 0;

	ADCStartConversion();
	ADCWaitConversion();

	if( ADMUX & (1 << ADLAR) )
	{
		// 8-bit resolution
		val = ADCH;
	}
	else
	{
		// Must get ADCL before ADCH in 10-bit resolution

		// 10-bit resolution
		val = ADCL;
		val2 = ADCH;
		val += (val2 << 8);
	}

	//DebugVar( (unsigned char)val, REGISTER, "ADCL" );
	//DebugVar( (unsigned char)val2, REGISTER, "ADCH" );

	return val;
}

void ADCSetChannel( u08 channelNum )
{
	// channel numbers correspond directly to the MUX bits
	// of ADMUX which are also the LSB of ADMUX, makes it easy here
	if( channelNum <= kADCMaxChannelNum )
		ADMUX = (ADMUX & kADMUXChannelMask) | channelNum;
}

 // Need to work out logical and comparator
void ADCStartConversion(void)
{
	ADCSRA = ADCSRA | (1 << ADSC);
}

// Also: fix the logical comparator
void ADCWaitConversion(void)
{
	// Loop until start conversion bit is ready
	//while( (adcsra & (1 << ADSC)) > 0 )
	while( (ADCSRA & (1 << ADIF)) == 0 )
		;

	// Clear the interrupt bit manually since we're not using them here
	// In the debugger, the interrupt bit is not cleared, but the code
	// seems to be correct
	ADCSRA = ADCSRA & (~(1 << ADIF));
}


