#ifndef H_AVR_ADC
#define H_AVR_ADC

#include "global.h"

/***************************************************************************
// ADC functions 
***************************************************************************/
void ADCEnable(void);
void ADCDisable(void);
void ADCSetChannel( u08 channelNum );
u16 ADCGetValue(void);
void ADCSetResolution( int resolutionType );

// pass one of these in ADCSetResolution()
enum {
	kADC8bitResolution,
	kADC10bitResolution
};


#endif


