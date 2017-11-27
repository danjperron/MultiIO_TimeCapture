
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif


#ifndef CAPSENSE_H
#define	CAPSENSE_H

#ifdef	__cplusplus
extern "C" {
#endif


void SetTimeCaptureConfig( unsigned char Pin, unsigned detection);
void InitTimeCapture(void);
void SetTimer1Clock(unsigned char value);

extern bit  WaitForStartDeciSecond;
extern bit  WaitForEndDeciSecond;
extern bit  GotCapSenseFlag;

extern volatile unsigned short TimeCapture1Count;
extern volatile unsigned short TimeCapture2Count;
extern volatile unsigned short TimeCapture3Count;
extern volatile unsigned short TimeCapture4Count;

extern volatile unsigned long TimeCapture[4];

extern volatile unsigned short TimerHiCount;
extern unsigned char Timer1Clock;



#endif