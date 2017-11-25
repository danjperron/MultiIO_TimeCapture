#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "IOConfig.h"
#include "TimeCapture.h"


bit  WaitForStartDeciSecond;
bit  WaitForEndDeciSecond;
bit  GotCapSenseFlag;

volatile unsigned long TimeCapture[4];

volatile unsigned short TimerHiCount;
volatile unsigned short TimerLoCount;

volatile unsigned short TimeCapture1Count;
volatile unsigned short TimeCapture2Count;
volatile unsigned short TimeCapture3Count;
volatile unsigned short TimeCapture4Count;

void InitTimeCapture(void)
{
        // assume 32Mhz clock
    // clock for timer1 is 8Mhz
    //  32Mhz/4/1 = 8Mhz
   TimeCapture[0]=0;
   TimeCapture[1]=0;
   TimeCapture[2]=0;
   TimeCapture[3]=0;
   
   TimerHiCount = 0;
   TMR1H = 0;
   TMR1L = 0;
   T1CON = 0b00000000;  // fsoc/4  (8Mhz timer clock
 //  T1CON = 0b00110000;  // fsoc/4/8  (1Mhz timer clock
   TMR1GE=0;   // no gate
   TMR1IF=0;
   TMR1IE=1;  // enable interrupt
   TMR1ON=1;  // run timer1

    return;
    
}


void SetTimeCaptureConfig( unsigned char Pin, unsigned detection)
{
    if(Pin == 0)
    {
        if(detection)
            CCP1CON = 0b00000101;  //capture mode rising edge
        else
            CCP1CON = 0b00000100;  //capture mode Falling edge
        // enable IRQ
        TimeCapture1Count=0;
        CCP1IF = 0; //clear interrupt flag
        CCP1IE =1;  // enable interrupt
        
    }
    else if(Pin == 3)
    {
        if(detection)
            CCP2CON = 0b00000101;  //capture mode rising edge
        else
            CCP2CON = 0b00000100;  //capture mode Falling edge
        // enable IRQ
        TimeCapture2Count=0;
        CCP2IF = 0; //clear interrupt flag
        CCP2IE =1;  // enable interrupt
        
    }
    else if(Pin == 8)
    {
        if(detection)
            CCP3CON = 0b00000101;  //capture mode rising edge
        else
            CCP3CON = 0b00000100;  //capture mode Falling edge
        // enable IRQ
        TimeCapture3Count=0;
        CCP3IF = 0; //clear interrupt flag
        CCP3IE =1;  // enable interrupt
        
        
    }
    else if(Pin ==9)
    {
        if(detection)
            CCP4CON = 0b00000101;  //capture mode rising edge
        else
            CCP4CON = 0b00000100;  //capture mode Falling edge
        // enable IRQ
        TimeCapture4Count=0;
        CCP4IF = 0; //clear interrupt flag
        CCP4IE =1;  // enable interrupt
        
        
    }
    return;
    
}


