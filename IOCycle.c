
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif



#include "IOConfig.h"
#include "IOCycle.h"


unsigned char CurrentIOStatus;
near ConfigUnion CurrentIOSensor;
near unsigned char CurrentIOPin;
near unsigned char CurrentIOCycle;



SensorDataUnion  IOSensorData[INPUT_COUNT] @ 0x1A0;
SensorDataUnion  WorkingSensorData;

// Counter will be on same bank
// than IOSensor data (bank 3)
// IOSensor is 10 data of 6 BYTE
// to speed up interrupt
// counter are alternating between COUNTER & ALT_COUNT

// ICOUNTER is use by interrupt to speed it up
unsigned char ICOUNTER[5] @ 0x1E0;
unsigned short COUNTER[5] @ 0x1E6;


volatile unsigned char _TMR0;
volatile unsigned short _TMR0_MSB;
volatile unsigned char BitCount;
volatile unsigned char WorkingByte;
volatile unsigned char CSum;
volatile unsigned  char ByteIndex;
bit      Timer0Overflow;
bit      TimerSecFlag;
bit      ResetCounterFlag;

near unsigned char Retry=0;



void DealWithError(void)
{
   TMR0IE=0;
   TMR0IF=0;
   WorkingSensorData.WORD[2]=0xFFFF;
}


void ScanNextIOPin(void)
{
  unsigned char loop;

  Retry=0;
  CurrentIOPin++;

  if(CurrentIOPin >= INPUT_COUNT)
   CurrentIOPin=0;

  CurrentIOSensor .Config= Setting.IOConfig[CurrentIOPin];
  CurrentIOCycle= IO_CYCLE_IDLE;
  CurrentIOStatus=IO_STATUS_UNKNOWN;
  Timerms=0;
  TMR0IE=0;
  Timer0Overflow=0;


  for(loop=0;loop<SENSOR_DATA_BYTE_MAX;loop++)
      WorkingSensorData.BYTE[loop]=0;

}

void DoIOCycle(void)
{
 

 SensorDataUnion *SensorPt;

  unsigned char  loop;

  if(CurrentIOCycle==  IO_CYCLE_END)
  {
    if(CurrentIOStatus==IO_STATUS_UNKNOWN)
       DealWithError();

    // Re-organize DATA

    SensorPt = &IOSensorData[CurrentIOPin];

    // DHT22 Nothing to do data is correct

    
    if(CurrentIOStatus==IO_STATUS_BAD)
        SensorPt->WORD[0]=0xffff;
    else
    {
        SensorPt->BYTE[0]=0;
        SensorPt->BYTE[1]=CurrentIOStatus;
    }
    
    // now it is time for next sensor;
         ScanNextIOPin();
 
 }
  else
// if(CurrentIOCycle < IO_CYCLE_WAIT)
  {
        // sensor doesn't need cycle  jump to the next one
        ScanNextIOPin();
   }

}


void ResetIOCycle(void)
{
 unsigned char loop;
    
 for(loop=0;loop<INPUT_COUNT;loop++)
 {
 
 CurrentIOCycle= IO_CYCLE_IDLE;
 CurrentIOSensor.Config = Setting.IOConfig[0];
 CurrentIOPin=0;
 }
}

void SetInputMode(unsigned char Pin)
{
 unsigned char _tmp= IOMASK[Pin];
 if(Pin<5)
     TRISB |= _tmp;
 else
     TRISA |= _tmp;
}

void SetOutputMode(unsigned char Pin)
{

 unsigned char _tmp= NOT_IOMASK[Pin];
 if(Pin<5)
     TRISB &= _tmp;
 else
     TRISA &= _tmp;


}



char  ReadIOPin(unsigned char Pin)
{
    unsigned char _tempb;
    unsigned char mask = IOMASK[Pin];
      if(Pin<5)
          _tempb = (unsigned char) (PORTB & mask);
      else
          _tempb =  (unsigned char) (PORTA & mask);
      if(_tempb==0)
          return 0;
      return 1;
}

void WriteIO(unsigned char Pin,unsigned char value)
{

    unsigned char mask = IOMASK[Pin];
    unsigned char nmask = NOT_IOMASK[Pin];

    if(Pin <5)
    {
       if(value==0)
       {
           di();
           PORTB &= nmask;
           ei();
       }
       else
       {
           di();
           PORTB |= mask;
           ei();
       }

    }
    else
    {
       if(value==0)
       {
           di();
           PORTA &= nmask;
           ei();
       }
       else
       {
           di();
           PORTA |= mask;
           ei();
       }

    }


}


void SetIOChange(unsigned char Pin, unsigned char value)
{
    unsigned char _temp;
    if(Pin<5)
        if(value==0)
        {
            _temp = NOT_IOMASK[Pin];
            di();
//          IOCBN&= _temp;
#asm
            movf SetIOChange@_temp,w
            movlb 7
            andwf _IOCBN&0x7F,f
#endasm
            ei();
        }
         else
         {
            _temp= IOMASK[Pin];
            di();
//            IOCBN|= _temp;
#asm
            movf SetIOChange@_temp,w
            movlb 7
            iorwf _IOCBN&0x7f,f
#endasm
            ei();
         }
}




