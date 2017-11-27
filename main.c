/* 
 * File:   main.c
 * Author: daniel
 *
 *  Date 21 Novembre 2017
 * 
 * 
 * 
 *  Version 1.01 Novembre 2017
 *  - Version simplifé qui utilise seulement
 *  - I/O en entrée ou sortie
 *  - Detection de capture de tempsavec l'utilisation du compteur1 (TIMER1)
 *    et de CCPx 
 */ 

#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "IOConfig.h"
#include "TimeCapture.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MultiIO10   Multifunction 10 pins
//
// Simple program with different function on each pin
// using RS-485 with modbus protocol to communicate
// 
//
//  List of IO mode
//
//
// 1- Digital input.  ( IO0..IO4 could have PULL_UP resistor)
// 2- Digital output.
// 3- Analog input withc selectable reference voltage.
//    (1.024,2.048,4.096V and  VDD).
// 4- Timer capture

//   Date: 11 Novembre 2017
//   programmer: Daniel Perron
//   Version: 1.01
//   Processeur: PIC16F1827
//   logiciel de compilation:  Microchip MPLAB  X IDE (freeware version)
//
//   The system use serial communication at 57600 BAUD with modbus protocol.
//  
//   Pin RB0 control the serial communication direction.
//
//
//   MODBUS REGISTER LIST
//
//
/////  Modbus Function  01 & 02
//
//  Lire IO0 to IO9
//
//  Modbus [SLA][1][0][A][0][1][CRC]
//
//  SLA - Adresse
//  A   - 0=IO0 1=IO1 ... 9=IO9
//  CRC - Cyclical redundancy check.
//
//  Ex:   Lire IO0 Bit sur le module 1  avec PicModule.py
//
//    import PicModule
//    m1 = PicModule.PicMbus(1)
//    m1.readIO(0)
//
//
//
/////  Modbus Function 3
//
//  Modbus [SLA][3][0][A][0][1][CRC]
//
//  A =
//    0,3,8 ou 9:  Lire Les timer de capture sur les IO 0,3,8 ou 9
//     160 :  Adresse modbus du module
//     240 :  Lire Horloge1  (32bits, 4 octets) (240= Hi word 241=LowWord)
//     250 :  Software version
//     251 :  Software Id number
// (0x100 .. 0x109):   Lire la cpnfiguration de chaque IO
// 
//
//
// Ex:  Python
//   Lire numéro d'identification du logiciel
//   m1.readId():
//
//   ou passer par modbus directement
//
//   m1.module.read_register(251,0,3);
//
//
///// Modbus Fonction 4
//
//  Modbus [SLA][4][0][A][0][Number of register][crc]
//

  // Function 4  Lire Register current
  //
  // Address 0x1000:  Valeur A/D de la reference VRef 2.048V versus VCC
  // Address 0x1001:  Valeur de la diode pour lire latemperature
  // Address 0x00n0:  Valeur du IOn ou n est le IO(0..9) spécifique.
//
//  Ex: Lire compteur de capture su IO0
//
//     m1.readTimer(0)
//
///// Modbus Fonction 5
//
//  Modbus [SLA][5][0][A][0][DATA][crc]
//
//  A = 0..9 : Set IO bit output
//
//
///// Modbus Fonction 6
//
//  Modbus [SLA][6][0][A][16 bits DATA][CRC]
//
//  A = 0..9:  Reset compteur de temps
//       160:  change la valeur de la nouvelle adresse modbus
//       240:  Reset horloge1 (Timer1)
// 0x100..0x10a:  Set new IO configuration  (IOCONFIG.H)

//
//  ex:    Configurer IO3 pour capture de temps
//  
//   m1.config(3,m1.IOCONFIG_TIME_CAPTURE_LOW)
// 
//


#define SOFTWARE_ID      0x653B
#define RELEASE_VERSION  0x0101



///////////////////  How to program the IC.
//
//    1 - Use MPLAB with pickit 3  (need version 8.90 at least)
//  or
//    2 - Use standalone pickit2 v 2.61 (Select the I.C. and load the corresponding hex file).
//  or
//    3 - Use Raspberry Pi board  with burnVLP.py . Software available from https://github.com/danjperron/burnLVP
//
////////////////////////////////////  GPL LICENSE ///////////////////////////////////


/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*  COMMUNICATION PROTOCOL

     baud rate 57600 , 8 bits data , no parity
    
*/
//////////////////////////  PORT DESCRIPTION
/*
 * IO5 RA0
 * IO6 RA1
 * IO7 RA2
 * IO8 RA3                          CPP3
 * IO9 RA4                          CPP4
 *     RA5    MCLR (RESET)
 *     RA6    CRYSTAL CLOCK
 *     RA7    CRYSTAL CLOCK
 *     RB0    RS-485 DIRECTION
 * IO1 RB1
 *     RB2    SERIAL IN
 * IO0 RB3                          CPP1
 * IO2 RB4
 *     RB5    SERIAL OUT
 * IO3 RB6                          CPP2
 * IO4 RB7
 *
 * P.S. ONLY IO0..IO4 support Counter, Pull-up, Cap Sense and DHT type sensor.
 *
 *
*/



//rs-485 data direction

#define TXM_ENABLE   RB0



#ifndef BORV_LO
#define BORV_LO BORV_19
#endif

#define iabs(A)  (A<0 ?  (-A) :  A)


// CONFIG1

#ifdef USE_EXTERNAL_XTAL
    #pragma config FOSC= HS
#else
    #pragma config FOSC = INTOSC // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#endif
#pragma config WDTE = ON // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF // Flash Memory Self-Write Protection (Write protection off)
   
#ifdef USE_EXTERNAL_XTAL
  #pragma config PLLEN = ON // PLL Enable (4x PLL enabled)
#else
  #pragma config PLLEN = OFF // PLL Enable (4x PLL enabled)
#endif
#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)



//Set default value
//  IO0..IO9 config , modbus address
//__EEPROM_DATA(IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT);
  __EEPROM_DATA(IOCONFIG_TIME_CAPTURE_HI,IOCONFIG_OUTPUT,IOCONFIG_OUTPUT,IOCONFIG_TIME_CAPTURE_HI,IOCONFIG_OUTPUT,\
                IOCONFIG_OUTPUT,IOCONFIG_OUTPUT,IOCONFIG_OUTPUT);

__EEPROM_DATA(IOCONFIG_TIME_CAPTURE_HI,IOCONFIG_TIME_CAPTURE_HI,127,0xff,0xff,0xff,0xff,0xff);


unsigned char VRange;  //1 = 1.024V,  2 = 2.048V, 3 = 4.096V else = VDD

unsigned char BadIO; 
SettingStruct Setting;

#define TIMER_100MS  100
near volatile unsigned short Timerms;       //  Interrupt Timer counter in 1 ms
near volatile unsigned short PrimaryTimerms;
near volatile unsigned char TimerDeciSec;    // modbus timer out in 1/10 of sec  in 0.5 ms count
#pragma pack 1


typedef union {
  unsigned short USHORT;
  unsigned char BYTE[2];
}ByteShortUnion;

typedef union {
    unsigned long ULONG;
    unsigned short USHORT[2];
    unsigned  char BYTE[4];
}ByteLongUnion;

//near unsigned char CurrentTimer1H;
//near unsigned char CurrentTimer1L;

// serial buffer
#define SERIAL_BUFFER_SIZE 32
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
near volatile unsigned char RcvInFiFo, RcvOutFiFo;
char SerialBuffer[SERIAL_BUFFER_SIZE];
char RcvSerialBuffer[SERIAL_BUFFER_SIZE];

unsigned char SerialSum;    // use for check sum
unsigned char RcvSerialSum;  // use for check sum verification
bit ModbusOnTransmit;
bit EnableConfigChange;
bit ForceReset;
char ModbusPacketBuffer[SERIAL_BUFFER_SIZE];


const unsigned char     IOMASK[11]={0b00001000,0b00000010,0b00010000,0b01000000,0b10000000,\
                                    0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0};

const unsigned char NOT_IOMASK[11]={0b11110111,0b11111101,0b11101111,0b10111111,0b01111111,\
                                    0b11111110,0b11111101,0b11111011,0b11110111,0b11101111,0b11111111};

//cap sense mask
const unsigned char CSMASK[10]={0b00001001,0b00001011,0b00001000,0b00000101,0b00000110,\
                                0b00000000,0b00000001,0b00000010,0b00000011,0b00000100};

// MODBUS


// CRC16 source code is in CRC16.c file
extern unsigned short CRC16(unsigned char * puchMsg, unsigned char usDataLen);


unsigned char ModbusFunction;
unsigned char ModbusSlave;
unsigned short ModbusAddress;
unsigned short ModbusData;
volatile unsigned short ModbusCRC;
unsigned char ModbusFramePointer;

unsigned char ModbusBuffer[10];

//MODBUS EXCEPTION
#define ILLEGAL_FUNCTION     1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE   3
#define SLAVE_DEVICE_FAILURE 4
#define ACKNOWLEDGE          5
#define SLAVE_DEVICE_BUZY    6
#define NEGATIVE_AKNOWLEDGE  7
#define MEMORY_PARITY_ERROR  8


/* Timer utilisation
Timer0  interrupt timer
Timer1  Sensor timer  utility, 
Timer2  Servo   Utility
Timer4  Hardware PWM
*/





  




void SaveSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
      eeprom_write(idx, *(pointer++));
}








void SetAnalogConfig(unsigned char Pin)
{
// SET ANALOG PIN
 
 unsigned char ioconfig = Setting.IOConfig[Pin];
 unsigned char _tmp= IOMASK[Pin];
 if(Pin<5)
 {
     TRISB |= _tmp;
     ANSELB |= _tmp;
 }
 else
 {
     TRISA |= _tmp;
     ANSELA |= _tmp;
 }

// SET REFERENCE VOLTAGE
   
  FVRCONbits.ADFVR = ioconfig;
 
// VREF or VDD
  if(ioconfig== IOCONFIG_ANALOGVDD)
    ADCON1bits.ADPREF=0;
  else
    ADCON1bits.ADPREF=3;
}





void SetOutputConfig(unsigned char Pin)
{

    unsigned char ioconfig = Setting.IOConfig[Pin];
 unsigned char _tmp= NOT_IOMASK[Pin];
 if(Pin<5)
 {
#asm
     movf  SetOutputConfig@_tmp,w
     movwf ??_SetOutputConfig
     movf  ??_SetOutputConfig,w
     movlb 0
     andwf 13,f
     movlb 1
     andwf 13,f
     movlb 3
     andwf 13,f
#endasm
 //    PORTB &= _tmp;
 //    TRISB &= _tmp;
 //    ANSELB &= _tmp;
 }
 else
 {
#asm
     movf  SetOutputConfig@_tmp,w
     movwf ??_SetOutputConfig
     movf  ??_SetOutputConfig,w
     movlb 0
     andwf 12,f
     movlb 1
     andwf 12,f
     movlb 3
     andwf 12,f
#endasm

//     PORTA &= _tmp;
//     TRISA &= _tmp;
//     ANSELA &= _tmp;
 }
}
/*ovf	SetOutputConfig@_tmp,w
  4553  0016' 0082'              	movwf	??_SetOutputConfig
  4554  0017' 0802'              	movf	??_SetOutputConfig,w
  4555  0018' 0020               	movlb	0	; select bank0
  4556  0019' 058D               	andwf	13,f	;volatile
  4557                           
  4558                           ;main.c: 398: TRISB &= _tmp;
  4559  001A' 0805'              	movf	SetOutputConfig@_tmp,w
  4560  001B' 0082'              	movwf	??_SetOutputConfig
  4561  001C' 0802'              	movf	??_SetOutputConfig,w
  4562  001D' 0021               	movlb	1	; select bank1
  4563  001E' 058D               	andwf	13,f	;volatile
  4564                           
  4565                           ;main.c: 399: ANSELB &= _tmp;
  4566  001F' 0805'              	movf	SetOutputConfig@_tmp,w
  4567  0020' 0082'              	movwf	??_SetOutputConfig
  4568  0021' 0802'              	movf	??_SetOutputConfig,w
  4569  0022' 0023               	movlb	3	; select bank3
  4570  0023' 058D               	andwf	13,f	;volatile

      //PORTB &= ServoMask
#asm
        movf _ServoMask,w
        andwf _PORTB,f
#endasm
*/



void SetPullUp(unsigned char Pin, unsigned char PullUp)
{
    if(Pin<5)
    {
        if(PullUp)
         WPUB |= IOMASK[Pin];
        else
         WPUB &= NOT_IOMASK[Pin];
    }
    else BadIO=1;
 
}

void SetInputConfig(unsigned char Pin)
{
    unsigned char _tmp = IOMASK[Pin];
    unsigned char _ntmp= NOT_IOMASK[Pin];
  if(Pin<5)
  {
    TRISB |= _tmp;
    ANSELB &= _ntmp;
  }
  else
  {
    TRISA |= _tmp;
    ANSELA &= _ntmp;
  }

}


void SetIOConfig(unsigned char Pin)
{
    unsigned char loop;
  ConfigUnion ioconfig;

  const unsigned char ValidCCP[10]={1,0,0,1,0,0,0,0,1,1};
  
  ioconfig.Config = Setting.IOConfig[Pin];
  ResetIOCycle();

  
  
  IOSensorData[Pin].DWORD=0;
  IOSensorData[Pin].WORD[2]=0;


  if(Pin<5)
  {
     IOCounterFlag.Byte & = NOT_IOMASK[Pin];
  }

  if(Pin==0)
  {
      CCP1CON=0;
      CCP1IE=0;
  }
 if(Pin==3)
 {
      CCP2CON=0;
      CCP2IE=0;
 }
  if(Pin==8)
  {
      CCP3CON=0;
      CCP3IE=0;
  }
  if(Pin==9)
  {
      CCP4CON=0;
      CCP4IE=0;
  }

  if(Pin<5)
  {
  SetPullUp(Pin,1);  // By default pull up is there
  SetIOChange(Pin,0);
  }

  if(ioconfig.Config<IOCONFIG_INPUT)
  {
      if(Pin<5)
          SetPullUp(Pin,0);
      SetAnalogConfig(Pin);
  }
  else if(ioconfig.Config==IOCONFIG_OUTPUT)
  {
      SetOutputConfig(Pin);
  }
  else if(ioconfig.Config  == IOCONFIG_TIME_CAPTURE_LO)
  {
      if(ValidCCP[Pin] ==0)
      {
          BadIO=1;
          return;
      }
      SetInputConfig(Pin);
      if(Pin<5)
         SetPullUp(Pin,0);
      SetTimeCaptureConfig(Pin,0);
          
   }  
  else if(ioconfig.Config  == IOCONFIG_TIME_CAPTURE_HI)
  {
      if(ValidCCP[Pin] ==0)
      {
          BadIO=1;
          return;
      }
      SetInputConfig(Pin);
      if(Pin < 5)
         SetPullUp(Pin,0);
      SetTimeCaptureConfig(Pin,1);
  }  
   else if(ioconfig.Config == IOCONFIG_INPUT)
  {
      if(Pin<5)
          SetPullUp(Pin,0);
      SetInputConfig(Pin);
  }

  /*

  else if(ioconfig.Config == IOCONFIG_PWM)
  {
      SetPWMConfig(Pin,0);
  }
  else if(ioconfig.COUNTER)
  {
                            SetPullUp(Pin,0);
                            SetInputConfig(Pin);
                            SetIOChange(Pin,1);
                            if(Pin==0)
                                IOCounterFlag.IO0=1;
                            if(Pin==1)
                                IOCounterFlag.IO1=1;
                            if(Pin==2)
                                IOCounterFlag.IO2=1;
                            if(Pin==3)
                                IOCounterFlag.IO3=1;
                            if(Pin==4)
                                IOCounterFlag.IO4=1;
  }
  */
  else
    SetInputConfig(Pin);
}






unsigned short ReadA2D(unsigned char channel)
{

 ByteShortUnion value;
 ADIE=0;                              // clear interrupt flag
 ADIF=0;
 ADON=1;
 ADCON0bits.ADON=1;
 ADCON0bits.CHS=channel;
 __delay_ms(1);
 ADCON0bits.ADGO=1;
 while(ADCON0bits.ADGO==1);
__delay_ms(1);
 ADCON0bits.ADGO=1;
 while(ADCON0bits.ADGO==1);
 value.BYTE[1]=ADRESH;
 value.BYTE[0]=ADRESL;
 return value.USHORT;
}



static void interrupt isr(void){
    static volatile unsigned char _temp;


// timer 1 use by  Time Capture
    if(TMR1IE)
    if(TMR1IF)
{
     TimerHiCount++;
     TMR1IF=0;
     return;  
}

if(CCP1IE)
  if(CCP1IF)    
  {      
      TimeCapture1Count++;
      IOSensorData[0].BYTE[0]=TimerHiCount >> 8;
      IOSensorData[0].BYTE[1]=TimerHiCount & 0xff;
      IOSensorData[0].BYTE[2]=CCPR1H;              
      IOSensorData[0].BYTE[3]=CCPR1L;     
      IOSensorData[0].BYTE[4]=TimeCapture1Count >> 8;
      IOSensorData[0].BYTE[5]=TimeCapture1Count &0xff;
      CCP1IF=0;
      return;
  }
    
if(CCP2IE)
  if(CCP2IF)        
  {
      TimeCapture2Count++;
      IOSensorData[3].BYTE[0]=TimerHiCount >> 8;
      IOSensorData[3].BYTE[1]=TimerHiCount & 0xff;
      IOSensorData[3].BYTE[2]=CCPR2H;              
      IOSensorData[3].BYTE[3]=CCPR2L;              
      IOSensorData[3].BYTE[4]=TimeCapture2Count >> 8;
      IOSensorData[3].BYTE[5]=TimeCapture2Count &0xff;
      CCP2IF=0;
      return;
   }    
   
if(CCP3IE)
  if(CCP3IF)
  {
      TimeCapture3Count++;
      IOSensorData[8].BYTE[0]=TimerHiCount >> 8;
      IOSensorData[8].BYTE[1]=TimerHiCount & 0xff;
      IOSensorData[8].BYTE[2]=CCPR3H;              
      IOSensorData[8].BYTE[3]=CCPR3L;              
      IOSensorData[8].BYTE[4]=TimeCapture3Count >> 8;
      IOSensorData[8].BYTE[5]=TimeCapture3Count &0xff;
      CCP3IF=0;
      return;
  }
if(CCP4IE)
  if(CCP4IF)        
{
      TimeCapture4Count++;
      IOSensorData[9].BYTE[0]=TimerHiCount >> 8;
      IOSensorData[9].BYTE[1]=TimerHiCount & 0xff;
      IOSensorData[9].BYTE[2]=CCPR4H;              
      IOSensorData[9].BYTE[3]=CCPR4L;              
      IOSensorData[9].BYTE[4]=TimeCapture4Count >> 8;
      IOSensorData[9].BYTE[5]=TimeCapture4Count & 0xff;              
      CCP4IF=0;
      return;
}    
    
// Timer 2 clock system
if(TMR2IF){
 TMR2IF=0;
 if(TimerSecFlag)
 {
     ResetCounterFlag=1;
     //Tell system to Transfer/reset counter
     IOCounterReset.Byte = IOCounterFlag.Byte;

  TimerSecFlag=0;
 }

 Timerms++;
 PrimaryTimerms--;
 if(PrimaryTimerms==0)
  {
     PrimaryTimerms=TIMER_100MS;
    if(WaitForEndDeciSecond)
    {
//        _TMR0=TMR0;
#asm
       movf _TMR0,w
       movwf __TMR0
#endasm

        CPSON=0;
        TMR0IE=0;
        WaitForEndDeciSecond=0;
        GotCapSenseFlag=1;
       
    }
    else if(WaitForStartDeciSecond)
    {
        WaitForStartDeciSecond=0;        
        TMR0=0;
        TMR0IF=0;
        TMR0IE=1;
        WaitForEndDeciSecond=1;
       
    }

     TimerDeciSec--;
     if(TimerDeciSec==0)
     {
//         TimerDeciSec=10;
#asm
         movlw 10
         movwf _TimerDeciSec
#endasm
         TimerSecFlag=1;
     }


  }

  }

    // check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
        TXREG= SerialBuffer[OutFiFo];
//        OutFiFo++;
#asm
       movlb 0 ; select bank0
       incf _OutFiFo
#endasm
       if(OutFiFo >= SERIAL_BUFFER_SIZE)
         OutFiFo=0;
      }
     else
   if(OutFiFo == InFiFo)
     {
       // nothing in buffer  disable tx interrupt
       TXIE=0;
     }
  }



  if(TMR0IE)
  if(TMR0IF)
   {
      TMR0IF=0;
      if(Timer0Overflow)
      {
          _TMR0_MSB++;
/*#asm
          banksel(__TMR0_MSB)
          incf __TMR0_MSB,f
          skipz
          incf __TMR0_MSB+1,f
#endasm
*/
      }
      else
      {
     // got timer0 time out
       TMR0IE=0;
//        CurrentIOCycle=IO_CYCLE_DHT_ANALYZE;
#asm
//       movlw IO_CYCLE_DHT_ANALYZE
       banksel(_CurrentIOCycle)
       movwf _CurrentIOCycle        
#endasm
      }
  }
    // check serial  receive
if(RCIE)
 if(RCIF)
   {
//     RcvSerialBuffer[RcvInFiFo++]= RCREG;
#asm
     banksel(_RcvInFiFo)
     movf _RcvInFiFo,w
     incf _RcvInFiFo,f
     addlw  low (_RcvSerialBuffer)
     movwf 6
     movlw high _RcvSerialBuffer
     movwf 7
     movlb 3  ; select bank 3
     movf 0x19 ,w
     movwf 1
     banksel(_RcvInFiFo)
#endasm
     if(RcvInFiFo == SERIAL_BUFFER_SIZE)
        RcvInFiFo=0;
   }
  
}

void putch(char char_out)
{
   unsigned char temp;

    SerialSum+= (unsigned char) char_out;
// increment circular pointer InFiFo
   temp = (unsigned char) (InFiFo + 1);
   if(temp >= SERIAL_BUFFER_SIZE)
     temp = 0;

//  wait  if buffer full
  while(temp == OutFiFo);

// ok write the buffer
  SerialBuffer[InFiFo]=char_out;
// now tell the interrupt routine we have a new char
InFiFo= temp;

// and enable interrupt
 TXIE=1;
}

void RcvClear(void)
{
    GIE=0;
    RcvInFiFo=0;
    RcvOutFiFo=0;
    GIE=1;
}
unsigned char RcvIsDataIn(void)
 {
     return (RcvInFiFo == RcvOutFiFo ?  0U :  1U);
 }

char RcvGetChar(void)
 {
    char temp;

   // wait until we received something
   while(!RcvIsDataIn());

  // get the character
    temp =  RcvSerialBuffer[RcvOutFiFo];
    RcvOutFiFo++;
    if(RcvOutFiFo >= SERIAL_BUFFER_SIZE)
      RcvOutFiFo=0;

    return temp;
}


void SendModbusPacket(unsigned char BufferSize)
{
    unsigned short CRC;
    unsigned char loop;

    if(ModbusSlave==0) return;

    CRC =  CRC16(ModbusPacketBuffer,BufferSize);
    //RS-485 on TRANSMISSION
    ModbusOnTransmit=1;
    TXM_ENABLE=1;
    // send data
    for(loop=0;loop<BufferSize;loop++)
        putch(ModbusPacketBuffer[loop]);
    // send CRC
    putch(CRC & 0xFF);
    putch(CRC >> 8);


}

void InitModbusPacket(void)
{
    ModbusPacketBuffer[0]=Setting.SlaveAddress;
    ModbusPacketBuffer[1]=ModbusFunction;
}


void  SendFrameError(unsigned char ErrorCode)
{
    InitModbusPacket();
    ModbusPacketBuffer[1]= (unsigned char) (ModbusFunction | 0x80);
    ModbusPacketBuffer[2]= ErrorCode;
    SendModbusPacket(3);
}



/*
void SendReadByteFrame(unsigned  char value)
{

   InitModbusPacket();
   ModbusPacketBuffer[2]= 1;  // byte count
   ModbusPacketBuffer[3]= value;
   SendModbusPacket(4);
 }
*/

void SendReadByteFrame(unsigned  char value)
{

#asm
    movlb 0
    movwf SendReadByteFrame@value
#endasm


   InitModbusPacket();

//   ModbusPacketBuffer[2]= 1;  // byte count
//   ModbusPacketBuffer[3]= value;

#asm
    movlw 1
    banksel(_ModbusPacketBuffer)
    movwf ((_ModbusPacketBuffer&127)+2)
    movlb 0
    movf SendReadByteFrame@value,w
    banksel(_ModbusPacketBuffer)
    movwf ((_ModbusPacketBuffer&127)+3)
#endasm
   SendModbusPacket(4);
 }

/*
void SendReadFrame(unsigned  short value)
{
   InitModbusPacket();
   ModbusPacketBuffer[2]= 2;  // byte count
   ModbusPacketBuffer[3]= value >> 8;
   ModbusPacketBuffer[4]= value & 0xff;
   SendModbusPacket(5);
}
*/


void SendReadFrame(unsigned  short value)
{

#asm
    movlb 0
    movwf SendReadFrame@value
#endasm
   InitModbusPacket();

//   ModbusPacketBuffer[2]= 2;  // byte count
//   ModbusPacketBuffer[3]= value >> 8;
//   ModbusPacketBuffer[4]= value & 0xff;

#asm
   movlw 2
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+2)
   movlb 0
   movf  SendReadFrame@value+1,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+3)
   movlb 0
   movf  SendReadFrame@value,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+4)
#endasm
   SendModbusPacket(5);
}

void SendReadRegistersFrame(ByteLongUnion temp)
{
  if(ModbusData==1)
      SendReadFrame(temp.USHORT[0]);
  else if(ModbusData==2)
  {
      InitModbusPacket();
      ModbusPacketBuffer[2]=4;
      ModbusPacketBuffer[3]=temp.BYTE[3];
      ModbusPacketBuffer[4]=temp.BYTE[2];
      ModbusPacketBuffer[5]=temp.BYTE[1];
      ModbusPacketBuffer[6]=temp.BYTE[0];
      SendModbusPacket(7);
  }
    
    
    
}


void SendBytesFrame(unsigned char _Address)
{
  unsigned char loop=0;
  unsigned char NByte=0;
 
  unsigned char _temp;

  _temp = Setting.IOConfig[_Address];

/*  if(_temp >IOCONFIG_PWM)
  {
      if(_temp<IOCONFIG_DHT11)
          NByte= 4;
      else if(_temp== IOCONFIG_SERVO)
          NByte= 0;
      else
          NByte=6;
  }
*/
  if((_temp & IOCONFIG_TIME_CAPTURE_LO)== IOCONFIG_TIME_CAPTURE_LO)
      NByte = 6;
  else
      NByte=4;

  if(NByte==0)
         SendFrameError(ILLEGAL_DATA_ADDRESS);
    else
     {
       InitModbusPacket();

       ModbusPacketBuffer[2]= NByte;
//  counter use interrupt
//  we need to transfer data with interrupt disable
// use FSR0 = IOSensor[_Address]
// and FSR1 = _modbusPacketBuffer[3]
// IOSensorData =0x1A0
//
//       di()
//              for(loop=0;loop<NByte;loop++)
//        {
//          _temp= tSensor.BYTE[loop];
//          ModbusPacketBuffer[3+loop]=_temp;
//        }
//       ei()

       _temp = _Address * sizeof(SensorDataUnion);
       #asm
       movlw _IOSensorData/256
       movwf FSR0H
       movlw _IOSensorData&0xff
       addwf SendBytesFrame@_temp,w
       movwf FSR0L
       movlw _ModbusPacketBuffer/256
       movwf FSR1H
       movlw (_ModbusPacketBuffer&0xff)+3
       movwf FSR1L
       movf SendBytesFrame@NByte,w
       movwf SendBytesFrame@loop
#endasm
      // ok move NBytes without interrupt
       di();
#asm
BYTELOOP:
        movf  INDF0,w
        movwi FSR1++
        incf FSR0L,f
        decfsz SendBytesFrame@loop,f
        goto BYTELOOP
#endasm
         ei();

       SendModbusPacket((unsigned char) (NByte+3));
     }
}

/*
void SendPresetFrame()
{
   unsigned char buffer[6];

   InitModbusPacket();

   ModbusPacketBuffer[2]= ModbusAddress >> 8;
   ModbusPacketBuffer[3]= ModbusAddress & 0xff;
   ModbusPacketBuffer[4]= ModbusData >> 8;
   ModbusPacketBuffer[5]= ModbusData & 0xff;
   SendModbusPacket(6);
   
}
*/
void SendPresetFrame()
{
      InitModbusPacket();




//   ModbusPacketBuffer[2]= ModbusAddress >> 8;
//   ModbusPacketBuffer[3]= ModbusAddress & 0xff;
//   ModbusPacketBuffer[4]= ModbusData >> 8;
//   ModbusPacketBuffer[5]= ModbusData & 0xff;

   #asm
   banksel(_ModbusAddress)
   movf _ModbusAddress+1,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+2)

   banksel(_ModbusAddress)
   movf _ModbusAddress,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+3)

   banksel(_ModbusData)
   movf _ModbusData+1,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+4)

   banksel(_ModbusData)
   movf _ModbusData,w
   banksel(_ModbusPacketBuffer)
   movwf ((_ModbusPacketBuffer&127)+5)
   #endasm


   SendModbusPacket(6);

}


unsigned char DecodeSerial(char * msg)
{
    int loop;
    unsigned char rcode;
    unsigned short CalcCRC;

    //unsigned char * pt=msg;

 //   unsigned char * pt=0;


//    ModbusSlave= *(pt++);
//    ModbusFunction= *(pt++);
    #define ToUSHORT    ((((unsigned short)*pt) << 8 ) | ((unsigned short) pt[1]));pt+=2;
//    ModbusAddress= ToUSHORT;
//    ModbusData= ToUSHORT;
    // MODBUS CRC have LSB FIRST
//    ModbusCRC= *pt++;
//    ModbusCRC|= ((unsigned short)(*pt)) << 8;


#asm
    movwf 6
    clrf 7
    moviw fsr1++
    movwf _ModbusSlave
    moviw fsr1++
    movwf _ModbusFunction
    moviw fsr1++
    movwf _ModbusAddress+1
    moviw fsr1++
    movwf _ModbusAddress
    moviw fsr1++
    movwf _ModbusData+1
    moviw fsr1++
    movwf _ModbusData
    moviw fsr1++
    movwf _ModbusCRC
    moviw fsr1++
    movwf _ModbusCRC+1
#endasm




    CalcCRC = CRC16(ModbusBuffer,6);

   if(CalcCRC != ModbusCRC) rcode=0;
   else if(ModbusSlave==Setting.SlaveAddress) rcode=1;
   else if(ModbusSlave==0) rcode=1;
   else rcode=2;

   return rcode;

}


/*
unsigned char DecodeSerial(char * msg)
{
    int loop;
    unsigned char rcode;
    unsigned short CalcCRC;

    unsigned char * pt=msg;

    ModbusSlave= *(pt++);
    ModbusFunction= *(pt++);


    #define ToUSHORT    ((((unsigned short)*pt) << 8 ) | ((unsigned short) pt[1]));pt+=2;

    ModbusAddress= ToUSHORT;
    ModbusData= ToUSHORT;

    // MODBUS CRC have LSB FIRST
    ModbusCRC= *pt++;
    ModbusCRC|= ((unsigned short)(*pt)) << 8;


    CalcCRC = CRC16(ModbusBuffer,6);

   if(CalcCRC != ModbusCRC) rcode=0;
   else if(ModbusSlave==Setting.SlaveAddress) rcode=1;
   else rcode=2;

   return rcode;

}

*/


  // Function 3  Read Holding Register
  //
  // Address 0..9: Read R/C servo on IOx if enable
  // Address 160: SlaveAddress (Node ID)
  // Address 0x10n: Read IOn Config
  // Address 250: Version Number
  // Address 251: Software ID NUMBER
  // Address 240: Timer1 CounterHI
  // Address 241: Timer1 CounterLo

void   ReadHoldingRegister()
{
    volatile unsigned char ctemp1,ctemp2,ctemp3;
    ByteLongUnion temp;
    char Flag= 0;
    if((ModbusAddress >= 0x100) && (ModbusAddress <= 0x10a))
      temp.USHORT[0] = Setting.IOConfig[ModbusAddress - 0x100];
   else if(ModbusAddress == 160)
      temp.USHORT[0] = Setting.SlaveAddress;
   else if(ModbusAddress == 250)
      temp.USHORT[0] = RELEASE_VERSION;
   else if(ModbusAddress == 251)
      temp.USHORT[0] = SOFTWARE_ID;
   else if(ModbusAddress == 240)
   {       
       do{
       ctemp1 = TMR1H;
       temp.USHORT[1]=TimerHiCount;
       ctemp2 = TMR1L;
       ctemp3 = TMR1H;
       }while(ctemp1 != ctemp3);
       temp.USHORT[0]= ((unsigned short) ctemp1 << 8) | (unsigned short)ctemp2;
   }
   else if(ModbusAddress == 241)
       {
           temp.USHORT[0] = TimerHiCount;
       }
   else if(ModbusAddress == 242)
   {
       temp.USHORT[0] = Timer1Clock;
   }
   else
      Flag= 1;

    if(Flag)
     SendFrameError( ILLEGAL_DATA_ADDRESS);
    else
     SendReadRegistersFrame(temp);
}


unsigned short ReadIO(unsigned char Pin)
{
  
  BadIO=0;  // clean Bad IO 
  unsigned char ioconfig = Setting.IOConfig[Pin];
  unsigned short temp;
  unsigned char mask;
  unsigned char _tempb;
  // ANALOG MODE
  if(ioconfig <= IOCONFIG_ANALOG4V)
    {
      SetAnalogConfig(Pin);  // set the analog VRef
      return ReadA2D(CSMASK[Pin]);
    } 

  // INPUT  & OUTPUT  MODE
  if(ioconfig <= IOCONFIG_OUTPUT)
  {
      return ReadIOPin(Pin);
  }
  else
      return(0xffff);
  
}

unsigned short ReadVRef()
{
  // A/D INPUT = FVR (2.048V)
  FVRCONbits.ADFVR=2;
  // A/D VREF = VDD
  ADCON1bits.ADPREF=0;
  return ReadA2D(31);
}

unsigned short ReadTSensor()
{
  // A/D VREF = VDD
  FVRCONbits.TSEN=1;
  FVRCONbits.TSRNG=0;
  ADCON1bits.ADPREF=0;
  return ReadA2D(29);
}


unsigned char  MultipleRegister(unsigned char _Address)
{
  if((Setting.IOConfig[_Address] & (IOCONFIG_TIME_CAPTURE_LO)) == IOCONFIG_TIME_CAPTURE_LO)
            return 1;
        return 0;
}


unsigned char GetInputPin(unsigned char thePin)
{
    unsigned char _tmp;

    if(thePin < 5)
       _tmp = PORTB;
    else
       _tmp = PORTA;
    _tmp &= IOMASK[thePin];
    return ( (unsigned char) (_tmp==0 ?0 : 1));
}

unsigned short ReadAllCoils(void)
{
    unsigned short stemp=0;
    unsigned char loop;

    for(loop=0;loop<INPUT_COUNT;loop++)
        {
            stemp *=2;
            stemp |= (unsigned short) GetInputPin( (unsigned char) ((INPUT_COUNT -1)-loop));
        }
    return stemp;
}


  // Function 4  Read Current Register
  //
  // Address 0x1000:  Current VRef 2.048V A/D value
  // Address 0x1001:  Current Build-in Temperature Sensor
  // Address 0xn0:  Read current IOn
void   ReadCurrentRegister()
{
    unsigned short temp;
    unsigned char IOn;

  
       temp=0;
       BadIO=0;
       if(ModbusAddress<0xA0)
       {
           IOn = ModbusAddress >>4;
       
       if(MultipleRegister(IOn))
       {
           SendBytesFrame(IOn);
           return;
       }
       else
           temp= ReadIO(IOn);
       }
       else if(ModbusAddress==0x1000)
           temp = ReadVRef(); // Read 2.048V reference Value
       else if(ModbusAddress==0x1001)
           temp = ReadTSensor(); // Read Build-in Temperature sensor
       else if(ModbusAddress==0x1002)
           temp = ReadAllCoils();
       else
           BadIO=1;
        
       if(BadIO)
         SendFrameError(ILLEGAL_DATA_ADDRESS);
       else
         SendReadFrame(temp);
   
}

/*
void ReadInputStatus()
{
  // Read Coil ou Read Input Status
    unsigned char _tmp;

    
    if(ModbusAddress < INPUT_COUNT)
    {
     if(ModbusAddress < 5)
         _tmp = PORTB;
     else
         _tmp = PORTA;
        _tmp&=IOMASK[ModbusAddress];


      SendReadByteFrame(_tmp==0 ? 0 : 1);

    }
   else
      SendFrameError( ILLEGAL_DATA_ADDRESS);  
}
*/
void ReadInputStatus()
{
  // Read Coil ou Read Input Status

    if(ModbusAddress < INPUT_COUNT)
    {
        SendReadByteFrame(GetInputPin(ModbusAddress));
    }

   else
      SendFrameError( ILLEGAL_DATA_ADDRESS);
}

/*
void ForceSingleCoil()
{
    unsigned char mask;
    if(ModbusAddress < INPUT_COUNT)
    {
       if(Setting.IOConfig[ModbusAddress] == IOCONFIG_OUTPUT)
         {
           mask = IOMASK[ModbusAddress];
           if(ModbusAddress<5)
           {
               if(ModbusData==0)
                   PORTB &= ~mask;
               else
                   PORTB |= mask;

           }
           else
           {
               if(ModbusData==0)
                   PORTA &= ~mask;
               else
                   PORTA |= mask;
           }

           SendPresetFrame();
           return;     
         }
    }
    SendFrameError( ILLEGAL_DATA_ADDRESS);
}
*/


void SetSingleCoil(unsigned char thePin, unsigned char value)
{

    if(Setting.IOConfig[thePin] != IOCONFIG_OUTPUT)
         return;
    WriteIO(thePin,value);
}

void WriteAllCoils()
{
    unsigned char loop;
    unsigned short stemp;

        stemp= ModbusData;
        for(loop=0;loop<INPUT_COUNT;loop++)
        {
            SetSingleCoil(loop,(stemp & 1));
            stemp/=2;
        }
       SendPresetFrame();
}

void ForceSingleCoil()
{
    if(ModbusAddress < INPUT_COUNT)
    {

       SetSingleCoil(ModbusAddress, (unsigned char) ( ModbusData ==0 ? 0 : 1));
       SendPresetFrame();

    }
    else
      SendFrameError( ILLEGAL_DATA_ADDRESS);
}


  // Function 6 Preset Single Register
  //
  // Address 0x10n: IOConfig I0 0..9
  // Address 0x0: Set RCServo0 and clear counter accumulator
  // Address 0x1: Set RCServo1 and clear counter accumulator
  // Address 160: Slave Address
  // Address 0x1ff:  if value is 0x5678 Enable Change in  configuration
  // Address 0xAA55: if value is 0x1234 this mean reset

void PresetSingleRegister()
{
  unsigned char oldConfig;
  unsigned char temp;

  if(ModbusAddress == 0x1002)
  {
      WriteAllCoils();
  }
  else if(ModbusAddress == 0x1ff)
  {
      EnableConfigChange= (ModbusData == 0x5678);
      SendPresetFrame();
  }
  else if(ModbusAddress == 0xAA55)
  {

      if(ModbusData == 0x1234)
      {
        ForceReset=1;
        WDTCON = 0b00010001;
        SendPresetFrame();
        // put watch dog 256ms
     
      }
      else
         SendFrameError(ILLEGAL_DATA_ADDRESS);
  }
  else if((ModbusAddress >=0x100) && (ModbusAddress <= 0x109))
    {
      if(EnableConfigChange)
      {
      temp = ModbusAddress - 0x100;
      BadIO=0;
      oldConfig=Setting.IOConfig[temp];
      Setting.IOConfig[temp]=ModbusData;
      SetIOConfig(temp);
      if(BadIO)
        {
         Setting.IOConfig[temp]=oldConfig;
         SetIOConfig(temp);
         SendFrameError(ILLEGAL_DATA_ADDRESS);
        }
      else
        {
          SendPresetFrame();
          SaveSetting();
        }
      EnableConfigChange=1;
      }
      else
          SendPresetFrame();
    }
    else if(ModbusAddress <10)
    {
        oldConfig= Setting.IOConfig[ModbusAddress];
        if((oldConfig&IOCONFIG_TIME_CAPTURE_LO)==IOCONFIG_TIME_CAPTURE_LO)
        {
            // reset Time Capture LATCH
            IOSensorData[ModbusAddress].BYTE[0]=0;
            IOSensorData[ModbusAddress].BYTE[1]=0;
            IOSensorData[ModbusAddress].BYTE[2]=0;              
            IOSensorData[ModbusAddress].BYTE[3]=0;     
            IOSensorData[ModbusAddress].BYTE[4]=0;
            IOSensorData[ModbusAddress].BYTE[5]=0;
            TimeCapture1Count=0;
            TimeCapture2Count=0;
            TimeCapture3Count=0;
            TimeCapture4Count=0;
            
            SendPresetFrame();
        }
        else
            SendFrameError(ILLEGAL_FUNCTION);
    } 
   else if(ModbusAddress == 160)
    {
       if(EnableConfigChange)
       {
      Setting.SlaveAddress=ModbusData;
      SaveSetting();
       }
      SendPresetFrame();
    }
   else if(ModbusAddress == 240)
   {
       di();
       TMR1ON=0;
       TMR1H=0;
       TMR1L=0;
       TimerHiCount=0;
       TMR1ON=1;
       ei();
       SendPresetFrame();          
   }
   else if(ModbusAddress == 242)
   {
       SetTimer1Clock(ModbusData);
       if(EnableConfigChange)
       {
           Setting.Timer1Clock = ModbusData;
           SaveSetting;
       }
       SendPresetFrame();
   }
    else
       SendFrameError( ILLEGAL_DATA_ADDRESS);
}


void ExecuteCommand(void)
{
  if(ModbusSlave!=0)
   if(ModbusSlave != Setting.SlaveAddress)
      return;    // this is not our Slave Address! just forget it

#if BAUD == 9600
  __delay_us(400);
#else
  __delay_us(100);
#endif
 // if(ModbusLRC != ModbusCheckSum)
 //     return; // invalide check sum we should deal with it

  if(ModbusFunction == 1)
      ReadInputStatus();
  else if(ModbusFunction == 2)
      ReadInputStatus();
  else if(ModbusFunction == 3)
      ReadHoldingRegister();
  else if(ModbusFunction == 4)
      ReadCurrentRegister();
  else if(ModbusFunction == 5)
      ForceSingleCoil();  
  else if(ModbusFunction == 6)
      PresetSingleRegister();
  else
     SendFrameError(ILLEGAL_FUNCTION);
}



 main(void){
     unsigned char loop;
     unsigned char rcode;


#ifndef USE_EXTERNAL_XTAL

 // assume 32Mhz
 OSCCON		= 0b11110000;	// 32MHz  internal clock

#endif


 // assume 32Mhz
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8
 

 ANSELA		= 0;	// NO Analog
 ANSELB         =0;
 PORTA   	= 0b00100000;
 WPUA		= 0b00111111;	// pull-up ON

 


 INTCON		= 0b00000000;	// no interrupt


 // Set Default Watch dog time to 16 sec
 WDTCON = 0b00011101; // bit 0 is ignore
 ForceReset=0;
#asm   
 CLRWDT
#endasm


 // Load Modbus and IO configuration
// EEPROM LOAD AND SAVE SETTING

  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
     *(pointer++) = eeprom_read(idx);

// ok set Timer1Clock from eeprom value
  
  Timer1Clock = Setting.Timer1Clock;
  

 TRISA		= 0b00111111;	// RA0,RA1,RA3,RA5 INPUT , RA2,RA4 OUTPUT
 TRISB          = 0b11011110;
 TXM_ENABLE=0;

 // set serial com with 57600 baud
//alternate pin

 APFCON0 = 0b10000000;
 APFCON1 = 0b00000001;
 
    
 TXSTA = 0b10000010;
 RCSTA = 0;


#if BAUD == 9600
 BRGH =0; //8mhz =>1;
 BRG16 = 1;
 SYNC =0;
 SPBRGL = 207; // assume 32Mhz clock
 SPBRGH =0;
#elif BAUD == 19200
  BRGH =0;
 BRG16 = 1;
 SYNC =0;

 SPBRGL = 103;
 SPBRGH = 0;

#elif BAUD == 38400
 BRGH = 1;
 BRG16=1;
 SYNC =0;
 SPBRG = 208;
#elif BAUD == 115200
 // assume  baud 115200
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


 SPBRGL = 68; // assume 32Mhz clock
 SPBRGH =0;

#else

// assume  baud 57600
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


 SPBRGL = 138; // assume 32Mhz clock
 SPBRGH =0;

#endif


 TXEN =1;   // enable transmitter
 SPEN = 1;  // enable serial port
 CREN = 1;  // enable receiver
 TXIE =0;   // disable transmit interrupt
 RCIF =0;   // clear received flag
 TXIF = 0;
 SCKP = 0;
 ABDEN = 0;
// reset interrupt fifo buffer
 InFiFo=0;
 OutFiFo=0;
 RcvInFiFo=0;
 RcvOutFiFo=0;


 GIE = 1;
 PEIE =1;   // enable peripheral
 RCIE =1;   // Enable received interrupt
 IOCBP =0;
 IOCBN = 0;
 IOCBF = 0;
 IOCIE = 1; // enable interrupt on change


IOCounterFlag.Byte=DHTFlag=0;

ModbusOnTransmit=0;

// Init1msTimer() ;
//void Init1msTimer()
// TIMER2
// 16Mhz clock / 4 =  250 ns clock
// 250ns * 8  = 2us clock
//  1000us / 2us = 250
// assume 32Mhz
T2CON= 0b00000111;
PR2=125;
TMR2=0;
// Enable IRQ
TMR2IF=0;
PrimaryTimerms=100;
TimerDeciSec=10;
TMR2IE=1;
PEIE = 1;
GIE=1;



// init A/D
ADCON1 =  0b11100011;  // fosc/64  32Mhz
ADCON0= 0b00000001; // enable a/d
ANSELA = 0b0000000;      // NO ANALOG
ANSELB = 0b0000000;
ADIE=0;
ADIF=0;
FVRCON=0b11000010;  // Vref internal 2.048V on ADC


 // prepare IO pin
 for(loop=0;loop<INPUT_COUNT;loop++)
 {
    EnableConfigChange=1; // enable IOCONFIG CHANGE
 SetIOConfig(loop);
 }
 EnableConfigChange=0;

 // timer 4 use for pwm
 T4CON = 0b00000111;  // 1:16 timer4 ON
 CCPTMRS= 0b01010101; // all pwm to timer4
 PR4=0xff;
 TMR4=0;
 TMR4IE=0;
 TMR4IF=0;
 
    // clear Modbus system first
 RcvClear();
 ModbusFramePointer=0;
 // five second delay for IO stabilisation
  __delay_ms(5000);
 // cputs("IO Multi 10 V1.0\n\r");

  ResetIOCycle();

  InitTimeCapture();

   
 while(1)
 {
     // clear watch dog
     if(!ForceReset)
     {
     #asm
      CLRWDT
     #endasm
     }
              // No more transmission. put the system back on reception
       // let's
       if(!ModbusOnTransmit)
           TXM_ENABLE=0;
     if(ModbusOnTransmit)
     {
         if(!TXIE)
         {
#if BAUD == 9600
             __delay_us(800);
#else
             __delay_us(200);
#endif
             TXM_ENABLE=0;
             ModbusOnTransmit=0;
         }
     }
     else
     if(RcvIsDataIn())
     {

         ModbusBuffer[ModbusFramePointer++]=RcvGetChar();
         if(ModbusFramePointer>=8)
         {
             ModbusFramePointer=8;
             rcode = DecodeSerial(ModbusBuffer);
          if(rcode==1)
          {
              ExecuteCommand();
             ModbusFramePointer=0;
          }
          else if(rcode ==2)
          {
              // ok not this slave
              ModbusFramePointer=0;
          }
          else
          {
              // something wrong then just shift data
              for(loop=1;loop<8;loop++)
                  ModbusBuffer[loop-1U]=ModbusBuffer[loop];
              ModbusFramePointer--;
          }
         }
     }

     DoIOCycle();

 }

}





