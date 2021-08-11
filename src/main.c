/********************************************************************************
*                   Fan Controller                                              *
*                                                                               *
*   This program is free software: you can redistribute it and/or modify        *
*   it under the terms of the GNU General Public License as published by        *
*   the Free Software Foundation, either version 3 of the License, or           *
*   (at your option) any later version.                                         *
*                                                                               *
*   This program is distributed in the hope that it will be useful,             *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of              *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
*   GNU General Public License for more details.                                *
*                                                                               *
*   You should have received a copy of the GNU General Public License           *
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.      *
*                                                                               *
*       Date            : Friday, 21 January 2011                               *
*       Author          : C.V.Niras/VU3CNS                                      *
*       Copyright       : (C) 2010 C. V. Niras                                  *
*       Email           : cvniras@gmail.com                                     *
*       Processor       : 12F675                                                *
*       First Release   : 23/01/2011                                            *
*       Ver             : 0.7                                                   *
*   Change History:                                                             *
*       Rev     Date        Description                                         *
*       0.5     06/03/2011  First stable release                                *
*       0.6     08/03/2011  Bug fixed - Can't swich on by CH+, when turned      *
*                           off by set speed to 0                               *
*       0.7     14/06/2011  Trigger off delayed for full speed (low firing      *
*                           angle) to reach the TRIAC latch current             *
*                                                                               *
*********************************************************************************/
#include <pic.h>
#include "type_def.h"
#include "remote_commands.h"

#if defined _12F675
__CONFIG(WDTDIS & MCLRDIS & INTIO & BORDIS & UNPROTECT & PWRTEN);
__IDLOC7('0','.','7','B');
#endif

#if defined _16F676
__CONFIG(WDTDIS & MCLREN & INTOSCIO & BORDIS & UNPROTECT & PWRTEN);
#endif
__EEPROM_DATA(5, 1, 1, 255, 255, 255, 255, 255);

#define TIMER_ENABLE                    // Enable timer
// -- Chip Configurations --
#define _XTAL_FREQ      4000000
#define TRIAC           GPIO5           // Triac pin
#define ZC_PIN_MASK     0x10            // Pin used for zero cross detection
#define IR_PIN_MASK     0x08            // IR sensor output
#define TIME_COUNT      6866            // Number of Timer1 overflows in an hour

#define IsIRDataBitHigh()   GPIO3 == 0  // Inverted logic
#define SW_UP           _GPIO,0         // Up switch
#define SW_DN           _GPIO,2         // Down switch
#define ANY_KEY         Key & 0x80
#define UP_KEY          Key & 0x01
#define DN_KEY          Key & 0x02
#define LEDOn()         GPIO1 = 1       // LED
#define LEDOff()        GPIO1 = 0
#define TriacOn()       GPIO5 = 0       // Triac
#define TriacOff()      GPIO5 = 1

// Timer 1 is the time base, 1 bit time = 8us; the NEC physical bit time is 560us = 70 Timer 1
// The following time are set with a +/- 20% tolerance

#define IR_MARK_MIN_TIME    787
#define IR_SPACE_MIN_TIME   219
#define MIN_IR_BIT_TIME     56
#define MAX_IR_BIT_TIME     84

// -- End of Chip Configurations --

#define IRRx            Flag.b0         // New IR bit received
#define IRNewHit        Flag.b1         // A new IR command received
#define Error           Flag.b2         // General error flag
#define IRCmdRepeat     Flag.b3         // A IR repeat command received
#define StartFan        Flag.b4         // Indicate to start the fan
#define EEPromWrite     Flag.b5         // Indicate to write to EEPROM
#define UnknownCmd      Flag.b6         // Unused IR command

#define EETimeUpdate    Flag1.b0        // Indicate to update time in eeprom
#define IR05Seconds     Flag1.b1        // A 0.5 seconds of IR inactivity, used to clear IR command repeate flag
#define ClearLED        Flag1.b2        // Need to turn off LED

#define FanOn           Status.b0       // Fan Running
#define TimerRunning    Status.b1       // Timer Running

#define AddZeroToIRData()   IRData._dword >>= 1                                 // Add a 0 bit to IR data
#define AddOneToIRData()    IRData._dword >>= 1; IRData._dword |= 0x80000000    // Add a 1 bit to IR data

// NEC IR protocol states
typedef enum _NEC_STATES
{
    IR_IDLE,
    IR_MARK,
    IR_SPACE,
    IR_HIGH,
    IR_LOW,
    IR_REPEAT
} NEC_STATES;

// V A R I A B L E S
//const unsigned char STable[10] = {0, 157, 165, 173, 181, 188, 195, 205, 216, 252};
// Modified to latch low power (load) fans from low firing angle (on full speed)
const unsigned char STable[10] = {0, 157, 165, 173, 181, 188, 195, 205, 216, 234};
near volatile BYTE Flag, Flag1, Status;
near NEC_STATES IRState;
unsigned char IRDataCount, PhaseAngle, Speed, Time, Count;
volatile near unsigned char EECounter, Ticks, Key, KeyCount;
volatile unsigned int IRTime, TimeCounter;
DWORD IRData;

// F U N C T I O N   P R O T O T Y P E S
void IRHandler(void);
void NECDecoder(void);
void InitIR(void);
void SetSpeed(void);
void OffTimer(void);
void OnTimer(void);
void SetTimer(void);
void GetEEVariables(void);
void SetEEVariables(void);
void KeyDelay(void);
void interrupt isr(void);
void Delay2s(void);

// M A I N
void main(void)
{
    OSCCAL = _READ_OSCCAL_DATA();                       // Set oscillator calibration
    GPIO    = 0;                                        // Clear PORT
    TriacOff();                                         // Triac off
    TRISIO  = ZC_PIN_MASK | IR_PIN_MASK | 0x05;         // IR, Zero cross and swiches
    WPU     = 0x05;                                     // Weak pull up enabled for wwitches
    IOC     = ZC_PIN_MASK | IR_PIN_MASK;                // Interrupt on change is enabled for GPIO2, GPIO4
    ANSEL   = 0x00;                                     // All are digital i/o
    CMCON   = 0x07;                                     // Comparators off
    INTCON  = 0x68;                                     // Enable PEIE, Timer0, Port change interrupts
    OPTION  = 0x05;                                     // INT falling edge, Prescaler 1:64
    T1CON   = 0x31;                                     // Prescaler 1:8, internal clk, TMR1ON
    LEDOn();
    Flag._byte = 0;                                     // Initialise flags and variables
    Flag1._byte = 0;
    InitIR();
    GetEEVariables();                                   // Read variables from eeprom
    IRState = IR_IDLE;                                  // Init IR state
    if(FanOn)
        StartFan = 1;
    FanOn = 0;                                          // This created a fresh start
#ifdef  TIMER_ENABLE
    if(TimerRunning)                                    // If already started, set the time again
        SetTimer();
#endif
    GIE = 1;                                            // Enable interrupts
    SetSpeed();                                         // Set the speed
    LEDOff();                                           // Turn off LED
    Count = Time << 1;                                  // Count = Time * 2, i.e turn on/off
    Ticks = 0;
    do
    {
        NECDecoder();                                   // Decode NEC formatted IR data
        if(IRNewHit)                                    // Is any new data?
        {
            IRNewHit = 0;                               // Y. Clear flag
            IRHandler();                                // Do the command
        }
        if(IRCmdRepeat)                                 // As long as the repeat command present
        {                                               // do not turn of LED
            IRCmdRepeat = 0;
            Ticks = 0;                                  // for the same clear Ticks
        }
        if(EEPromWrite)
        {
            EEPromWrite = 0;                            // Clear eeprom update flag
            SetEEVariables();                           // Write variables if necessory
        }
        if(ClearLED)
        {
            if(Ticks & 0x20)                            // If remote command received, turn off LED after only
            {                                           // few ms (~320ms)
                ClearLED = 0;
                LEDOff();
            }
        }
#ifdef  TIMER_ENABLE
        else if(TimerRunning && (Ticks & 0x40))         // If timer is running and a short time expired (640ms)
        {                                               // used to blink LED to show the time remaining
            if(Count == 0xF7)                           // Reload counter to show the time, after a certain time gap
            {
                Count = Time << 1;                      // Count = Time * 2, i.e turn on/off
                LEDOff();                               // Ensure LED is off
            }
            if(!(--Count & 0x80))                       // Blinks the LED till counter become -ve
            {
                if(Count & 0x01) {                      // On odd numbers in counter LED on
                    LEDOn();
                }
                if(!(Count & 0x01)) {                   // For even numbers LED off
                    LEDOff();
                }
            }
            Ticks = 0;                                  // Reset ticks
        }//if(TimerRunning && (Ticks & 0x40))
#endif

        if(KeyCount >= 4)                               // Has key pressed for a short time?
        {
            if(UP_KEY)                                  // Y. Check for Up key press
            {
                StartFan = 1;                           // Y. Up key pressed, inc the speed
                Speed++;
            }
            if(DN_KEY)                                  // Check for a down key press
            {
                StartFan = 1;                           // Y. Down key pressed, decrement the speed
                Speed--;
            }
            if(StartFan)                                // If any key pressed, do the action
            {
                SetSpeed();
                KeyDelay();                             // Give a delay beore the next check
            }
        }
    }while(1);
}// main

// F U N C T I O N S
/**
The function is used to decode IR commands and make changes required
*/
void IRHandler(void)
{
    if(!(IRData.byte0 ^ IRData.byte1) == 0)             // Address and its compliment match
     {
         if(IRData.byte0 == 0)                          // Address  is zero
         {
             if(!(IRData.byte2 ^ IRData.byte3) == 0)    // Command and its compliment match
             {
                UnknownCmd = 0;
                switch(IRData.byte2)
                {
                    case VOL_PLUS:
                    StartFan = 1;
                    Speed++;
                    break;
                    case VOL_MINUS:
                    StartFan = 1;
                    Speed--;
                    break;
                    case DIGIT0:
                    Speed = 0;
                    OffTimer();
                    break;
                    case DIGIT1:
                    StartFan = 1;
                    Speed = 1;
                    break;
                    case DIGIT2:
                    StartFan = 1;
                    Speed = 2;
                    break;
                    case DIGIT3:
                    StartFan = 1;
                    Speed = 3;
                    break;
                    case DIGIT4:
                    StartFan = 1;
                    Speed = 4;
                    break;
                    case DIGIT5:
                    StartFan = 1;
                    Speed = 5;
                    break;
                    case DIGIT6:
                    StartFan = 1;
                    Speed = 6;
                    break;
                    case DIGIT7:
                    StartFan = 1;
                    Speed = 7;
                    break;
                    case DIGIT8:
                    StartFan = 1;
                    Speed = 8;
                    break;
                    case DIGIT9:
                    StartFan = 1;
                    Speed = 9;
                    break;

                    case CH_MINUS:
                    FanOn = 0;
                    OffTimer();
                    break;

                    case CH_PLUS:
                    if(Speed == 0) Speed++;
                    StartFan = 1;
                    break;

                    case PREV:
                    if(FanOn)   Time--;
                    SetTimer();
                    break;

                    case NEXT:
                    if(FanOn)   Time++;
                    SetTimer();
                    break;

                    case EQ:
                    OffTimer();
                    break;

                    case PLAY:
                    OnTimer();
                    break;

                    default:
                    UnknownCmd = 1;
                    break;
                }
                if(!UnknownCmd){
                    LEDOn();                // A new command received
                    ClearLED = 1;           // Clear LED after a little time
                    Ticks = 0;              // Reset timer ticks
                    SetSpeed();
                }
             }// if((IRData.byte0 ^ IRData.byte1) == 0)
         }// if(IRData.byte3 == 0)
     }//if((IRData.byte3 ^ IRData.byte2) == 0)
}
/**
Off the timer, without changing previous set time
*/
void OffTimer(void)
{
    TimerRunning = 0;                                   // Turn off timer
    Time = eeprom_read(1);                              // Read the eeprom, value
    EETimeUpdate = 1;                                   // Update time in the eeprom
    EECounter = 0;                                      // Clear the eeprom counter to update eeprom
}
/**
On timer, start with the previous set time, if the timer is not running
*/
void OnTimer(void)
{
    if(FanOn == 1)                                      // Only start Timer when fan on
    {
        if(!TimerRunning)                               // Nothing to do if already started
        {
            TimerRunning = 1;                           // Start the timer
            Time = eeprom_read(1);                      // Get time from eeprom
            if(Time == 0)                               // If zero, switch on timer with a non zero value, i.e one hour
                Time++;
            TimeCounter = TIME_COUNT;                   // Reload the counter
            EETimeUpdate = 1;                           // Update time in the eeprom
            EECounter = 0;                              // Clear the eeprom counter to update eeprom
        }
    }
}
/**
Set the counters for the timer. When the counter reached zero, the fan will be turned off
If the timer not running, it reload time value with the last saved value from the eeprom,
otherwise adjust counters as required, and clear eeprom counter to update time value in the
eeprom, for the same it set a EETimeUpdate flag
*/
void SetTimer(void)
{
    if(Time & 0x80)                                     // A neg, so clear it
        Time = 0;
    if(Time > 8)
        Time = 8;                                       // Max 8 hours
    if(FanOn == 0)                                      // Nothing to do if fan is not running
        return;
    if(!TimerRunning && (Time != 0)) {                  // If decrementing, and time reached zero, on further
        Time = eeprom_read(1);                          // decrements never start the timer, only start on
        TimerRunning = 1;                               // increments, i.e when time not zero.
        if(Time == 0)                                   // If zero, start with a non zero value
            Time++;                                     // i.e. set to one
    }
    if(Time == 0)                                       // Stop timer, if time is zero
        TimerRunning = 0;
    TimeCounter = TIME_COUNT;                           //
    EETimeUpdate = 1;                                   // Update time in the eeprom
    EECounter = 0;                                      // Clear the eeprom counter to update eeprom
}
/**
Set the speed and update phase angle for a given speed. If start fan flag is set, and fan not running
it starts fan will full speed for a second, then changes to selected speed.
*/
void SetSpeed(void)
{
    if(Speed & 0x80)                                    // A neg, keep it as zero
        Speed = 0;
    if(Speed >= 10)                                     // Reach the max.
        Speed = 9;
    if(Speed == 0){
        StartFan = 0;                                   // For a zero speed, switch off
        FanOn = 0;
    }
    if(StartFan){                                       // If required, try to start the fan
        if(!FanOn){                                     // Already running?
            FanOn = 1;                                  // No. Switch on fan
            PhaseAngle = STable[9];                     // If just swiched on run it on full speed
            Delay2s();                                  // for a short time
        }
        StartFan = 0;                                   // Clear the flag
    }
    if(FanOn == 0) OffTimer();
    PhaseAngle = STable[Speed];                         // Set the phase angle for the speed
    EECounter = 0;                                      // Update EEPROM
}
/**
Decode the IR data received in NEC format
*/
void NECDecoder(void)
{
    static unsigned int IRBitTime, PrevIRTimer;
    #if 0
    static unsigned char Port;
    if((Port ^ GPIO) & IR_PIN_MASK)                     // IR line changed
    {
        *((unsigned char*)&IRTime) = TMR1L;             // Copy the current Timer 1 value (LSB)
        *((unsigned char*)&IRTime + 1) = TMR1H;         // -- do -- (MSB)
        Port = GPIO;
        IRBitTime = IRTime - PrevIRTimer;
        PrevIRTimer = IRTime;
        if(IRBitTime & 0x0800)
            IRState = IR_IDLE;               // Idle for more than 16 ms
        IRRx = 0;
    #else
    if(IRRx)
    {
        IRBitTime = IRTime - PrevIRTimer;
        PrevIRTimer = IRTime;
        if(IRBitTime & 0x0800)
            IRState = IR_IDLE;              // Idle for more than 16 ms
        IRRx = 0;
    #endif
        Error = 0;
        switch(IRState)
        {
            case IR_IDLE:
            if(IsIRDataBitHigh())           // If it is a one, IR starting
                IRState++;                  // Now is IR_MARK
            IRDataCount = 0;
            IRData._dword = 0;
            break;

            case IR_MARK:                   // Now IR Mark just over
            if(IRBitTime < IR_MARK_MIN_TIME)
            {
                Error = 1;                  // Less than specified mark time
            }
            IRState++;
            break;

            case IR_SPACE:                  // Now IR space just over
            if(IRBitTime < IR_SPACE_MIN_TIME)
            {
                Error = 1;                  // Less than specified space time
            }
            if(IRBitTime < IR_SPACE_MIN_TIME * 2)
            {
                IRState = IR_REPEAT;        // If it is less than 4.5 ms, it may be a repeat command
                break;
            }
            IRState++;                      // Space is 4.5 ms, now IR high
            break;

            case IR_HIGH:                   // IR high just over, check it
            if(((unsigned char) IRBitTime < MIN_IR_BIT_TIME) || ((unsigned char) IRBitTime > MAX_IR_BIT_TIME))
            {
                Error = 1;                  // Too short or too long pulse
            }
            IRState++;
            break;

            case IR_LOW:                    // IR low just over, check it
            if(((unsigned char) IRBitTime < MIN_IR_BIT_TIME) || ((unsigned char) IRBitTime > MAX_IR_BIT_TIME * 3))
            {
                Error = 1;                  // Too short or too long pulse
            }
            if((unsigned char) IRBitTime >= MIN_IR_BIT_TIME * 3)
            {
                AddOneToIRData();           // Longer low time, add a one
            }
            else
            {
                AddZeroToIRData();          // Short low time add a zero
            }
            IRDataCount++;                  // Increment the counter
            IRState--;                      // Now is IR High
            break;

            case IR_REPEAT:                 // In repeat, the 560us IR burst just over, check it now
            if(((unsigned char) IRBitTime < MIN_IR_BIT_TIME) || ((unsigned char) IRBitTime > MAX_IR_BIT_TIME))
            {
                IRCmdRepeat = 0;
                Error = 1;
            }
            IRCmdRepeat = 1;
            IRState = IR_IDLE;
            break;

        }// switch(IRState)
        if(Error)
        {
            InitIR();
        }
        if(IRDataCount == 32)
        {
            IRNewHit = 1;
            IRState = IR_IDLE;
            IRDataCount = 0;
            //IROff = 0;
        }
        IR05Seconds = 0;
    }// if(IRRx)
}
/**
Initialise IR states
*/
void InitIR( void ) // initial IR engine
{
    IRState = IR_IDLE;
    IRNewHit = 0;
    IRCmdRepeat = 0;
    IRDataCount = 0;
}
/**
Read variables from eeprom
*/
void GetEEVariables(void)
{
    Speed = eeprom_read(0);
    Time = eeprom_read(1);
    Status._byte = eeprom_read(2);
}
/**
Write to eeprom, the variables
*/
void SetEEVariables(void)
{
    // Compare with the eeprom variables and update if necessory
    if(Speed != eeprom_read(0))
        eeprom_write(0, Speed);
    if(EETimeUpdate)                                    // Time may change while running, so only update
    {                                                   // if user make changes
        if(Time != eeprom_read(1))
            eeprom_write(1, Time);
        EETimeUpdate = 0;
    }
    if(Status._byte != eeprom_read(2))
        eeprom_write(2, Status._byte);
}
/**
Create a 2 seconds delay, used to start the fan
*/
void Delay2s(void)
{
    unsigned char Count;
    Count = 200;                                        // Make a 200 x 10 ms delay
    while(--Count != 0)
        __delay_ms(10);
}
/**
Delay after a key press. This function retuns after a specified time or when no key pressed down
*/
void KeyDelay(void)
{
    unsigned char ms;
    ms = 250;                                           // 500 ms delay
    do
    {
        if(!ANY_KEY)                                    // If key released, return
            break;
        __delay_ms(2);
    }while(--ms != 0);
}

void interrupt isr(void)
{
    static unsigned char PortStatus, PortChanges;
    if(GPIF)
    {
        PortChanges = PortStatus;                       // Copy the previous value
        PortStatus = GPIO;                              // Update PortStatus
        GPIF = 0;                                       // Clear interrrupt
        PortChanges ^= PortStatus;                      // Find the differences
        if(PortChanges & ZC_PIN_MASK)                   // Zero cross
        {
            TMR0 = PhaseAngle;                          // Phase angle is controlled via TMR0, on interrupt it trigger
            if(FanOn && (PhaseAngle >= STable[9])){     // For full speed hold trigger from now, and TMR0 interrupt (delayed
                TriacOn();                              // to reach TRIAC holding current) clear the trigger
            }
            Ticks++;                                    // Ticks @ 10 ms
            {
            #asm
            INCFSZ      _KeyCount,W                     // Inc Key Count, but not beyond 255
            MOVWF       _KeyCount
            CLRW                                        // Clear W
            BTFSS       SW_UP                           // Up Key pressed?
            MOVLW       0x81                            // Y. Load W with 0x81
            BTFSS       SW_DN                           // Down Key pressed?
            MOVLW       0x82                            // Y. Load W with 0x82
            MOVWF       _Key                            // Save W to Key
            BTFSS       _Key,7                          // Has any key pressed?
            CLRF        _KeyCount                       // N. Clear the counter
            #endasm
            }
        }//if(PortChanges & ZC_PIN_MASK)
        if(PortChanges & IR_PIN_MASK)                   // IR line changed
        {
            *((unsigned char*)&IRTime) = TMR1L;         // Copy the current Timer 1 value (LSB)
            *((unsigned char*)&IRTime + 1) = TMR1H;     // -- do -- (MSB)
            IRRx = 1;
        }
    }
    if(T0IF)                                            // Interrupt depends on phase angle (TMR0 is set by phase angle)
    {
        if(FanOn){
            TriacOn();                                  // Triac triggered
        }
        T0IF = 0;                                       // Clear flag
        NOP();
        NOP();
        NOP();
        NOP();
        TriacOff();                                     // Triac off, beacuse already triggered
    }//if(T0IF)

    if(TMR1IF)                                          // @ 524ms; This interrupt is not set, but polled
    {                                                   // with other interrupts
        TMR1IF = 0;
#ifdef  TIMER_ENABLE
        if(TimerRunning && (--TimeCounter == 0))        // If timer running, decrement counter
        {
            TimeCounter = TIME_COUNT;                   // Reload counter
            if(--Time == 0)                             // The time expired, switch off fan and counter
            {
                OffTimer();                             // Switch off timer
                FanOn = 0;
            }
        }
#endif
        {
        #asm
        INCFSZ  _EECounter,W                            // Inc eeprom timer, but not beyond 255
        MOVWF   _EECounter
        #endasm
        }
        if(EECounter == 8)                              // After 4 seconds since clearing the counter
            EEPromWrite = 1;                            // set eeprom write flag
        if(IR05Seconds)                                 // After 0.5 seconds of inactivity
            IRCmdRepeat = 0;                            // clear the repeat command
        IR05Seconds = 1;
    }//if(TMR1IF)
}

