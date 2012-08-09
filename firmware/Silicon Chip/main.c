/*******************************************************************************************************************************
Main.c

Version 1.0   Mar 2010

Go to http://geoffg.net/fancontroller.html for updates, errata and helpful notes
    
    Copyright 2010 Geoff Graham - http://geoffg.net
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
    Public License as published by the Free Software Foundation, either version 2 of the License, or (at your
    option) any later version.

    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.  You should have received a copy of the GNU General Public License along with this program.
    If not, see <http://www.gnu.org/licenses/>.


This is the main source file for the Fan Controller project.

Development Environment
    To compile this you need the following software which is available free of charge from Microchip (www.microchip.com)
     - Microchip MPLAB IDE Version 8.14 or higher
     - Microchip C18 Compiler Lite Version
    You should unzip all the source code files while retaining the directory structure.  Then use the MPLAB
    "Project -> Open.." menu selection to open the file "Fan Controller.mcp".  This will load the complete project
    and its dependent files.

Program summary:
    The program consists of two main sections:
    
    Timer 1 interrupt:
	    This occurs every 400uS.  Within this interrupt a fast loop counts down the pulse width
	    of the pulses used to drive the buck converters.  The length of this loop is about 170uS which is the 
	    maximum width of the output pulse.  Other functions within the timer include:
	     - setting a new second flag every second
	     - counts pulses on the various tachometer inputs
		Overall the interrupt lasts for about 200uS leaving about 200uS before the next interrupt.
    
    Main function:
	    After initialising everything main() enters a high speed loop which runs forever.  Functions
	    within the loop include:
	     - reading the temperatures and calculating the pulse widths
	     - checking the USB stack for data to be sent/received
	    Every second the main function will also:
	     - format the status string and send it off to the USB interface
	     - check for alarms (failed sensors or fans)
	     - count down timers (for example the startup timer and the alarm timer)
    
********************************************************************************************************************************/


/** INCLUDES *******************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include "usb_config.h"
#include "./USB/usb.h"
#include "./USB/usb_device.h"
#include "./USB/usb_function_cdc.h"


#define BC_PERIOD				400									// period of the buck waveform
#define	BC_MAX_PULSE			45									// max pulsewidth of the buck pulse is (BC_MAX_PULSE * 3.8) uSec
#define BC_MAX_PULSE_ONE_FAN	36									// max pulsewidth when only one fan in the pair is connected
																	// this should be approx 80% of BC_MAX_PULSE

#define	FAN_START_TIME			3									// run fans at high speed for 3 seconds after startup
#define	FAN_START_PWR			85									// power to use after startup

#define ALARM_DELAY				5									// an alarm must stay on for more than this seconds before we beep



/** CONFIGURATION **************************************************/
        #pragma config PLLDIV   = 5         // (20 MHz crystal)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      	//USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = OFF
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
        #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF


#include "C18-18F2550.h"
#include "HardwareProfile.h"
#include "EEP.h"
#include "string.h"
#include "ctype.h"


/** V A R I A B L E S ********************************************************/
#pragma udata

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessUSBIO(void);
uint8 CheckHeader(void);
sint8 ReadFanData(uint8 fan, uint8 idx);
void WriteFanData(uint8 fan, uint8 idx, sint8 data);

void USBDeviceTasks(void);
void ISRCode();
void BlinkUSBStatus(void);
void UserInit(void);

/** VECTOR REMAPPING ***********************************************/

#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818

//extern void _startup (void);        // See c018i.c in your C18 compiler dir
//#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
//void _reset (void)
//{
//    _asm goto _startup _endasm
//}

//#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
//void Remapped_High_ISR (void)
//{
//     _asm goto ISRCode _endasm
//}

#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR (void)
{
     _asm goto ISRCode _endasm
}

#pragma code

#define F_DATA_LEN			7
#define F_DATA_START		4
#define F_MINPWR			0
#define F_SENSOR			1
#define F_MINTEMP			2
#define F_MAXTEMP			3
#define F_CANSTOP			4
#define F_ATYPE				5
#define F_BTYPE				6

#pragma romdata eedata_scn=0xf00000
rom sint8 eedata_values[] = {
				// Sensors
				1,			// Temperature sensor A type (0 = not connected, 1 or 2 = connected)
				0,			// Temperature sensor B type (0 = not connected, 1 or 2 = connected)
				0,			// Temperature sensor C type (0 = not connected, 1 or 2 = connected)
				0,			// Temperature sensor D type (0 = not connected, 1 or 2 = connected)
				// Fan 1
				25,			// Minimum power (zero to 100) required to keep the fan spinning
				0,			// Temperature sensor controlling the fan (A = 0, B = 1, etc).
				30,			// Temperature for minimum speed 
				50,			// Temperature for maximum speed
				0,			// Allow fan to be completely stopped.  YES = 1, NO = 0
				1,			// Type of fan A (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				0,			// Type of fan B (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				
				// Fan 2
				25,			// Minimum power (zero to 100) required to keep the fan spinning
				0,			// Temperature sensor controlling the fan (A = 0, B = 1, etc).
				30,			// Temperature for minimum speed 
				50,			// Temperature for maximum speed
				0,			// Allow fan to be completely stopped.  YES = 1, NO = 0
				1,			// Type of fan A (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				0,			// Type of fan B (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				
				// Fan 3
				25,			// Minimum power (zero to 100) required to keep the fan spinning
				0,			// Temperature sensor controlling the fan (A = 0, B = 1, etc).
				30,			// Temperature for minimum speed 
				50,			// Temperature for maximum speed
				0,			// Allow fan to be completely stopped.  YES = 1, NO = 0
				1,			// Type of fan A (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				0,			// Type of fan B (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				
				// Fan 4
				25,			// Minimum power (zero to 100) required to keep the fan spinning
				0,			// Temperature sensor controlling the fan (A = 0, B = 1, etc).
				30,			// Temperature for minimum speed 
				50,			// Temperature for maximum speed
				0,			// Allow fan to be completely stopped.  YES = 1, NO = 0
				1,			// Type of fan A (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
				0			// Type of fan B (0 = not connected, 1 = 2 wire, 2 = 3 wire x1 tacho, ..., 5 = 4 wire)
};			
#pragma romdata


#define USB_BUFF_SIZE  126											// buffer size for USB communications
#define FAULTY_SENSOR  127                                          // magic number indicating a faulty sensor

#pragma udata somevars
sint8 USB_In_Buffer[USB_BUFF_SIZE];
sint8 USB_Out_Buffer[USB_BUFF_SIZE];
#pragma udata othervars
sint8 DATA_In_Buffer[USB_BUFF_SIZE];
//#pragma udata


uint16 TachoDebounce[8];
uint8 TachoCapture[8];
uint8 Tacho[8];
uint8 FanVoltage[4];
uint8 FanCtrl[4];
sint16 Temp[8];
uint8 StartupCount[4];
bit	  NewSecond;
uint8 AlarmTempCount[4];
uint8 AlarmFanCount[8];


#pragma interruptlow ISRCode
void ISRCode() {
	static uint16 SecCnt = (uint16)((uint32)1000000/(uint32)BC_PERIOD);
	uint8 i;

	//if(!((PIE1bits.TMR1IE)&&(PIR1bits.TMR1IF))) return; 			// abort if not the timer 1 interrupt
	
	// reset the timer for the next interrupt
	TMR1H = ((uint16)0 - (uint16)BC_PERIOD * 12) >> 8;
	TMR1L = ((uint16)0 - (uint16)BC_PERIOD * 12) & 0x0f;

	// Time the buck converter pulse width for each fan pair
	// each loop through here takes 3.8uS and the total (for BC_MAX_PULSE = 45) is 170uS
	for(i = 0; i < BC_MAX_PULSE; i++) {
		FAN_1 = (i < FanVoltage[0]);
		FAN_2 = (i < FanVoltage[1]);
		FAN_3 = (i < FanVoltage[2]);
		FAN_4 = (i < FanVoltage[3]);
	}

	// count down to each second and raise a flag indicating that a new second has occurred
	if(--SecCnt == 0) {
		NewSecond = true;
		SecCnt = (uint16)((uint32)1000000/(uint32)BC_PERIOD);
	}	

	// Read the tacho inputs and accumulate the tacho count in Tacho[] (this is reset to zero every second)
	// Debounce the tacho input by only counting a pulse after it is high for 8 times through this code
	for(i = 0; i < 8; i++) {
		if(PORTB & (1 << i)) {
			if(TachoDebounce[i] != 255) TachoDebounce[i]++;			// count up if the input is high
		} else
			TachoDebounce[i] = 0;									// reset the count if the input is low
		if(TachoDebounce[i]++ == 8) TachoCapture[i]++;				// only count a pulse on the 8th time it is high
	}
	
	PIR1bits.TMR1IF = false;
}	//This return will be a "retfie", since this is in a #pragma interruptlow section 





#pragma code

void main(void)
{   
	uint8  Fan;
	uint8 i;
	uint8 p;
	sint16 tmp;
	uint8 AlarmSecondCnt;
	bit   SoundAlarm;

    InitializeSystem();

    while(1)
    {
		// In the idle period between buck pulses we do the rest of the processing such as check the tachos,
		// measure temperatures and calculate the power to each fan.  Total time through here is about 350uS
		// although an unusual event such as sending/receiving USB traffic will extend this considerably.
		

		// Read the temperatures
		for(i = 0; i < 4; i++) {
			UINT16_VAL t;											// declare a temporary variable using a union
			ADCON0 = (i << 2) | 0b00000011;							// select the input and start the conversion
			for(p = 0; p < 255; p++)
				if((ADCON0 & 0b00000010) == 0) break;				// wait for the conversion with a timeout
			t.byte.HB = ADRESH;  t.byte.LB = ADRESL;				// get the reading
			tmp = (t.Val * 5) - ((t.Val * 5)/43);					// convert the reading to mV (assumes a 5V reference)
																	// this method gives the best accuracy using 16 bit integers
			tmp = (tmp - 2730 + 4)/10;								// convert mV to degrees C (the 4 is rounding offset)
			if(tmp < -30 || tmp > 120) 								// check for out of bounds reading
				Temp[i] = FAULTY_SENSOR;							// and if so, flag as faulty
			else
				Temp[i] = ((Temp[i] * 10) + tmp) / 11;				// else average the temperature over the last 10 readings
		}
		
		Temp[4] = Temp[0] - Temp[3];								// calculate the difference temperatures (eg, A - D)
		Temp[5] = Temp[1] - Temp[3];
		Temp[6] = Temp[2] - Temp[3];
		Temp[7] = -10;												// manual temperature control
		
		if(Temp[0] == FAULTY_SENSOR) Temp[4] = FAULTY_SENSOR;       // check for faulty sensors in the difference temperatures
		if(Temp[1] == FAULTY_SENSOR) Temp[5] = FAULTY_SENSOR;
		if(Temp[2] == FAULTY_SENSOR) Temp[6] = FAULTY_SENSOR;
		if(Temp[3] == FAULTY_SENSOR) Temp[4] = Temp[5] = Temp[6] = FAULTY_SENSOR;
		
		// set the speed of each fan pair
		for(Fan = 0; Fan < 4; Fan++) {
			uint8 PreviousFanCtrl = FanCtrl[Fan];					// record the previous fan control percentage
			sint16 TempT = Temp[ReadFanData(Fan, F_SENSOR)];		// Save the current temperature
			if(FanCtrl[Fan] == 0) TempT -= 2;						// subtract a little bit to introduce some hysterisis

			// calculate the fan control in the range of zero to 100
			if(TempT < ReadFanData(Fan, F_MINTEMP)) {				// if temp is less than the minimum temp
				if(ReadFanData(Fan, F_CANSTOP)) {					// if we are allowed to turn off the fan
					FanCtrl[Fan] = 0;								// then turn it off
					StartupCount[Fan] = 0;							// override any startup because we are turning off anyway
				} else
					FanCtrl[Fan] = ReadFanData(Fan, F_MINPWR);		// else set the fan to the minimum
			}
			else
				// calculate the fan speed proportional to temperature
				// Fan Control (zero to 100) = (((Temp - MinTemp) * (100 - MinPower)) / (MaxTemp - MinTemp)) + MinPower
				FanCtrl[Fan] = (((TempT - (sint16)ReadFanData(Fan, F_MINTEMP)) 
								* ((sint16)100 - (sint16)ReadFanData(Fan, F_MINPWR)))/((sint16)ReadFanData(Fan, F_MAXTEMP)
								- (sint16)ReadFanData(Fan, F_MINTEMP))) + (sint16)ReadFanData(Fan, F_MINPWR);
		
			// if we are in the startup phase we overrule the above and set the fan to high speed
			if(PreviousFanCtrl == 0 && FanCtrl[Fan] > 0) StartupCount[Fan] = FAN_START_TIME;
			if(StartupCount[Fan]) FanCtrl[Fan] = FAN_START_PWR;

			if(FanCtrl[Fan] > 100) FanCtrl[Fan] = 100;				// don't let it go above 100%
			
			// if both fans in a pair are not configured we override the above and set the output to zero
			if(ReadFanData(Fan, F_ATYPE) == 0 && ReadFanData(Fan, F_BTYPE) == 0) FanCtrl[Fan] = 0;
			
			// calculate the buck converter pulse length and set it to give the output voltage
			if(ReadFanData(Fan, F_ATYPE) != 0 && ReadFanData(Fan, F_BTYPE) != 0)
				FanVoltage[Fan] = ((uint16)BC_MAX_PULSE * (uint16)FanCtrl[Fan]) / (uint16)100;				// this is for two fans in parallel
			else
				FanVoltage[Fan] = ((uint16)BC_MAX_PULSE_ONE_FAN * (uint16)FanCtrl[Fan]) / (uint16)100;		// and this is for just one fan
				
			// make sure that 100% power results in the output being permanently held high
			if(FanCtrl[Fan] >= 98) FanVoltage[Fan] = BC_MAX_PULSE;
				
			// if either fan is PWM set the voltage to the max
			if(ReadFanData(Fan, F_ATYPE) == 5 || ReadFanData(Fan, F_BTYPE) == 5) FanVoltage[Fan] = BC_MAX_PULSE;
			
			// set the PWM outputs for fans that may be PWM controlled
			if(Fan == 0) 
				if(ReadFanData(0, F_ATYPE) == 5 || ReadFanData(0, F_BTYPE) == 5)
					CCPR2L = FanCtrl[0] + FanCtrl[0]/5;					// set the speed via PWM output #2 (120 is full speed)
				else
					CCPR2L = 120;										// hold the output high if it is not a PWM type fan
			if(Fan == 1) 
				if(ReadFanData(1, F_ATYPE) == 5 || ReadFanData(1, F_BTYPE) == 5)
					CCPR1L = FanCtrl[1] + FanCtrl[1]/5;					// set the speed via PWM output #1 (120 is full speed)
				else
					CCPR1L = 120;										// hold the output high if it is not a PWM type fan
		}	

		ProcessUSBIO();													// do any USB I/O that is pending


		// detect when one second has passed and process accordingly:
		//   - transmit current status to the USB host
		//   - reset the tacho counts
		//   - count down the startup count used to speed up the fans on initial startup
		//   - check for faulty fans or sensors and sound the alarm
		if(NewSecond) {
			NewSecond = false;
			
			// count down the initial startup (high speed) phase for the fans
			for(Fan = 0; Fan < 4; Fan++) if(StartupCount[Fan]) StartupCount[Fan]--;			
			
			// Get the tacho readings and re arrange the array so that it is suitable for use in this code
			Tacho[0] = TachoCapture[6];									// Fan 1A
			Tacho[1] = TachoCapture[7];			
			Tacho[2] = TachoCapture[4];			
			Tacho[3] = TachoCapture[5];			
			Tacho[4] = TachoCapture[2];			
			Tacho[5] = TachoCapture[3];			
			Tacho[6] = TachoCapture[0];			
			Tacho[7] = TachoCapture[1];									// Fan 4B
			
			// reset all tacho counts ready for the next second's worth of counting
			for(Fan = 0; Fan < 8; Fan++) TachoCapture[Fan] = 0;
			
			// send the regular one second update to the host via USB
			p = sprintf(USB_Out_Buffer, " FCD");
			for(i = 0; i < 4; i++) p += sprintf(USB_Out_Buffer + p, ",%0d", Temp[i]);
			for(Fan = 0; Fan < 4; Fan++) p += sprintf(USB_Out_Buffer + p, ",%0d", FanCtrl[Fan]);
			for(Fan = 0; Fan < 8; Fan++) {
				if(ReadFanData(Fan/2, F_ATYPE + (Fan%2)) == 2)
					p += sprintf(USB_Out_Buffer + p, ",%0d", (sint16)Tacho[Fan]*60);
				else if(ReadFanData(Fan/2, F_ATYPE + (Fan%2)) == 4)
					p += sprintf(USB_Out_Buffer + p, ",%0d", (sint16)Tacho[Fan]*15);
				else
					p += sprintf(USB_Out_Buffer + p, ",%0d", (sint16)Tacho[Fan]*30);
			}
			sprintf(USB_Out_Buffer + p, "\r\n");
			
			SoundAlarm = false;										// start with no alarm
			
			// check for faulty temperature sensors
			for(i = 0; i < 4; i++) {
				if(Temp[i] == FAULTY_SENSOR && Read_b_eep(i) != 0) 
					if(AlarmTempCount[i] < ALARM_DELAY)
						AlarmTempCount[i]++;						// wait for ALARM_DELAY seconds
					else
						SoundAlarm = true;							// sound the alarm if failure
				else
					AlarmTempCount[i] = 0;							// otherwise all ok
			}
			
			// check for faulty fans
			for(Fan = 0; Fan < 8; Fan++) {
				if(Tacho[Fan] < 4 && ReadFanData(Fan/2, F_ATYPE + (Fan%2)) >= 2 && FanCtrl[Fan/2] > 0) 
					if(AlarmFanCount[Fan] < ALARM_DELAY)
						AlarmFanCount[Fan]++;						// wait for ALARM_DELAY seconds
					else
						SoundAlarm = true;							// sound the alarm if failure
				else
					AlarmFanCount[Fan] = 0;							// otherwise all ok
			}
				
			// if there is an alarm sound the "beeper" for one second in 60
			if(!SoundAlarm || AlarmSecondCnt++ > 59)
				AlarmSecondCnt = 0;
			BEEPER = (AlarmSecondCnt == 1);
		}
	}
}



static void InitializeSystem(void) {
	
	// configure the i/o ports for input or output
	PORTC = PORTA = 0;												// start out with all outputs low
    TRISA  = TRISA_INIT;
    TRISB  = TRISB_INIT;
    TRISC  = TRISC_INIT;
	INTCON2 = 0b00000000;											// turn on weak pullups on port B and leave interrupts off
	
	// configure the analog to digital converter
    ADCON1 = 0b00001011;                 							// Make A0 to A3 into analog inputs, reference is Vdd/Vss
	ADCON0 = 0b00000001;											// Enable the ADC
	ADCON2 = 0b10010110;											// ADC result right justified, setup 4 Tad, clock Fosc/64
	
	// configure the pulse width modulators to control fans that use PWM
	T2CON  = 0b00000101;											// Timer 2 ON, prescale 1:4 and postscale 1:4
	PR2    = 119;													// set period to generate 25KHz
	CCP1CON= 0b00001111;											// set CCP1 to ON
	CCPR1L = 120;													// set CCP1 to an initial 100% duty cycle
	CCP2CON= 0b00001111;											// set CCP2 to ON
	CCPR2L = 120;													// set CCP2 to an initial 100% duty cycle
	
	StartupCount[0] = FAN_START_TIME;								// set all fans to run fast for a short time after power on
	StartupCount[1] = FAN_START_TIME;
	StartupCount[2] = FAN_START_TIME;
	StartupCount[3] = FAN_START_TIME;
	
	FanCtrl[0] = FanCtrl[1] = FanCtrl[2] = FanCtrl[3] = 0;			// record that currently all fans are not spinning
	
	// check if temp A is shorted to ground and, if so, reset the eeprom to the defaults
	ADCON0 = 0b00000011;											// select temperature A input and start the conversion
	while((ADCON0 & 0b00000010) != 0);								// wait for the conversion
	if(ADRESH == 0 && ADRESL < 100) {								// get the reading and if the input is shorted to ground
		uint8 i;													// reset the eeprom setup values to the defaults
		Write_b_eep(0, 1); Busy_eep();								// first the temperature sensor setup
		Write_b_eep(1, 0); Busy_eep();
		Write_b_eep(2, 0); Busy_eep();
		Write_b_eep(3, 0); Busy_eep();
		for(i = 0; i < 4; i++) {									// then for each fan load standard defaults
			WriteFanData(i, F_MINPWR, 25);
			WriteFanData(i, F_SENSOR, 0);
			WriteFanData(i, F_MINTEMP, 30);
			WriteFanData(i, F_MAXTEMP, 50);
			WriteFanData(i, F_CANSTOP, 0);
			WriteFanData(i, F_ATYPE, 1);
			WriteFanData(i, F_BTYPE, 0);
		}
	}		
	
	FAN_1 = FAN_2 = FAN_3 = FAN_4 = 1;								// set all outputs high in case we hang in USB init
    USBDeviceInit();												// usb_device.c.  Initialise USB module SFRs and firmware
	
	T1CON  = 0b10000001;											// setup timer 1 to count clock cycles (12MHz) with interrupt on overflow
	INTCON = 0b11000000;											// enable peripheral interrupts
	INTCON3= 0;														// disable external interrupts
	PIE1   = 0b00000001;											// enable timer 1 overflow interrupt

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Process all USB related I/O
// First this checks for any incoming data.  This data is saved in DATA_In_Buffer until we get a CR or LF.
// When a complete line is received it is checked for possible commands (ie, FQn or FSn) and the command actioned.
//
// Secondly this routine checks if there is any data in USB_Out_Buffer.  If there is, the contents are sent off to
// the host.  Note that quite often the data has originated as part of actioning a command as listed above.
// The send routine never checks if the USB interface is ready.  There is no point as the data is low value and it
// does not matter if it is accidently lost.
void ProcessUSBIO(void){
	uint8 n, i, j, p;
	static uint8 x = 0;
	
	    USBDeviceTasks(); 											// Check bus status and service USB interrupts.
	    
	    // First check if any data has arrived
		n = getsUSBUSART(USB_In_Buffer, USB_BUFF_SIZE);
		if(n) {														// there is data waiting to be read
			if(n + x >= USB_BUFF_SIZE) x = 0;
			strncpy(DATA_In_Buffer + x, USB_In_Buffer, n);			// copy the data into our buffer
			x += n;													// adjust the pointer for the next lot of data
			DATA_In_Buffer[x] = 0;									// terminate the string
			
			// check if we have a line terminated in CR or LF
			if(strchr(DATA_In_Buffer, '\r') != NULL || strchr(DATA_In_Buffer, '\n') != NULL) {
				i = CheckHeader();									// YES!  Interpret the command first
				if(i == 0) {										// FCQ - list configuration parameters in eeprom	
					p = sprintf(USB_Out_Buffer, " FCR", i+1);	// fill the output buffer with the data to be sent
					for(j = 0; j < F_DATA_START + (F_DATA_LEN * 4); j++) p += sprintf(USB_Out_Buffer + p, ",%0d", (sint8)Read_b_eep(j));
					sprintf(USB_Out_Buffer + p, "\r\n");
				} else if(i == 1) { 								// FCS - Set the parameters
					for(j = 0; j < F_DATA_START + (F_DATA_LEN * 4); j++) {	// for each item of data to be read
						while(DATA_In_Buffer[p++] != ',');			// step over the comma
						Write_b_eep(j, atoi(DATA_In_Buffer + p));	// write the data to EEPROM
						Busy_eep();									// and wait for the write to finish
					}	
					sprintf(USB_Out_Buffer, " FCA\r\n");
				} else
					sprintf(USB_Out_Buffer, " \r\nERR: %s\r\n", DATA_In_Buffer);
				
				x = 0; DATA_In_Buffer[0] = 0;						// clear the input buffer as we have finished with the data
			}	
		}		
		
		// If the output buffer has some data send it off to the host
		if(mUSBUSARTIsTxTrfReady() && USB_Out_Buffer[0] != 0) {
			putUSBUSART(USB_Out_Buffer, strlen(USB_Out_Buffer));
			USB_Out_Buffer[0] = 0;
		}	
		
		CDCTxService();												// Process any data for the USB interface
}



		
// Check the data in the command sent to the fan controller
// the data is assumed to be in DATA_In_Buffer[]
// Return 0 for query (FCQ)
//        1 for setting (FCS) the parameters
//		  99 for error (ie, invalid command or invalid characters)
uint8 CheckHeader(void) {
	uint8 p, j;
	
	// then check the command header
	if(toupper(DATA_In_Buffer[0]) != 'F') return 99;
	if(toupper(DATA_In_Buffer[1]) != 'C') return 99;
	if(toupper(DATA_In_Buffer[2]) == 'Q') return 0;
	
	// first check the whole buffer for invalid characters
	p = 3;															// start of the buffer after the command letters
	j = 0;
	while(DATA_In_Buffer[p] != '\r' && DATA_In_Buffer[p] != '\n') {
		if(DATA_In_Buffer[p] == ',') {
			j++;													// count the the commas
		} else {
			if(!(DATA_In_Buffer[p] == ' ' || (DATA_In_Buffer[p] >= '0' && DATA_In_Buffer[p] <= '9')))
				return 99;											// error if not numeric or space
		}
		p++;		
	}
	if(j != F_DATA_START + (F_DATA_LEN * 4)) return 99;				// incorrect number of commas
	
	if(toupper(DATA_In_Buffer[2]) == 'S') return 1;
	return 99;	
}	



// get fan data from the eeprom
// Arg 1 is the fan number (starting from zero)
// Arg 2 is the item of data (ie, F_SENSOR)
sint8 ReadFanData(uint8 fan, uint8 idx) {
	return (sint8)Read_b_eep((fan*F_DATA_LEN) + F_DATA_START + idx);
}	



// write fan data to the eeprom
// Arg 1 is the fan number (starting from zero)
// Arg 2 is the item of data (ie, F_SENSOR)
// Arg 3 is the data
void WriteFanData(uint8 fan, uint8 idx, sint8 data) {
	Write_b_eep((fan*F_DATA_LEN) + F_DATA_START + idx, data);
	Busy_eep();
}	
