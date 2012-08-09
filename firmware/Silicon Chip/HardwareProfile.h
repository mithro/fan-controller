

/************************************************************************************************
Define the i/o pins
************************************************************************************************/
#define     TRISA_INIT			0b11001111  	// this MUST agree with the #defines below
#define     TEMP_A				PORTAbits.RA0   // input analog - Temperature A
#define     TEMP_B              PORTAbits.RA1   // input analog - Temperature B
#define     TEMP_C				PORTAbits.RA2   // input analog - Temperature C
#define     TEMP_D				PORTAbits.RA3   // input analog - Temperature D
#define     FAN_3				PORTAbits.RA4   // output - Fan 1 voltage control
#define     FAN_4				PORTAbits.RA5   // output - Fan 2 voltage control

#define     TRISB_INIT          0b11111111  	// this MUST agree with the #defines below
                                        		// pullups available (all or nothing)
#define     TACHO_1A			PORTBbits.RB0	// input - Tachometer for fan 1A
#define     TACHO_1B			PORTBbits.RB1	// input - Tachometer for fan 1B
#define     TACHO_2A			PORTBbits.RB2	// input - Tachometer for fan 2A
#define     TACHO_2B			PORTBbits.RB3	// input - Tachometer for fan 2B
#define     TACHO_3A			PORTBbits.RB4	// input - Tachometer for fan 3A
#define     TACHO_3B			PORTBbits.RB5	// input - Tachometer for fan 3B
#define     TACHO_4A			PORTBbits.RB6	// input - Tachometer for fan 4A -also- ICD PGD
#define     TACHO_4B			PORTBbits.RB7	// input - Tachometer for fan 4B -also- ICD PGC

#define     TRISC_INIT          0b00000000  	// this MUST agree with the #defines below
#define     BEEPER				PORTCbits.RC0   // output - USB connected to host
#define     FAN_1_PWM			PORTCbits.RC1   // output - Fan 1 PWM control (CCP2)
#define     FAN_2_PWM           PORTCbits.RC2   // output - Fan 2 PWM control (CCP1)
#define     UNAVAILABLE         PORTCbits.RC3   // Unavailable on 18F2550 and 18F4550
#define     USB_DN              PORTCbits.RC4   // output - USB D-
#define     USB_DP              PORTCbits.RC5   // output - USB D+
#define     FAN_1				PORTCbits.RC6   // output - Fan 3 voltage control
#define     FAN_2				PORTCbits.RC7   // output - Fan 4 voltage control

#define     USB_BUS_SENSE		PORTEbits.RE3   // input - USB connected to host - no TRIS required for this pin

// define debugging signals - not used in production code
#define Mark1               { UNASSIGNED1 = 1; UNASSIGNED1 = 0; }
#define Mark1double         { UNASSIGNED1 = 1; UNASSIGNED1 = 0; UNASSIGNED1 = 1; UNASSIGNED1 = 0; }
#define Mark2               { UNASSIGNED2 = 1; UNASSIGNED2 = 0; }
#define Mark2double         { UNASSIGNED2 = 1; UNASSIGNED2 = 0; UNASSIGNED2 = 1; UNASSIGNED2 = 0; }
#define Mark3               { UNASSIGNED3 = 1; UNASSIGNED3 = 0; }
#define Mark3double         { UNASSIGNED2 = 1; UNASSIGNED2 = 0; UNASSIGNED2 = 1; UNASSIGNED2 = 0; }


#define self_power          1
//#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER


