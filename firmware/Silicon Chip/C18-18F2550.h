 // Standard header file for the C18 compiler and PIC18F2550

/* Copyright 2009 Geoff Graham - http://geoffg.net
   This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
   Public License as published by the Free Software Foundation, either version 2 of the License, or (at your
   option) any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.  You should have received a copy of the GNU General Public License along with this program.
   If not, see <http://www.gnu.org/licenses/>. 
   
*/


// standard type definitions
typedef     unsigned char		bit;
typedef     signed char     	sint8;
typedef     signed short int    sint16;
typedef     signed short long   sint24;
typedef     signed long int    	sint32;
typedef     unsigned char   	uint8;
typedef     unsigned short int  uint16;
typedef     unsigned short long uint24;
typedef     unsigned long int  	uint32;


#define forever 1
#define OFF     0
#define ON      1
#define true	1
#define false	0

void uSec(uint8 a) {
	while(a--) {
		Nop();
		Nop();
		Nop();
	}
}

void Delay10TCYx(unsigned char);
void Delay10TCYx(unsigned char);
void Delay10KTCYx(unsigned char);


void uSecX10(uint8 a) {
	while(a--) Delay10TCYx(11);
}

void mSec(uint8 a) {
	while(a--) Delay10TCYx(12); 
}

void mSecX10(uint8 a) {
	while(a--) Delay10KTCYx(12); 
}
