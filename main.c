/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 1995-2006 Ake Hedman, eurosource
 *  Copyright (C) 2014 David Steeman, www.steeman.be
 *
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
 *
 * This software is part of the VSCP node 'Mespelare'
 * Based on original VSCP source code of 'Paris' copyright by Ake Hedman
 * 
 * ******************************************************************************
*/

#define MESPELARE_V9
#define PERMANENT_NICK  //This sets a permanent nickname irrispective of what is in EEPROM

#include <p18cxxx.h>
#include <timers.h>
#include "/common/eeprom.h"         //copied common files to local common directory since checking out whole SVN is huge
#include "/common/can18xx8.h"       //copied common files to local
#include "can18F66K80.h"
#include "/common/vscp_firmware.h"  //copied common files to local
#include "/common/vscp_class.h"     //copied common files to local
#include "/common/vscp_type.h"      //copied common files to local

#ifdef MESPELARE_V6
#include "VSCP_node_defines_06.h"
#endif
#ifdef MESPELARE_V9
#include "VSCP_node_defines_09.h"
#endif

#include "main.h" 
#include <delays.h>   //david xxx Needed for Delay10KTCYx() function to make buzzer beep at startup and OneWire

/* --- configure DS1820 temparture sensor --- */
//#include "types.h"
//#include "ds1820.h"
//#include "ow.h"
//#include "ds18s20.h"


#define DEBUG         //david: debugging mode on
#define SERIAL_DEBUG //david: enable debugging via serial port (9k6 8n1 no flow)
//#define SERIAL_DEBUG_DUMPEEPROM //david: enable DumpEEPROM() function, dumps EEPROM contents to USART
//#define SERIAL_DEBUG_RW_APP_REG //david: enable vscp_readAppReg() and vscp_writeAppReg() debugging
#define SERIAL_DEBUG_DM  //david: enable debug info for DoDM()
#define SERIAL_DEBUG_DM_ACTIONS
//#define SERIAL_DEBUG_INPUTS  //david: enable debug info on input status changes


#ifdef SERIAL_DEBUG
    #include <usart.h> // Include function definitions for the USART library
    #include "C:\Program Files\Microchip\mplabc18\v3.47\h\daf_usart.c" //David's own USART routines
    #include <stdio.h>
#endif

#if defined(__18F25K80) || defined(__18F26K80) || defined(__18F45K80) || defined(__18F46K80) || defined(__18F65K80) || defined(__18F66K80)
#pragma config FOSC = HS1 //INTIO1 //HS1
#pragma config PLLCFG = ON
#pragma config SOSCSEL = DIG
#pragma config PWRTEN = ON
#pragma config STVREN = ON
#pragma config CPB = OFF
#pragma config CANMX = PORTB
#pragma config XINST = OFF
#if defined(RELEASE)
#pragma config WDTEN = ON
#pragma config BOREN = ON
#pragma config BORV = 3
#else
#pragma config WDTEN = OFF
#pragma config BOREN = OFF
#pragma config BORV = 3
#endif
#endif


void FillEEPROM(void);
void DumpEEPROM(void);
void DumpDM(void);


// Startup code from c018.c
void _startup (void);

// ISR Routines
void isr_low( void );
void isr_high (void);

// The device URL (max 32 characters including null termination)
const far rom uint8_t vscp_deviceURL[] = "192.168.1.245/mesp09.xml";

volatile unsigned long measurement_clock;	// Clock for measurments
unsigned short tick_10ms;			// Clock for debouncing
BYTE seconds;					// counter for seconds
BYTE minutes;					// counter for minutes
BYTE hours;					// Counter for hours

unsigned short relay_protection_timer[8];
unsigned short PWM_value[4];
unsigned short PWM_state[4];

uint16_t blinkenlights;
uint8_t counter;
uint16_t Timer1Reload = 45000;

unsigned short INP_state[8];
unsigned short INP_hold_timer[8];

static uint8_t BufferINP1 = 0;
static uint8_t BufferINP2 = 0;
static uint8_t BufferINP3 = 0;
static uint8_t BufferINP4 = 0;
static uint8_t BufferINP5 = 0;
static uint8_t BufferINP6 = 0;
static uint8_t BufferINP7 = 0;
static uint8_t BufferINP8 = 0;

unsigned short PlayingSong;
unsigned short NoteCountdown;


#ifdef SERIAL_DEBUG
    char debugstring[80];
#endif


///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- Services Timer0 Overflow
//////////////////////////////////////////////////////////////////////////////

#ifdef RELOCATE
#pragma code low_vector = 0x818
#else
#pragma code low_vector = 0x18
#endif

void interrupt_at_low_vector( void ) {
	_asm GOTO isr_low _endasm 
}

#pragma code
 
#pragma interruptlow isr_low
void isr_low( void )                // called every 10ms
{
	BYTE temp;
	unsigned short tval0, tval1;
	char *p;	
	
	// Clock
	//if ( PIR1bits.TMR2IF ) {	// If a Timer2 Interrupt, Then...
	if ( INTCONbits.TMR0IF ) {	// If a Timer0 overflow Interrupt, Then...

		vscp_timer++;
		measurement_clock++;
		tick_10ms++;

              //  #if defined (debug)
                // if (blinkenlights++==1000) {
                 //    blinkenlights = 0;
                // }
                //#endif

		// Check for init INP
		if ( !INIT_BUTTON ) {
                    vscp_initbtncnt++;
                    if (vscp_initbtncnt>100) vscp_initbtncnt = 101;
		}
		//else {
			// Active
		//	vscp_initbtncnt++;
		//}

		// Status LED
		vscp_statuscnt++;
		if ( ( VSCP_LED_BLINK1 == vscp_initledfunc ) && ( vscp_statuscnt > 100 ) ) {

			if ( INIT_LED ) {
				INIT_LED = 0; 
			}
			else {
				INIT_LED = 1; 
			}	

			vscp_statuscnt = 0;

		}
		else if ( VSCP_LED_ON == vscp_initledfunc ) {
			INIT_LED = 0;
			vscp_statuscnt = 0;	
		}
		else if ( VSCP_LED_OFF == vscp_initledfunc ) {
			INIT_LED = 1;
			vscp_statuscnt = 0;
		}

		//PIR1bits.TMR2IF = 0;     // Clear Timer2 Interrupt Flag
                Timer1Reload = Timer1Reload + 10;
                if (Timer1Reload >= 64000) Timer1Reload = 45000;
        
                WriteTimer0(Timer0ReloadValue);
                INTCONbits.TMR0IF = 0; //clear overflow IRQ

	}
         
/*	
	// CAN error
	if ( PIR3bits.ERRIF ) {
		
		temp = COMSTAT;
		PIR3 = 0;
		
	}
*/	
	return;
}

///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine (high)
//      	- Services Timer1 Overflow for sound generation
//////////////////////////////////////////////////////////////////////////////

#ifdef RELOCATE
#pragma code high_vector = 0x808
#else
#pragma code high_vector = 0x08
#endif


void interrupt_at_high_vector(void) {
_asm GOTO isr_high _endasm
}

#pragma code /* return to the default code section */

#pragma interrupt isr_high
void isr_high (void) {
    if (PIR1bits.TMR1IF) {
    LATCbits.LATC0 = !LATCbits.LATC0; //temporary to test audio generation  // David: C0 = OUT?
    WriteTimer1(Timer1Reload);
    PIR1bits.TMR1IF = 0;
    }
}


void SoundBuzzer(void) { // david
        BUZZER = 1;
        Delay10KTCYx(500);
        BUZZER = 0;
        Delay10KTCYx(100);
}

void ToggleOutputs(void) {
    unsigned int delay=100;

    OUT1 = 1; Delay10KTCYx(delay);
    OUT2 = 1; Delay10KTCYx(delay);
    OUT3 = 1; Delay10KTCYx(delay);
    OUT4 = 1; Delay10KTCYx(delay);
    OUT5 = 1; Delay10KTCYx(delay);
    OUT6 = 1; Delay10KTCYx(delay);
    OUT7 = 1; Delay10KTCYx(delay);
    OUT8 = 1; Delay10KTCYx(delay);
    OUT9 = 1; Delay10KTCYx(delay);
    OUT10 = 1; Delay10KTCYx(delay);
    OUT11 = 1; Delay10KTCYx(delay);
    OUT12 = 1; Delay10KTCYx(delay);
    OUT13 = 1; Delay10KTCYx(delay);
    OUT14 = 1;
    Delay10KTCYx(500);
    OUT1 = 0; Delay10KTCYx(delay);
    OUT2 = 0; Delay10KTCYx(delay);
    OUT3 = 0; Delay10KTCYx(delay);
    OUT4 = 0; Delay10KTCYx(delay);
    OUT5 = 0; Delay10KTCYx(delay);
    OUT6 = 0; Delay10KTCYx(delay);
    OUT7 = 0; Delay10KTCYx(delay);
    OUT8 = 0; Delay10KTCYx(delay);
    OUT9 = 0; Delay10KTCYx(delay);
    OUT10 = 0; Delay10KTCYx(delay);
    OUT11 = 0; Delay10KTCYx(delay);
    OUT12 = 0; Delay10KTCYx(delay);
    OUT13 = 0; Delay10KTCYx(delay);
    OUT14 = 0;


}


//***************************************************************************
// Main() - Main Routine
//***************************************************************************
void main()
{
	unsigned char a;
	unsigned int i;
	unsigned char keyval;
	BOOL bOn = FALSE;
	unsigned char shortpress;

	init();                 // Initialize Microcontroller

	vscp_init();		// Initialize the VSCP functionality

        SoundBuzzer();          // david xxx Sound the buzzer at startup

//      FillEEPROM();
        
#ifdef SERIAL_DEBUG
        sprintf(debugstring, "\r\n\r\n*****START MAIN*****\r\nvscp_deviceURL[] = ");
        daf_puts1USART(debugstring);
        daf_putrs1USART(vscp_deviceURL);
        daf_putrs1USART("\r\n");
#endif

	while ( 1 ) {           // Loop Forever	
		ClrWdt();			// Feed the dog
                // Updated init INP code

                if (INIT_BUTTON) {  // button released
#ifdef SERIAL_DEBUG_DUMPEEPROM
                    if (( vscp_initbtncnt > 51 ) ) {           //if INIT_BUTTON was held (and released)
                        daf_putrs1USART("\r\nINIT_BUTTON held, calling DumpEEPROM()");
                        ToggleOutputs();        // toggle outputs on and off again
//                        DumpDM();               // dump DM EEPROM contents to USART
//                        DumpEEPROM();           // dump EEPROM contents to USART
                    }
#endif
                    if (( vscp_initbtncnt > 100 ) && ( VSCP_STATE_INIT != vscp_node_state ) ) {
                        vscp_nickname = VSCP_ADDRESS_FREE;
			writeEEPROM( VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE );
      			vscp_init();
                    }
                    if (( vscp_initbtncnt > 51 ) && ( VSCP_STATE_ACTIVE == vscp_node_state ) ) {
//			SendProbeAck(vscp_initbtncnt); //TODO: remove //, commented out not to interfere with DumpEEPROM() above
                    }
                    vscp_initbtncnt = 0;
		}

		// Check for a valid  event
		vscp_imsg.flags = 0;
		vscp_getEvent();

		// do a meaurement if needed
		if ( measurement_clock > 100 ) {
			
			measurement_clock = 0;	
			// Do VSCP one second jobs 
			vscp_doOneSecondWork();

			// Temperature report timers are only updated if in active 
			// state guid_reset
			if ( VSCP_STATE_ACTIVE == vscp_node_state  ) {
					
				// Do application one second jobs
			    doApplicationOneSecondWork();	
			}				
		}	
		
		if ( seconds > 59 ) {
        	        seconds = 0;
			minutes++;
				
			if ( minutes > 59 ) {
			    minutes = 0;
				hours++;
			}
				
			if ( hours > 23 ) hours = 0;
		}

		switch ( vscp_node_state ) {

			case VSCP_STATE_STARTUP:			// Cold/warm reset

				// Get nickname from EEPROM
				if ( VSCP_ADDRESS_FREE == vscp_nickname ) {
					// new on segment need a nickname
					vscp_node_state = VSCP_STATE_INIT; 	
				}
				else {
					// been here before - go on
					vscp_node_state = VSCP_STATE_ACTIVE;
					vscp_goActiveState();
				}
				break;

			case VSCP_STATE_INIT:			// Assigning nickname
				vscp_handleProbeState();
				break;

			case VSCP_STATE_PREACTIVE:		// Waiting for host initialisation
				vscp_goActiveState();					
				break;

			case VSCP_STATE_ACTIVE:			// The normal state
				
				if ( vscp_imsg.flags & VSCP_VALID_MSG ) {	// incoming message?
					
					vscp_handleProtocolEvent();
					doDM();		
				}
				break;

			case VSCP_STATE_ERROR:			// Everything is *very* *very* bad.
				vscp_error(); 
				break;

			default:					// Should not be here...
				vscp_node_state = VSCP_STATE_STARTUP;
				break;

		} 
                
		doWork();

	} // while
}

void DumpDM(void) {
#ifdef SERIAL_DEBUG
    unsigned int i;
    
    daf_putrs1USART("\r\nDumpDM() reading from EEPROM");
    for (i=0; i< DESCISION_MATRIX_ELEMENTS; i++) {
        sprintf(debugstring,
            "\r\nRow=%1i, oaddr=0x%02x, dmflags=0x%02x, class_mask=0x%02x, ",
            i,
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_OADDR),
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_FLAGS),
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_CLASSMASK)
        );
        daf_puts1USART(debugstring);
        sprintf(debugstring,
            "class_filter=0x%02x, type_mask=0x%02x, type_filter=0x%02x, ",
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_CLASSFILTER),
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_TYPEMASK),
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_TYPEFILTER)
        );
        daf_puts1USART(debugstring);
        sprintf(debugstring,
            " action=0x%02x, action_param=0x%02x",
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_ACTION),
            readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_ACTIONPARAM)
        );
        daf_puts1USART(debugstring);
    }
#endif
}


///////////////////////////////////////////////////////////////////////////////
// Init - Initialization Routine
//  

void init()
{
	BYTE msgdata[ 8 ];
        uint8_t i;

	// Initialize data
        OSCCON = 0b01110000;
        OSCTUNE = 0b01000000;
        init_app_ram();

	// Initialize the TRIS registers. 0 is output, 1 is input
        //   pins 76543210
#ifdef MESPELARE_V6
        TRISA = 0b00000000;                                                     // Hasselt: 0b11000000;
        TRISB = 0b11111011;                                                     // Hasselt: 0b11001100;
        TRISC = 0b01000000;                                                     // Hasselt: not set
        TRISD = 0b10000000;                                                     // Hasselt: 0b11111110;
        TRISE = 0b00000000;                                                     // Hasselt: not set
#endif
#ifdef MESPELARE_V9
        TRISA = 0b00000000;
        TRISB = 0b01111011;
        TRISC = 0b11000000;
        TRISD = 0b10000000;
        TRISE = 0b00000000;
#endif

	INIT_LED = 1; // Status LED on
	BUZZER = 0;   // Buzzer is off

	// Timer 0 used as 10ms clock tick
	OpenTimer0( TIMER_INT_ON &
		    T0_16BIT &
		    T0_PS_1_2 &
		    T0_SOURCE_INT );
        
        WriteTimer0(Timer0ReloadValue);
	// Timer 1 used as sound generator using high interrupt
	WriteTimer1(61000);
        /*OpenTimer1( TIMER_INT_ON &
					T1_16BIT_RW &
					T1_SOURCE_FOSC &
					T1_PS_1_1,0 );
        */
        WriteTimer1(61000);

#ifdef SERIAL_DEBUG  //david: enable serial debugging
    // baud rate calculator: http://dkitsch.com/pic-spbrg-baud-rate-calculator/
    TRISCbits.TRISC6 = 0; //TX pin set as output

    Open1USART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & 
//               USART_BRGH_LOW, 51); //9600 baud - error 0,16% - works
//               USART_BRGH_LOW, 16);  //28800 baud - error 2.12% - works
//               USART_BRGH_LOW, 12);  //38400 baud - error 0.16% - works
               USART_BRGH_LOW, 8);  //57600 baud - error 3.55%- works
//               USART_BRGH_LOW, 3);  //115200 baud - error 8.5% - frame error too high?
#endif
        
        RCONbits.IPEN = 1;              //enable prioritized interrupts
        INTCONbits.GIEH = 1;            //enable high priority interrupts
        INTCONbits.GIEL = 1;            //enable low priority interrupts
        INTCON2bits.TMR0IP = 0;         //TMR0 overflow = low interrupt
        IPR1bits.TMR1IP = 1;            //TMR1 overflow = high interrupt
        
        //IPR1bits.TMR2IP = 1;          //assign low priority to TMR2 interrupt
        //PIE1bits.TMR2IE=1;            //enable Timer2 interrupt
        //INTCONbits.PEIE = 1;          // Enable peripheral interrupt
	//INTCONbits.GIE = 1;           // Enable global interrupt

        // Initialize timer 4 for the PWM module
        OpenTimer4( TIMER_INT_OFF & T4_PS_1_16 & T4_POST_1_16);
        PR4 = 0xFF; //Timer 4 period register
        CCPTMRS = 0b00010001; // select PWM to run off timer 4
        if (readEEPROM( VSCP_EEPROM_END + REG_MODE_OUT3) == 0x01) CCP5CON = 0b0000000; //check if the mode for the output is set to output or PWM
        else CCP5CON = 0b00001100; //enable CCP5 for PWM output 3 (RB5) (PWM2)
        if (readEEPROM( VSCP_EEPROM_END + REG_MODE_OUT4) == 0x01) CCP1CON = 0b0000000; //check if the mode for the output is set to output or PWM
        else CCP1CON = 0b00001100; //enable CCP1 for PWM output 4 (RB4) (PWM1)

	vscp18f_init( TRUE );
	
	// Initialize all relays in off pos
	/* // Done in init_app_eeprom routine
        for ( i = 1; i<=8 ; i++) {
            SetOut(i,0);
	}
         */
        // Initialize PWM outputs to zero
        PWM1 = 0xff;
	PWM2 = 0xff;

        // set multiplexed I/O / AD pins to I/O mode
        ANCON0 = 0;                     //set all pins to digital I/O
        ANCON1 = 0;                     //set all pins to digital I/O

	// Enable peripheral interrupt	
	
	
	// EnableCAN Receive Error Interrupt
	//PIE3bits.IRXIE = 1;

	return;
}

///////////////////////////////////////////////////////////////////////////////
// init_app_ram
//

void init_app_ram( void )
{
	unsigned int i;
	
	measurement_clock = 0;	// start a new mesurement cycle
	
	for ( i = 0; i<8 ; i++) {
		INP_hold_timer[i] = 0;
		INP_state[i] = INP_RELEASED;
                SetOut(i+1,0); // set outputs to default.
	}
	
	seconds = 0;
	minutes = 0;
	hours = 0;
	
}
 

///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom( void )
{
	unsigned char i,j;
	
        for ( i=0 ; i<8; i++){
            writeEEPROM( VSCP_EEPROM_END + REG_STATUS_OUT1 + i, 0);  //Set all output registers to off
            SetOut(i+1,0); // Actually set the outputs to off
            writeEEPROM( VSCP_EEPROM_END + REG_SUBZONE_OUT1 + i, 0); //set all subzones to zero
            writeEEPROM( VSCP_EEPROM_END + REG_CONTROL_OUT1 + i, RELAY_CONTROLBIT_ONEVENT | RELAY_CONTROLBIT_OFFEVENT | RELAY_CONTROLBIT_ENABLED); //enable all outputs & have them send on/off event. No protection timer as most likely the outputs are indicator leds.
            writeEEPROM( VSCP_EEPROM_END + REG_PTIME_OUT1 + i, 1); //set default protection timer to 64s
        }
         
        writeEEPROM( VSCP_EEPROM_END + REG_MODE_OUT3, 1); //set default mode of OUT3 to output
        writeEEPROM( VSCP_EEPROM_END + REG_MODE_OUT4, 1); //set default mode of OUT4 to output

	// * * * Decision Matrix * * *
	// All elements disabled.
	for ( i=0; i<DESCISION_MATRIX_ELEMENTS; i++ ) {
		for ( j=0; j<8; j++ ) {
			writeEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + i*8 + j, 0 );
		}
	}
		
}


/*void readTemp(void) {
    char testdata1,testdata2;

    Delay100TCYx(30);
    ow_write_byte(0xCC);
    Delay100TCYx(1);
    ow_write_byte(0x44);
    Delay10KTCYx(255);
    Delay10KTCYx(255);
    ow_reset();
    Delay100TCYx(30);
    ow_write_byte(0xCC);
    Delay100TCYx(1);
    ow_write_byte(0xBE);
    Delay100TCYx(1);
    testdata1=ow_read_byte();
    Delay10TCYx(1);
    testdata2=ow_read_byte();
    ow_reset();
    Delay10KTCYx(255);

    sprintf(debugstring, "\r\ntemp: %c %c", testdata1, testdata2 );
    puts1USART(debugstring);
}
*/

///////////////////////////////////////////////////////////////////////////////
// doApplicationOneSecondWork
//

void doApplicationOneSecondWork( void )
{
    unsigned char i;
    unsigned char controlbits;

#ifdef SERIAL_DEBUG
//    putrs1USART("\r\n-->Entering doApplicationOneSecondWork");
    daf_putrs1USART("."); //output one dot per second to the terminal (crude timestamp indication)
#endif

/*    if ( DS1820_FindFirstDevice() ) {
        daf_putrs1USART("\r\nDS1820 device found");
    }

    sprintf(debugstring, "\r\ntemp: %f", DS1820_GetTempFloat() );
    puts1USART(debugstring);
*/
//    readTemp();
    
    for ( i=0; i<8; i++ ) {
        controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_OUT1 + i ); // Read control bits for this output
/*
#ifdef SERIAL_DEBUG  
    sprintf(debugstring, 
        "\r\ni=%2i, controlbits=0x%2x, REG_CONTROL_OUT1+i=0x%2x, %2i",
        (int)i, controlbits, REG_CONTROL_OUT1+i, REG_CONTROL_OUT1+i);
    puts1USART(debugstring);
#endif
*/
        if ( !( controlbits & RELAY_CONTROLBIT_ENABLED ) ) continue; // If not enabled check next
	if ( relay_protection_timer[ i ] ) {
                        relay_protection_timer[ i ]--;
			// Check if its time to act on protection time
			if ( !relay_protection_timer[ i ] && ( controlbits & RELAY_CONTROLBIT_PROTECTION ) ) {
				SetOut(i+1,0); //disable the output
                                if( controlbits & RELAY_CONTROLBIT_ONEVENT ) { // Should on event be sent?
                                                SendInformationEvent( REG_SUBZONE_OUT1 + i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_OFF );
                                }
				if ( controlbits & RELAY_CONTROLBIT_ALARM ) { // Should alarm be sent?
				    SendInformationEvent( REG_SUBZONE_OUT1 + i, VSCP_CLASS1_ALARM, VSCP_TYPE_ALARM_ALARM );
				}
                        }
        }
    }
							//do nothing so far for Hasselt
}
 

///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char getMajorVersion()
{
	return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char getMinorVersion()
{
	return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char getSubMinorVersion()
{
	return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get GUID from EEPROM
//

#ifdef GUID_IN_EEPROM
unsigned char getGuidFromEEprom( unsigned char idx )
{
	readEEPROM( EEPROM_REG_GUIID0 + idx );		
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

#ifdef MANUFACTURER_INFO_IN_EEPROM
unsigned char getManufacturerIdFromEEprom( unsigned char idx )
{
	readEEPROM( EEPROM_REG_MANUFACTUR_ID0 + idx );	
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

unsigned char getBootLoaderAlgorithm( void ) 
{
	return VSCP_BOOTLOADER_PIC1;	
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

unsigned char getBufferSize( void ) 
{
	return 8;	// Standard CAN frame
}


///////////////////////////////////////////////////////////////////////////////
//  vscp_readNicknamePermanent
//

uint8_t vscp_readNicknamePermanent( void )
{
    unsigned int nickname;

    nickname = readEEPROM( VSCP_EEPROM_NICKNAME );
#ifdef PERMANENT_NICK
    nickname = 0x02;                                                    //This sets a permanent nickname irrispective of what is in EEPROM
#endif                                                                  //because this is oftwen overwritten when debugging, which is annoying
                                                                        //TODO: remove this line when ready debugging
    if (nickname == 0xff) nickname = readFPM(VSCP_FPM_DEFAULT_NICK);
    return nickname;
}


///////////////////////////////////////////////////////////////////////////////
//  vscp_writeNicknamePermanent
//

void vscp_writeNicknamePermanent( uint8_t nickname )
{
	writeEEPROM( VSCP_EEPROM_NICKNAME, nickname );
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getZone
//

uint8_t vscp_getZone( void )
{
	return readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE );	
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getSubzone
//

uint8_t vscp_getSubzone( void )
{
	return readEEPROM( VSCP_EEPROM_END + EEPROM_SUBZONE );
}

///////////////////////////////////////////////////////////////////////////////
// doWork
//
// The actual work is done here.
//

void doWork( void )
{
	BOOL bOn = FALSE;

	if ( VSCP_STATE_ACTIVE == vscp_node_state ) {
            
            //execute input debounce & soundplay every 10 ms
		if ( tick_10ms > 1) {
                    ProcessInputs();
//                    PlaySound();
                    tick_10ms = 0;
		} //end 10ms
		
	}	
}

///////////////////////////////////////////////////////////////////////////////
// vscp_readAppReg
//

uint8_t vscp_readAppReg( uint8_t reg )
{
        uint8_t rv;
//	int tmpval;
//	unsigned char val, checksum;
        uint16_t userreg;

	rv = 0x00; // default return value
        userreg = reg + (vscp_page_select * 128);           // handle register pagination (see Register Abstraction Model)

        switch (userreg) {
            case REG_STATUS_OUT1:
            case REG_STATUS_OUT2:
            case REG_STATUS_OUT3:
            case REG_STATUS_OUT4:
            case REG_STATUS_OUT5:
            case REG_STATUS_OUT6:
            case REG_STATUS_OUT7:
            case REG_STATUS_OUT8:
		rv = ReadOut(userreg - REG_STATUS_OUT1 + 1);   // these registers only show status and are not in EEPROM
#ifdef SERIAL_DEBUG_RW_APP_REG
                daf_putrs1USART("\r\nCHECK READ");
#endif
                break;
            case REG_STATUS_PWM1:
            case REG_STATUS_PWM2:
		rv = ReadPWM((userreg - REG_STATUS_PWM1 + 1));   // these registers only show status and are not in EEPROM
                break;
            default:   
        	if ( userreg <= USER_EEPROM_LAST_POS ) {
                    rv = readEEPROM( VSCP_EEPROM_END + userreg );
#ifdef SERIAL_DEBUG_RW_APP_REG
    sprintf(debugstring,
            "\r\nRead app register: %02id:%02id (0x%02x:0x%02x), paginated: %02id, read value: %02id (0x%02x)",
            vscp_page_select, reg, vscp_page_select, reg, userreg, rv, rv );
    daf_puts1USART(debugstring);         //print to USART
#endif
                }
                break;
        }
	
	return rv;
}

////////    ///////////////////////////////////////////////////////////////////////
// vscp_writeAppReg
//

uint8_t vscp_writeAppReg( uint8_t reg, uint8_t val )
{
    uint8_t rv;
    uint16_t userreg;

	rv = ~val; // on error, return inverted value
        userreg = reg + (vscp_page_select * 128);           // handle register pagination (see Register Abstraction Model)

        switch (userreg) {
            case REG_STATUS_OUT1:
            case REG_STATUS_OUT2:
            case REG_STATUS_OUT3:
            case REG_STATUS_OUT4:
            case REG_STATUS_OUT5:
            case REG_STATUS_OUT6:
            case REG_STATUS_OUT7:
            case REG_STATUS_OUT8:
#ifdef SERIAL_DEBUG_RW_APP_REG
                daf_putrs1USART("\r\nREG_STATUS_OUT set");
#endif
                if ( 1 == val ) {
                    SetOut((userreg - REG_STATUS_OUT1 + 1),1);  // these registers only show status so no use writing them to EEPROM
                    rv = val;  // return OK
/*                    sprintf(debugstring,
                            "\r\nSetOut() called, reg: %02i, userreg: %02i, REG_STATUS_OUT1: %02i, output: %i, value: %i",
                            reg, userreg, REG_STATUS_OUT1, (userreg - REG_STATUS_OUT1 + 1), 1);
                    daf_puts1USART(debugstring);*/
                }
                if ( 0 == val ) {
                    SetOut((userreg - REG_STATUS_OUT1 + 1),0);
                    rv = val;  // return OK
/*                    sprintf(debugstring,
                            "\r\nSetOut() called, userreg: %02i, REG_STATUS_OUT1: %02i, output: %i, value: %i",
                            userreg, REG_STATUS_OUT1, (userreg - REG_STATUS_OUT1 + 1),0);
                    daf_puts1USART(debugstring);*/
                }
                break;
            case REG_STATUS_PWM1:
		SetPWM((userreg - REG_STATUS_PWM1 + 1),val);
                rv = val;  // return OK
                break;
            case REG_STATUS_PWM2:
		SetPWM((userreg - REG_STATUS_PWM1 + 1),val);
                rv = val;  // return OK
                break;
            case REG_MODE_OUT3:
                if (val == 0x01) CCP5CON = 0b0000000; //check if the mode for the output is set to output
                else CCP5CON = 0b00001100; //enable CCP5 for PWM output 3 (RB5) (PWM2)
                rv = val;  // return OK
                break;
            case REG_MODE_OUT4:
                if (val == 0x01) CCP1CON = 0b0000000; //check if the mode for the output is set to output
                else CCP1CON = 0b00001100; //enable CCP for PWM output
                rv = val;  // return OK
                break;
            default:
        	if ( userreg <= USER_EEPROM_LAST_POS ) {   //avoid overwriting registers outside of app register space
                    writeEEPROM( VSCP_EEPROM_END + userreg, val );
                    rv = readEEPROM( VSCP_EEPROM_END + userreg );
                } 
#ifdef SERIAL_DEBUG_RW_APP_REG
    sprintf(debugstring,
            "\r\nWrite app reg: %02id:%02id (0x%02x:0x%02x), paginated: %02id (0x%02x), ",
            vscp_page_select, reg, vscp_page_select, reg, userreg, userreg );
    daf_puts1USART(debugstring);         //print to USART
    sprintf(debugstring,
            "EEPROM loc.: %02id (0x%02x), write value: %02id, re-read value: %id",
             VSCP_EEPROM_END + userreg, VSCP_EEPROM_END + userreg, val, rv );
    daf_puts1USART(debugstring);         //print to USART
#endif
                break;
        }
	
	return rv;
}

///////////////////////////////////////////////////////////////////////////////
// Send Decsion Matrix Information
//

void sendDMatrixInfo( void )
{
	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG + 2;
	vscp_omsg.class = VSCP_CLASS1_PROTOCOL;
	vscp_omsg.type = VSCP_TYPE_PROTOCOL_GET_MATRIX_INFO_RESPONSE;

	vscp_omsg.data[ 0 ] = DESCISION_MATRIX_ELEMENTS;
	vscp_omsg.data[ 1 ] = REG_DESCISION_MATRIX;

	vscp_sendEvent();	// Send data		
}


///////////////////////////////////////////////////////////////////////////////
// SendInformationEvent
//

void SendInformationEvent( unsigned char idx, unsigned char eventClass, unsigned char eventTypeId) 
{
	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG + 3;
	vscp_omsg.class = eventClass;
	vscp_omsg.type = eventTypeId;

	vscp_omsg.data[ 0 ] = idx;
	vscp_omsg.data[ 1 ] = readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE );
 	vscp_omsg.data[ 2 ] = readEEPROM( VSCP_EEPROM_END + idx );

	vscp_sendEvent();	// Send data
}


///////////////////////////////////////////////////////////////////////////////
// SendInformationEventData
//

void SendInformationEventData( unsigned char idx, unsigned char eventClass, unsigned char eventTypeId, unsigned char data)
{
	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG + 3;
	vscp_omsg.class = eventClass;
	vscp_omsg.type = eventTypeId;

	vscp_omsg.data[ 0 ] = data;
	vscp_omsg.data[ 1 ] = readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE );
 	vscp_omsg.data[ 2 ] = readEEPROM( VSCP_EEPROM_END + REG_SUBZONE_INP1 + idx );

	vscp_sendEvent();	// Send data
}


///////////////////////////////////////////////////////////////////////////////
// Send Probe Ack
//

void SendProbeAck( unsigned char INPtime )
{
	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG+1;
	vscp_omsg.class = VSCP_CLASS1_PROTOCOL;
	vscp_omsg.type = VSCP_TYPE_PROTOCOL_PROBE_ACK;
	vscp_omsg.data[ 0] = INPtime;

	vscp_sendEvent();	// Send data		
}


///////////////////////////////////////////////////////////////////////////////
// Do decision Matrix handling
// 
// The routine expects vscp_imsg to contain a vaild incoming event
//

void doDM( void )
{
	unsigned char i;
	unsigned char dmflags;
	unsigned short class_filter;
	unsigned short class_mask;
	unsigned char type_filter;
	unsigned char type_mask;


	// Don't deal with the control functionality
	if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.class ) return;

#ifdef SERIAL_DEBUG_DM
        //Skip heartbeat messages 
        if ( vscp_imsg.class == 0x14 && vscp_imsg.type == 0x09) return;  
#endif

	for ( i=0; i<DESCISION_MATRIX_ELEMENTS; i++ ) {

		// Get DM flags for this row
		dmflags = readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + 1 + ( 8 * i ) );

#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring,
        "\r\n*** DoDM() row=%1i, flags=0x%02x, oaddr=0x%02x, zone=0x%02x, ",
        i, dmflags, vscp_imsg.oaddr, vscp_imsg.data[1]);
    daf_puts1USART(debugstring);
/*    sprintf(debugstring,
        "VSCP_DM_FLAG_ENABLED=%x, VSCP_DM_FLAG_CHECK_OADDR=%x, VSCP_DM_FLAG_CHECK_ZONE=%x, ",
        dmflags & VSCP_DM_FLAG_ENABLED, dmflags & VSCP_DM_FLAG_CHECK_OADDR, dmflags & VSCP_DM_FLAG_CHECK_ZONE );
    daf_puts1USART(debugstring);*/
    sprintf(debugstring,
        "class_mask=0x%02x, class_filter=0x%02x, type_mask=0x%02x, type_filter=0x%02x, ",
        class_mask, class_filter, type_mask, type_filter );
    daf_puts1USART(debugstring);
    sprintf(debugstring,
        " action=0x%02x, action_param=0x%02x",
        readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_ACTION), readEEPROM(VSCP_EEPROM_END + REG_DESCISION_MATRIX + (8*i) + VSCP_DM_POS_ACTIONPARAM) );
    daf_puts1USART(debugstring);
#endif

		// Is the DM row enabled?
		if ( dmflags & VSCP_DM_FLAG_ENABLED ) {
#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring, "\r\nDM row enabled, dmflags=0x%02x", dmflags);    daf_puts1USART(debugstring);
#endif
			// Should the originating id be checked and if so is it the same?
			if ( ( dmflags & VSCP_DM_FLAG_CHECK_OADDR ) &&  
				(  vscp_imsg.oaddr != readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) ) ) ) {
#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring,
            "\r\nOaddr is checked and does not match, oaddr: 0x%02x, this node's addr: 0x%02x",
            vscp_imsg.oaddr, readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) ));
    daf_puts1USART(debugstring);
#endif
				continue; // if oaddr doesn't match, skip to for-loop's next iteration
			}	

			// Check if zone should match and if so if it match
			if ( dmflags & VSCP_DM_FLAG_CHECK_ZONE  ) {
#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring, "\r\nCheck zone flag is set");
    daf_puts1USART(debugstring);
#endif
				if ( 255 != vscp_imsg.data[ 1 ] ) {
					if ( vscp_imsg.data[ 1 ] != readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE  ) ) {
#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring,
            "\r\nZones do not match, received zone: 0x%02x, this node's zone: 0x%02x ",
            vscp_imsg.data[1], readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE ) );
    daf_puts1USART(debugstring);
    sprintf(debugstring,
            ", VSCP_EEPROM_END: 0x%02x, EEPROM_ZONE: 0x%02x, sum: 0x%x",
            VSCP_EEPROM_END, EEPROM_ZONE, (VSCP_EEPROM_END + EEPROM_ZONE) );  
    daf_puts1USART(debugstring);
#endif
						continue;
					} else {
#ifdef SERIAL_DEBUG_DM
    sprintf(debugstring,
            "\r\nZones match, received zone: 0x%04x, this node's zone: 0x%04x ",
            vscp_imsg.data[1], readEEPROM( VSCP_EEPROM_END + EEPROM_ZONE ) );
    daf_puts1USART(debugstring);
    sprintf(debugstring,
            ", VSCP_EEPROM_END: 0x%04x, EEPROM_ZONE: 0x%04x, sum: 0x%04x",
            VSCP_EEPROM_END, EEPROM_ZONE, (VSCP_EEPROM_END + EEPROM_ZONE) );  
    daf_puts1USART(debugstring);
#endif
                                        }
				}	
			}				

			class_filter = ( dmflags & VSCP_DM_FLAG_CLASS_FILTER )*256 + 
									readEEPROM( VSCP_EEPROM_END + 
													REG_DESCISION_MATRIX +
													( 8 * i ) + 
													VSCP_DM_POS_CLASSFILTER  );
			class_mask = ( dmflags & VSCP_DM_FLAG_CLASS_MASK )*256 + 
									readEEPROM( VSCP_EEPROM_END + 
													REG_DESCISION_MATRIX +
													( 8 * i ) +
													 VSCP_DM_POS_CLASSMASK  );
			type_filter = readEEPROM( VSCP_EEPROM_END + 
										REG_DESCISION_MATRIX +
										( 8 * i ) + 
										VSCP_DM_POS_TYPEFILTER );
			type_mask = readEEPROM( VSCP_EEPROM_END + 
										REG_DESCISION_MATRIX +
										( 8 * i ) + 
										VSCP_DM_POS_TYPEMASK  );

#ifdef SERIAL_DEBUG_DM_ACTIONS
    if (vscp_imsg.class != 0x14 && vscp_imsg.type != 0x09) {   // do not display heartbeat events
        sprintf(debugstring,
                "\r\n* This node: Class mask: 0x%02x, Class filter: 0x%02x, Type mask: 0x%02x, Type filter: 0x%02x",
                class_mask, class_filter, type_mask, type_filter );
        daf_puts1USART(debugstring);
        sprintf(debugstring,
                "\r\nReceived event: Class: 0x%02x, Type: 0x%02x, Data[0]: 0x%02x, Data[1]: 0x%02x, Data[2]: 0x%02x, Data[3]: 0x%02x",
                vscp_imsg.class, vscp_imsg.type, vscp_imsg.data[0], vscp_imsg.data[1], vscp_imsg.data[2], vscp_imsg.data[3]);
        daf_puts1USART(debugstring);
    }
#endif

			if ( !( ( class_filter ^ vscp_imsg.class ) & class_mask ) &&
				 	!( ( type_filter ^ vscp_imsg.type ) & type_mask )) {

				// OK Trigger this action
				switch ( readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTION  ) ) {
					
					case ACTION_ON:			// Enable relays 
#ifdef SERIAL_DEBUG_DM_ACTIONS
    daf_putrs1USART("\r\nDoDM() -> ACTION_ON 0x01");
/*    sprintf(debugstring,
            ", action param: 0x%02x (register 0x%02x)",
            readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM ),
            REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM );
    daf_puts1USART(debugstring);*/
#endif
                                                doActionOn( dmflags, readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
						break; 
						
					case ACTION_OFF: 		// Disable relays 
#ifdef SERIAL_DEBUG_DM_ACTIONS
    daf_putrs1USART("\r\nDoDM() -> ACTION_OFF 0x02");
#endif
						doActionOff(dmflags,  readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
						break;
                                        case ACTION_TOGGLE: 		// Disable relays
#ifdef SERIAL_DEBUG_DM_ACTIONS
    daf_putrs1USART("\r\nDoDM() -> ACTION_TOGGLE 0x04");
#endif
						doActionToggle(dmflags,  readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
						break;
                                        case ACTION_PWM: 		// Disable relays 
#ifdef SERIAL_DEBUG_DM_ACTIONS
    daf_putrs1USART("\r\nDoDM() -> ACTION_PWM 0x08");
#endif
						doActionPWM(dmflags,  readEEPROM( VSCP_EEPROM_END + REG_DESCISION_MATRIX + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
						break;

				} // case	
			} // Filter/mask
		} // Row enabled
	} // for each row
        
}

///////////////////////////////////////////////////////////////////////////////
// doActionOn
// 

void doActionOn( unsigned char dmflags, unsigned char arg )  // passed: dmflags, action param
{
	unsigned char i;
        unsigned char controlbits;
	
	for ( i=0; i<8; i++ ) { // for each output

//david: it seems that arg is ignored in this code, and that all outputs are switched on if their zone matches what was sent
		// If the output should not be handled just move on
		//if ( !( arg & ( 1 << i ) ) ) continue;

		// Check if subzone should match and if so if it matches
		if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
			if ( vscp_imsg.data[ 2 ] != readEEPROM( VSCP_EEPROM_END + REG_SUBZONE_OUT1 + i ) ) {
				continue;  // skip code that follows and go straight to next iteration of for loop
			}
		}
                controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_OUT1 + i); //read the control bits for the output
                if ( !( controlbits & RELAY_CONTROLBIT_ENABLED ) ) continue; //Check if the output is enabled only then continue
                SetOut(i+1,1); //activate the output
		if( controlbits & RELAY_CONTROLBIT_ONEVENT ) { // Should on event be sent?
			SendInformationEvent( REG_SUBZONE_OUT1 + i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_ON );
		}
                if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // Should output be protected with countdown timer?
			relay_protection_timer[ i ] = readEEPROM( VSCP_EEPROM_END + REG_PTIME_OUT1 + i ) * 64;
                }
	}	
}

///////////////////////////////////////////////////////////////////////////////
// doActionOff
// 

void doActionOff( unsigned char dmflags, unsigned char arg )
{
	unsigned char i;
        unsigned char controlbits;
	
	for ( i=0; i<8; i++ ) {
		
		// If the output should not be handled just move on
		// if ( !( arg & ( 1 << i ) ) ) continue;
		
		// Check if subzone should match and if so if it match
		if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
			if ( vscp_imsg.data[ 2 ] != readEEPROM( VSCP_EEPROM_END + REG_SUBZONE_OUT1 + i ) ) {
				continue;
			}
		}
                controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_OUT1 + i); //read the control bits for the output
                if ( !( controlbits & RELAY_CONTROLBIT_ENABLED ) ) continue; //Check if the output is enabled only then continue
                SetOut(i+1,0); //disable the output
		if( controlbits & RELAY_CONTROLBIT_ONEVENT ) { // Should on event be sent?
			SendInformationEvent( REG_SUBZONE_OUT1 + i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_OFF );
		}
                if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // is output protected with countdown timer. Reset in case.
			relay_protection_timer[ i ] = 0;
                }
                
	}
}

///////////////////////////////////////////////////////////////////////////////
// doActionToggle
//

void doActionToggle( unsigned char dmflags, unsigned char arg )
{
//	unsigned char i;
        uint8_t i;
        unsigned char controlbits;
	BOOL bOn = FALSE;

	for ( i=0; i<8; i++ ) {

		// If the rely should not be handled just move on
		// if ( !( arg & ( 1 << i ) ) ) continue;

		// Check if subzone should match and if so if it match
		if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
			if ( vscp_imsg.data[ 2 ] != readEEPROM( VSCP_EEPROM_END + REG_SUBZONE_OUT1 + i ) ) {
				continue;
			}
		}

                if ( vscp_imsg.data[ 0 ] != INP_PRESSED ) continue;  //check if button press event (not repeat or release) otherwise continue next i in loop

		controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_OUT1 + i); //read the control bits for the output
                if ( !( controlbits & RELAY_CONTROLBIT_ENABLED ) ) continue; //Check if the output is enabled only then continue
                if ( ReadOut( i+1 ) ) { 
#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nSetOut(%i,%i), OUT1: %i", i+1, 0, OUT1 );
    daf_puts1USART(debugstring);
#endif
                    SetOut(i+1,0);
                    if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // is output protected with countdown timer. Reset in case.
			relay_protection_timer[ i ] = 0;
                    }
                    if( controlbits & RELAY_CONTROLBIT_OFFEVENT ) SendInformationEvent( i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_OFF );
                }
                else {
#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nSetOut(%i,%i), OUT1: %i", i+1, 0, OUT1 );
    daf_puts1USART(debugstring);
#endif
                    SetOut(i+1,1);
                    if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // Should output be protected with countdown timer?
			relay_protection_timer[ i ] = readEEPROM( VSCP_EEPROM_END + REG_PTIME_OUT1 + i ) * 64;
                    }
                    if( controlbits & RELAY_CONTROLBIT_ONEVENT ) SendInformationEvent( i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_ON );

                }
	}
}



///////////////////////////////////////////////////////////////////////////////
// doActionPWM
//

void doActionPWM( unsigned char dmflags, unsigned char arg )
{
	unsigned char i;
	unsigned char controlbits, INP_type, assout;
	BOOL bOn = TRUE;

        for ( i=0; i<2; i++) {


		// Check if subzone should match and if so if it match
		if ( dmflags & VSCP_DM_FLAG_CHECK_SUBZONE ) {
			if ( vscp_imsg.data[ 2 ] != readEEPROM( VSCP_EEPROM_END + REG_SUBZONE_PWM1 + i) ) {
				continue;
			}
		}
                controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_PWM1 + i); //read the control bits for the PWM
		if ( !( controlbits & RELAY_CONTROLBIT_ENABLED ) ) continue; //Check if the output is enabled only then continue
                
                //seems event has right zone for PWM & enabled so continue

		//val = readEEPROM( VSCP_EEPROM_END + REG_PWM_CONTROL ); //leave in here for compat for now
		INP_type = vscp_imsg.data[ 0 ];

		switch ( PWM_state[i]) {

			case PWM_OFF:
				if (INP_type == INP_KEY){
					PWM_state[i] = PWM_ON;
					SetPWM(i+1, PWM_value[i]);

				}
				if (INP_type == INP_HOLD) {
					PWM_state[i] = PWM_UP;
					PWM_value[i] = 0;
				}
				break;

			case PWM_UP:
				if (INP_type == INP_RELEASED){
					PWM_state[i] = PWMMED_UP;
					//writeEEPROM( VSCP_EEPROM_END + REG_STATUS_PWM1 + i, PWM_value );
					SetPWM(i+1, PWM_value[i]);
				}
				if (INP_type == INP_HOLD) {
					PWM_state[i] = PWM_UP;
					if ((PWM_value[i] + arg) >= 255) PWM_value[i] = 255;
						else PWM_value[i] = PWM_value[i] + arg;
					SetPWM(i+1, PWM_value[i]);
				}
				break;

			case PWMMED_UP:
				if (INP_type == INP_KEY){
					PWM_state[i] = PWM_OFF;
					SetPWM(i+1,0);
					bOn = FALSE;
				}
				if (INP_type == INP_HOLD) {
					PWM_state[i] = PWM_DOWN;
				}
				break;

			case PWM_DOWN:
				if (INP_type == INP_RELEASED){
					if (PWM_value[i] < readEEPROM( VSCP_EEPROM_END + REG_TRESH_PWM1 + i)) {
						PWM_state[i] = PWM_OFF;
						PWM_value[i] = 0;
						bOn = FALSE;
					}
					else {
						PWM_state[i] = PWM_ON;
					}
					//writeEEPROM( VSCP_EEPROM_END + REG_PWM_VALUE, PWM_value );
					SetPWM(i+1, PWM_value[i]);
				}
				if (INP_type == INP_HOLD) {
					PWM_state[i] = PWM_DOWN;
					if ((PWM_value[i] - arg) <= 0) PWM_value[i] = 0;
					else PWM_value[i] = PWM_value[i] - arg;
					SetPWM(i+1, PWM_value[i]);
				}
				break;

			case PWM_ON:
				if (INP_type == INP_KEY){
					PWM_state[i] = PWM_OFF;
					SetPWM(i+1, 0);
					bOn = FALSE;
				}
				if (INP_type == INP_HOLD) {
					PWM_state[i] = PWM_UP;
				}
				break;
		}

                // turn on/off associated outut if PWM value is above treshold
                assout = readEEPROM( VSCP_EEPROM_END + REG_ASSOUT_PWM1 + i);
                if (assout > 0 && assout < 9) {
                    if(PWM_value[i] >= readEEPROM( VSCP_EEPROM_END + REG_TRESH_PWM1 + i)) {
                        SetOut(assout,1);
                        if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // Should output be protected with countdown timer?
                            relay_protection_timer[ assout-1 ] = readEEPROM( VSCP_EEPROM_END + REG_PTIME_OUT1 + assout - 1 ) * 64;
                    }
                    }
                    else {
                        SetOut(assout,0);
                         if( controlbits & RELAY_CONTROLBIT_PROTECTION ) { // is output protected with countdown timer. Reset in case.
                            relay_protection_timer[ assout-1 ] = 0;
                         }
                    }
                }

                //send event
		if ( bOn ) {

			// Should on event be sent?
			if( controlbits & RELAY_CONTROLBIT_ONEVENT ) {
				SendInformationEvent( REG_SUBZONE_PWM1 + i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_ON );
			}

		}
		else {

			// Should off event be sent?
			if( controlbits & RELAY_CONTROLBIT_OFFEVENT ) {
				SendInformationEvent( REG_SUBZONE_PWM1 + i, VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_OFF );
			}

		}
        }

}


///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char vscp_getMajorVersion()
{
	return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char vscp_getMinorVersion()
{
	return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char vscp_getSubMinorVersion()
{
	return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// getVSCP_GUID
//
// Get GUID from EEPROM
//

uint8_t vscp_getGUID( uint8_t idx )
{
	return readFPM( VSCP_FPM_GUID + idx );
}


///////////////////////////////////////////////////////////////////////////////
// getDeviceURL
//
// Get device URL from EEPROM
//

uint8_t vscp_getMDF_URL( uint8_t idx )
{
	return vscp_deviceURL[ idx ];
}

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getUserID( uint8_t idx )
{
	return readEEPROM( VSCP_EEPROM_REG_USERID + idx );	
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPUserID
//

void vscp_setUserID( uint8_t idx, uint8_t data )
{
	writeEEPROM( idx + VSCP_EEPROM_REG_USERID, data );
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getManufacturerId( uint8_t idx )
{
	return readEEPROM( VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx );	
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

void vscp_setManufacturerId( uint8_t idx, uint8_t data )
{
	writeEEPROM( VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx, data );	
}

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

uint8_t vscp_getBootLoaderAlgorithm( void ) 
{
	return VSCP_BOOTLOADER_PIC1;	
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

uint8_t vscp_getBufferSize( void ) 
{
	return 8;	// Standard CAN frame
}

	
///////////////////////////////////////////////////////////////////////////////
//  getNickname
//

uint8_t vscp_getNickname( void )
{
	return readEEPROM( VSCP_EEPROM_NICKNAME );
}

///////////////////////////////////////////////////////////////////////////////
//  setNickname
//

void vscp_setNickname( uint8_t nickname )
{
	writeEEPROM( VSCP_EEPROM_NICKNAME, nickname );
}

///////////////////////////////////////////////////////////////////////////////
//  getSegmentCRC
//

uint8_t vscp_getSegmentCRC( void )
{
	return readEEPROM( VSCP_EEPROM_SEGMENT_CRC );
}

///////////////////////////////////////////////////////////////////////////////
//  setSegmentCRC
//

void vscp_setSegmentCRC( uint8_t crc )
{
	writeEEPROM( VSCP_EEPROM_SEGMENT_CRC, crc );
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPControlByte
//

void vscp_setControlByte( uint8_t ctrl )
{
	writeEEPROM( VSCP_EEPROM_CONTROL, ctrl );
}


///////////////////////////////////////////////////////////////////////////////
//  getVSCPControlByte
//

uint8_t vscp_getControlByte( void )
{
	return readEEPROM( VSCP_EEPROM_CONTROL );
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getEmbeddedMdfInfo
//

void vscp_getEmbeddedMdfInfo( void )
{
	// No embedded DM so we respond with info about that
	
	vscp_omsg.priority = VSCP_PRIORITY_NORMAL;
	vscp_omsg.flags = VSCP_VALID_MSG + 3;
	vscp_omsg.class = VSCP_CLASS1_PROTOCOL;
	vscp_omsg.type = VSCP_TYPE_PROTOCOL_RW_RESPONSE;

	vscp_omsg.data[ 0 ] = 0;
	vscp_omsg.data[ 1 ] = 0;
	vscp_omsg.data[ 2 ] = 0;	
	
	// send the message
	vscp_sendEvent();
}

/*
///////////////////////////////////////////////////////////////////////////////
// vscp_getZone
//

uint8_t vscp_getZone( void )
{
	return readEEPROM( EEPROM_ZONE );  //david: shouldn't this be prefixed by VSCP_EEPROM_END?
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getSubzone
//

uint8_t vscp_getSubzone( void )
{
	return readEEPROM( EEPROM_SUBZONE ); //david: shouldn't this be prefixed by VSCP_EEPROM_END?
}
*/

///////////////////////////////////////////////////////////////////////////////
// vscp_goBootloaderMode
//

void vscp_goBootloaderMode( void )
{	 											
	// OK, We should enter boot loader mode
	// 	First, activate bootloader mode
	writeEEPROM( VSCP_EEPROM_BOOTLOADER_FLAG, VSCP_BOOT_FLAG );
					
	//_asm goto _startup reset _endasm
	_asm reset _endasm
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getMatrixInfo
//

void vscp_getMatrixInfo( char *pData )
{
	uint8_t i;
	
	for ( i = 0; i < 8; i++ ) {
		vscp_omsg.data[ i ] = 0;
	}	
	vscp_omsg.data[ 0 ] = DESCISION_MATRIX_ELEMENTS; //[KURT] added for test  //TODO
	vscp_omsg.data[ 1 ] = REG_DESCISION_MATRIX; //[KURT] added for test       //TODO
}

///////////////////////////////////////////////////////////////////////////////
//
//

uint8_t vscp_getRegisterPagesUsed( void )
{
	return 3;	// One page used   // this register is obsolete so return value doesn't matter
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPFrame
//

int8_t sendVSCPFrame( uint16_t vscpclass, 
                      uint8_t vscptype,
                      uint8_t nodeid,
		      uint8_t priority,
		      uint8_t size,
	              uint8_t *pData )
{
	uint32_t id = ( (uint32_t)priority << 26 ) |
						( (uint32_t)vscpclass << 16 ) |
						( (uint32_t)vscptype << 8) |
						nodeid;		// nodeaddress (our address)
	
	if ( !sendCANFrame( id, size, pData ) ) {
		// Failed to send message
		vscp_errorcnt++;
		return FALSE;
	}
	
	return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// getVSCPFrame
//

int8_t getVSCPFrame( uint16_t *pvscpclass, 
						uint8_t *pvscptype, 
						uint8_t *pNodeId, 
						uint8_t *pPriority, 
						uint8_t *pSize, 
						uint8_t *pData )
{
	uint32_t id;
	
	if ( !getCANFrame( &id, pSize, pData ) ) {
		return FALSE;
	}

	*pNodeId = id & 0x0ff;
	*pvscptype = ( id >> 8 ) & 0xff;
	*pvscpclass = ( id >> 16 ) & 0x1ff;
        *pPriority = (uint16_t)( 0x07 & ( id >> 26 ) );
    
	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// sendCANFrame
//

int8_t sendCANFrame( uint32_t id, uint8_t dlc, uint8_t *pdata )
{
	if ( !vscp18f_sendMsg( id, 
							pdata , 
							dlc, 
							CAN_TX_XTD_FRAME  ) ) {
		
		// Failed to send message
		return FALSE;

	}

	vscp_omsg.flags = 0;
	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// getCANFrame
//

int8_t getCANFrame( uint32_t *pid, uint8_t *pdlc, uint8_t *pdata )
{
	uint8_t flags;

	// Dont read in new message if there already is a message
	// in the input buffer
	if ( vscp_imsg.flags & VSCP_VALID_MSG ) return FALSE;

	if ( vscp18f_readMsg( pid, pdata, pdlc, &flags ) ) {

		// RTR not interesting
		if ( flags & CAN_RX_RTR_FRAME ) return FALSE;

		// Must be extended frame
		if ( !( flags & CAN_RX_XTD_FRAME ) ) return FALSE;
		
		return TRUE;
	}	
	
	return FALSE;
}


///////////////////////////////////////////////////////////////////////////////
// DebounceSwitch
//

unsigned char DebounceINP1(void)
{
// <<11111111011101110001111100000110000000keypressed000000000001100101111011111111111111<<
BufferINP1=(BufferINP1<<1) | INP1 | 0xe0;   // 11100000
if(BufferINP1==0xf0)return INP_PRESSED;     // 11110000
if(BufferINP1==0xe0)return INP_HOLD;        // 11100000
if(BufferINP1==0xe1)return INP_RELEASED;    // 11100001
return FALSE;
}

unsigned char DebounceINP2(void)
{
BufferINP2=(BufferINP2<<1) | INP2 | 0xe0;
if(BufferINP2==0xf0)return INP_PRESSED;
if(BufferINP2==0xe0)return INP_HOLD;
if(BufferINP2==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP3(void)
{
BufferINP3=(BufferINP3<<1) | INP3 | 0xe0;
if(BufferINP3==0xf0)return INP_PRESSED;
if(BufferINP3==0xe0)return INP_HOLD;
if(BufferINP3==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP4(void)
{
BufferINP4=(BufferINP4<<1) | INP4 | 0xe0;
if(BufferINP4==0xf0)return INP_PRESSED;
if(BufferINP4==0xe0)return INP_HOLD;
if(BufferINP4==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP5(void)
{
BufferINP5=(BufferINP5<<1) | INP5 | 0xe0;
if(BufferINP5==0xf0)return INP_PRESSED;
if(BufferINP5==0xe0)return INP_HOLD;
if(BufferINP5==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP6(void)
{
BufferINP6=(BufferINP6<<1) | INP6 | 0xe0;
if(BufferINP6==0xf0)return INP_PRESSED;
if(BufferINP6==0xe0)return INP_HOLD;
if(BufferINP6==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP7(void)
{
BufferINP7=(BufferINP7<<1) | INP7 | 0xe0;
if(BufferINP7==0xf0)return INP_PRESSED;
if(BufferINP7==0xe0)return INP_HOLD;
if(BufferINP7==0xe1)return INP_RELEASED;
return FALSE;
}

unsigned char DebounceINP8(void)
{
BufferINP8=(BufferINP8<<1) | INP8 | 0xe0;
if(BufferINP8==0xf0)return INP_PRESSED;
if(BufferINP8==0xe0)return INP_HOLD;
if(BufferINP8==0xe1)return INP_RELEASED;
return FALSE;
}



uint8_t ReadOut(uint8_t output) {     //ccc2
    uint8_t ReadValue;

#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nOutput: %d", output);
    daf_puts1USART(debugstring);
#endif

    switch ( (uint8_t)output ) {
                        case 0:
                                ReadValue = 0;
#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nCase %d; ReadValue: %d, OUT1: %d", (uint8_t)output, ReadValue, OUT1 );
    daf_puts1USART(debugstring);
#endif
                                break;
                        case 1:
                                ReadValue = OUT1;
#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nCase %d; ReadValue: %d, OUT1: %d", (uint8_t)output, ReadValue, OUT1 );
    daf_puts1USART(debugstring);
#endif
				break;

			case 2:
				ReadValue = OUT2;
				break;

			case 3:
				ReadValue = OUT3;
				break;

			case 4:
				ReadValue = OUT4;
				break;

			case 5:
				ReadValue = OUT5;
				break;
                        case 6:
				ReadValue = OUT6;
				break;

			case 7:
				ReadValue = OUT7;
				break;

			case 8:
				ReadValue = OUT8;
				break;
    }
#ifdef SERIAL_DEBUG_DM_ACTIONS
    sprintf(debugstring, "\r\nReadOut(%d)= %d", (uint8_t)output, ReadValue );
    daf_puts1USART(debugstring);
#endif
    return ReadValue;
}


void SetOut(uint8_t output, BOOL value)  // output: 1-8, value: 0 or 1
{
    switch ( output ) {
                        case 0:
                                break;
                        case 1:
#ifdef SERIAL_DEBUG_DM_ACTIONS
//                                daf_putrs1USART("SetOut() output 1");  //for some reason a serial output command here crashes the PIC...
#endif
                                OUT1 = value;
                                OUT8 = value;

				break;
			case 2:
				OUT2 = value;
                                OUT9 = value;
				break;

			case 3:
				OUT3 = value;
                                OUT10 = value;
				break;

			case 4:
				OUT4 = value;
                                OUT11 = value;
				break;
			case 5:
				OUT5 = value;
                                OUT12 = value;
				break;
                        case 6:
				OUT6 = value;
                                OUT13 = value;
				break;
			case 7:
				OUT7 = value;
                                OUT14 = value;
				break;
			case 8:
				OUT8 = value;
//				LED8 = value;
				break;
	}
}

uint8_t ReadPWM(uint8_t pwm)
{
    uint8_t ReadValue;
    switch ( pwm ) {
			case 1:
                                ReadValue = PWM1;
				break;

			case 2:
				ReadValue = PWM2;
				break;

    }
    return ~ReadValue;
}

void SetPWM(uint8_t pwm, uint8_t value)
{
    value = ~value; //inverse value as PWM's are inversed
    //if ( !( readEEPROM( VSCP_EEPROM_END + REG_MODE_OUT3 + pwm - 1) == 0x02)) continue; //check if the mode for the output is set to PWM. If not ... abort
    switch ( pwm ) {
			case 1:
				PWM1 = value;
				break;

			case 2:
				PWM2 = value;
				break;
    }
}


void ProcessInputs()
{
    unsigned int i;
    unsigned char keyval,controlbits;

    for ( i=0 ; i<8; i++) {
            switch (i) {
                    case 0:
                            keyval = DebounceINP1();
                            break;
                    case 1:
                            keyval = DebounceINP2();
                            break;
                    case 2:
                            keyval = DebounceINP3();
                            break;
                    case 3:
                            keyval = DebounceINP4();
                            break;
                    case 4:
                            keyval = DebounceINP5();
                            break;
                    case 5:
                            keyval = DebounceINP6();
                            break;
                    case 6:
                            keyval = DebounceINP7();
                            break;
                    case 7:
                            keyval = DebounceINP8();
                            break;
            }
            controlbits = readEEPROM( VSCP_EEPROM_END + REG_CONTROL_INP1 + i );
            switch (INP_state[i]) {
                    case INP_RELEASED:
                            if (keyval == INP_PRESSED) {
                                    INP_state[i] = INP_PRESSED;
                                    if( controlbits & INP_CONTROLBIT_REDG ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_PRESSED );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_PRESSED");
#endif
                            }
                            if (keyval == INP_RELEASED) {
                                    INP_state[i] = INP_RELEASED;
                                    INP_hold_timer[i] = 0;
                            }
                            break;
                    case INP_PRESSED:
                            if (keyval == INP_RELEASED) {
                                    INP_state[i] = INP_RELEASED;
                                    if( controlbits & INP_CONTROLBIT_FEDG ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_KEY );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_KEY");
#endif
                            }
                            if (keyval == INP_HOLD) {
                                    INP_state[i] = INP_HOLD;
                            }
                            break;
                    case INP_HOLD:
                            if (keyval == INP_RELEASED) {
                                    INP_state[i] = INP_RELEASED;
                                    if( controlbits & INP_CONTROLBIT_FEDG ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_KEY );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_KEY");
#endif
                            }
                            if (keyval == INP_HOLD) {
                                    INP_hold_timer[i] += 1;
                                    if ( INP_hold_timer[i] > 50 ) {
                                                    INP_state[i] = INP_HOLD_REPEAT;
                                                    if( controlbits & INP_CONTROLBIT_RPT ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_HOLD );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_HOLD");
#endif
                                                    INP_hold_timer[i] = 0;
                                            }
                                            else INP_state[i] = INP_HOLD;
                            }
                            break;
                    case INP_HOLD_REPEAT:
                            if (keyval == INP_RELEASED) {
                                    INP_state[i] = INP_RELEASED;
                                    if( controlbits & INP_CONTROLBIT_FEDG ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_RELEASED );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_RELEASED");
#endif
                                    INP_hold_timer[i] = 0;
                            }
                            if (keyval == INP_HOLD) {
                                    INP_hold_timer[i] += 1;
                                    if ( INP_hold_timer[i] > 5 ) {
                                                    if( controlbits & INP_CONTROLBIT_RPT ) SendInformationEventData( i , VSCP_CLASS1_INFORMATION, VSCP_TYPE_INFORMATION_BUTTON, INP_HOLD );
#ifdef SERIAL_DEBUG_INPUTS
    daf_putrs1USART("\r\nINP_HOLD");
#endif
                                                    INP_hold_timer[i] = 0;
                                            };
                                    INP_state[i] = INP_HOLD_REPEAT;
                            }
                            break;
                    }
    } //end for i
}

void PlaySound()
{
    
}



void FillEEPROM(void) { //write EEPROM contents with 0's or incrimental numbers, for debugging
#ifdef SERIAL_DEBUG_DUMPEEPROM
    unsigned int LocationIndex;
    unsigned int StartLocation = 0;            //TODO: change to actual VSCP EEPROM location
    unsigned int EndLocation = 1023;//USER_EEPROM_LAST_POS;

    if (EndLocation > 1023)                                 // prevent reading beyond EEPROM memory end
        EndLocation = 1023;

    sprintf(debugstring,
            "\r\n*** FillEEPROM(), StartLocation = %i, EndLocation = %i\r\n",
            StartLocation, EndLocation );
    daf_puts1USART(debugstring);

    for (LocationIndex=0; LocationIndex <= EndLocation; LocationIndex++) {   //for each line
        writeEEPROM( LocationIndex, 0xFF );  //make empty
//        writeEEPROM( LocationIndex, Value);  //fill with incremental value
        sprintf(debugstring,
                "0x%02x:0x%02x\r\n",
//                LocationIndex, Value );  // fill with incremental value
                LocationIndex, readEEPROM( LocationIndex) );
        daf_puts1USART(debugstring);
//        Value++;
    }
    daf_putrs1USART("\r\n");
#endif
}


void DumpEEPROM(void) {  //dump EEPROM contents to USART, for debugging
#ifdef SERIAL_DEBUG_DUMPEEPROM
    int i, j;                           //i = element index, j = column index
    int StartLocation, EndLocation, StartLocationDM, EndLocationDM;
    int colums = 8;

    StartLocation = VSCP_EEPROM_END;  // start off app registers
//    StartLocation = 0;                  // start of vscp registers (app registers follow)
    EndLocation = VSCP_EEPROM_END + USER_EEPROM_LAST_POS;
//    EndLocation = 383;
//    StartLocationDM = (unsigned int)REG_DESCISION_MATRIX;
//    EndLocationDM = (unsigned int)REG_DESCISION_MATRIX + 8 * (unsigned int)DESCISION_MATRIX_ELEMENTS;

    if (EndLocation > 1023)                                 // prevent reading beyond EEPROM memory end
        EndLocation = 1023;
//    if (EndLocationDM > 1023)                                 // prevent reading beyond EEPROM memory end
//        EndLocationDM = 1023;

    sprintf(debugstring,
            "\r\n*** DumpEEPROM(), StartLocation = %i, EndLocation = %i, USER_EEPROM_LAST_POS = %i\r\n",
            StartLocation, EndLocation, USER_EEPROM_LAST_POS );
    daf_puts1USART(debugstring);
    sprintf(debugstring,
            "page select MSB (0x92): %02xx (%02id), LSB (0x93): %02xx (%02id)\r\n",
            (vscp_page_select >> 8) & 0xff, (vscp_page_select >> 8) & 0xff, vscp_page_select & 0xff, vscp_page_select & 0xff );
    daf_puts1USART(debugstring);

/*    sprintf(debugstring,
            "\r\nStartLocationDM = %i, EndLocationDM = %i\r\n",
            StartLocationDM, EndLocationDM);
    daf_puts1USART(debugstring);
*/

    for (i=StartLocation; i <= EndLocation; i++) {   //just dump contents, one line per value (app registers)
//    for (i=0; i <= 530; i++) {   //don't take into account defined locations
        sprintf(debugstring,                 //print EEPROM location and its content
                "%4i:0x%02X / %03id (page 0 reg %i)\r\n",
                i, readEEPROM(i), readEEPROM(i), i - VSCP_EEPROM_END );
//        if ( (i>=0   && i<=127) ||  // page 0 app reg
        if ( (i>=0   && i<=128) ||  // page 0 app reg
             (i>=256 && i<=383) // page 1 app reg
           ) {
            daf_puts1USART(debugstring);         //print to USART
        }
    }

/*    for (i=StartLocationDM; i <= EndLocationDM; i++) {   //just dump contents, one line per value (DM registers)
        sprintf(debugstring,                 //print EEPROM location and its content
                "%4i:0x%02X / %03id (page 1 reg %i)\r\n",
                i, readEEPROM(i), readEEPROM(i), 256 - i );
        daf_puts1USART(debugstring);         //print to USART
    }
*/
/*    //print EEPROM locations in a neat table
    for (i=0; i < ( (EndLocation - StartLocation) / colums ) ; i++) {   //for each line
        for (j=0; j<colums; j++) {               //for each element in this row
            sprintf(debugstring,                 //print EEPROM location and its content
                    "%4i:0x%02X ",
                    ( (EndLocation - StartLocation) / colums * j + i + StartLocation),
                    readEEPROM( (EndLocation - StartLocation) / colums * j + i + StartLocation) );
            daf_puts1USART(debugstring);         //print to USART
        }
        daf_putrs1USART("\r\n");                 //next line
    }
*/
#endif
}
