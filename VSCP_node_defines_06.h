/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 * 	2009-11-08
 * 	kurt.sidekick@yahoo.com
 *
 *  Copyright (C) 2009 Kurt Herremans
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
*/
/*
 * IO definition for Antwerp
 */

#ifndef HASSELT_v2
#define HASSELT_v2
#ifndef MESPELARE_v1
#define MESPELARE_v1

// Firmware version

#define FIRMWARE_MAJOR_VERSION		1
#define FIRMWARE_MINOR_VERSION		0
#define FIRMWARE_SUB_MINOR_VERSION	0
#define VSCP_FPM_DEFAULT_NICK           0x600
#define VSCP_FPM_GUID                   0x602   //location in program flash where node GUID is stored (programmed together with bootloader)


//Timer0 used as 10ms timer. Timer generates interupt on overflow.
//10ms = 80000 clock ticks at 8Mhz Fosc/4. Prescaler = 1/2. Timer0ReloadValue=65536-40000=25536
#define Timer0ReloadValue               25536

//// * * * I/O Definitions * * *

//#define PWMaudio                                                        CCPR1L
#define PWM1								CCPR1L
#define PWM2								CCPR5L

//           Mespelare pins                                 // original Hasselt pins
#define OUT1 LATEbits.LATE1                                 // LATBbits.LATB0
#define OUT2 LATEbits.LATE2                                 // LATBbits.LATB1
#define OUT3 LATCbits.LATC0                                 // LATBbits.LATB5  //PWM5 out -> output or PWM
#define OUT4 LATCbits.LATC1                                 // LATBbits.LATB4  //PWM1 out -> output or PWM
#define OUT5 LATCbits.LATC2                                 // LATAbits.LATA3
#define OUT6 LATDbits.LATD0                                 // LATAbits.LATA2
#define OUT7 LATDbits.LATD1                                 // LATAbits.LATA1
#define OUT8 LATDbits.LATD1                                 // LATAbits.LATA0

//                  Mespelare pins                          // original Hasselt pins
#define INIT_LED    LATDbits.LATD3                          // LATAbits.LATA5
#define BUZZER      LATDbits.LATD2                          // LATAbits.LATA5
#define INIT_BUTTON PORTBbits.RB7                           // LATBbits.LATB7
#define SDA_PIN     LATCbits.LATC4                          // not available
#define SCL_PIN     LATCbits.LATC3                          // not available
#define ONEWIRE_PIN LATCbits.LATC5                          // not available

//           Mespelare pins                                 // original Hasselt pins
#define INP1 PORTCbits.RC6                                  // PORTCbits.RC7
#define INP2 PORTBbits.RB6                                  // PORTCbits.RC1
#define INP3 PORTBbits.RB5                                  // PORTCbits.RC2
#define INP4 PORTBbits.RB4                                  // PORTCbits.RC0
#define INP5 PORTBbits.RB1                                  // PORTCbits.RC6
#define INP6 PORTBbits.RB0                                  // PORTCbits.RC5
#define INP7 PORTDbits.RD7                                  // PORTCbits.RC4
#define INP8 PORTDbits.RD7 // works, mapped to above        // PORTCbits.RC3

//           Mespelare pins                                 // original Hasselt pins
#define LED1 LATAbits.LATA0                                 // not available
#define LED2 LATAbits.LATA1                                 // not available
#define LED3 LATAbits.LATA2                                 // not available
#define LED4 LATAbits.LATA3                                 // not available
#define LED5 LATAbits.LATA4                                 // not available
#define LED6 LATAbits.LATA5                                 // not available
#define LED7 LATAbits.LATA6                                 // not available


// * * * Values * * *
#define DEBOUNCE_CHECKS     5 // # checks before a switch is debounced

#define INP_RELEASED                                            0x04
#define INP_PRESSED                                             0x01
#define INP_HOLD                                                0x03
#define INP_HOLD_REPEAT                                         0x05
#define INP_KEY							0x02


//Registers 0x00?0x7F (0-127) are application specific. Registers between 0x80?0xFF (128-255) are reserved for VSCP usage. If the node has implemented the decision matrix it is stored in application register space.
// * * * User defined Registers * * *
#define USER_EEPROM_OFFSET                                      VSCP_EEPROM_END
#define EEPROM_ZONE						0   //page0:0  Zone node belongs to
#define EEPROM_SUBZONE						1   //page0:1  Subzone node belongs to


// David: values below are EEPROM locations (?), not VSCP register values
// REG_STATUS bit field
// 0 : current state output
#define REG_STATUS_OUT1                                         2   //page 0:2
#define REG_STATUS_OUT2                                         3   //page 0:3
#define REG_STATUS_OUT3                                         4   //page 0:4
#define REG_STATUS_OUT4                                         5   //page 0:5
#define REG_STATUS_OUT5                                         6   //page 0:6
#define REG_STATUS_OUT6                                         7   //page 0:7
#define REG_STATUS_OUT7                                         8   //page 0:8
#define REG_STATUS_OUT8                                         9   //page 0:9


#define REG_CONTROL_OUT1                                        30  //page 0:30
#define REG_CONTROL_OUT2                                        31  //page 0:31
#define REG_CONTROL_OUT3                                        32  //page 0:32
#define REG_CONTROL_OUT4                                        33  //page 0:33
#define REG_CONTROL_OUT5                                        34  //page 0:34
#define REG_CONTROL_OUT6                                        35  //page 0:35
#define REG_CONTROL_OUT7                                        36  //page 0:36
#define REG_CONTROL_OUT8                                        37  //page 0:37


#define REG_SUBZONE_OUT1                                        58  //page 0:58
#define REG_SUBZONE_OUT2                                        59  //page 0:59
#define REG_SUBZONE_OUT3                                        60  //page 0:60
#define REG_SUBZONE_OUT4                                        61  //page 0:61
#define REG_SUBZONE_OUT5                                        62  //page 0:62
#define REG_SUBZONE_OUT6                                        63  //page 0:63
#define REG_SUBZONE_OUT7                                        64  //page 0:64
#define REG_SUBZONE_OUT8                                        65  //page 0:65


#define REG_PTIME_OUT1                                          86  //page 0:86 RW Output protection timer in multiples of 64 seconds./n Output will automatically disable after time
#define REG_PTIME_OUT2                                          87  //page 0:87
#define REG_PTIME_OUT3                                          88  //page 0:88
#define REG_PTIME_OUT4                                          89  //page 0:89
#define REG_PTIME_OUT5                                          90  //page 0:90
#define REG_PTIME_OUT6                                          91  //page 0:91
#define REG_PTIME_OUT7                                          92  //page 0:92
#define REG_PTIME_OUT8                                          93  //page 0:93


#define REG_STATUS_PWM1                                         128  //page 1:0 RW Actual status of the PWM output (OUT4 = RB4)
#define REG_STATUS_PWM2                                         129  //page 1:1 RW Actual status of the PWM output. (OUT3 = RB5)

#define REG_CONTROL_PWM1                                        132  //page 1:4
#define REG_CONTROL_PWM2                                        133  //page 1:5

#define REG_SUBZONE_PWM1                                        136  //page 1:8 RW Subzone for PWM.
#define REG_SUBZONE_PWM2                                        137  //page 1:9

#define REG_ASSOUT_PWM1                                         140  //page 1:12 RW Output associated with PWM./n Output will go active if PWM > treshold.
#define REG_ASSOUT_PWM2                                         141  //page 1:13

#define REG_TRESH_PWM1                                          144  //page 1:16 RW Treshold value./n Above this PWM value the associated output will go active.
#define REG_TRESH_PWM2                                          145  //page 1:17

#define REG_MODE_OUT3                                           148  //page 1:20 Operation mode of OUT3. 0x01 = output, 0x02 = PWM, 0x03=SoundPlay
#define REG_MODE_OUT4                                           149  //page 1:21 Operation mode of OUT4. 0x01 = output, 0x02 = PWM,


#define REG_CONTROL_INP1                                        150  //page 1:22
#define REG_CONTROL_INP2                                        151  //page 1:23
#define REG_CONTROL_INP3                                        152  //page 1:24
#define REG_CONTROL_INP4                                        153  //page 1:25
#define REG_CONTROL_INP5                                        154  //page 1:26
#define REG_CONTROL_INP6                                        155  //page 1:27
#define REG_CONTROL_INP7                                        156  //page 1:28
#define REG_CONTROL_INP8                                        157  //page 1:29

#define REG_SUBZONE_INP1                                        158  //page 1:30
#define REG_SUBZONE_INP2                                        159  //page 1:31
#define REG_SUBZONE_INP3                                        160  //page 1:32
#define REG_SUBZONE_INP4                                        161  //page 1:33
#define REG_SUBZONE_INP5                                        162  //page 1:34
#define REG_SUBZONE_INP6                                        163  //page 1:35
#define REG_SUBZONE_INP7                                        164  //page 1:36
#define REG_SUBZONE_INP8                                        165  //page 1:37

#define USER_EEPROM_LAST_POS					191 // ?? VSCPworks reads app registers up to this location

// --------------------------------------------------------------------------------
// TODO: tackle problem with vscpworks incorrectly loading paginated registers

#define REG_DESCION_MATRIX					256	// Start of matrix
#define DESCION_MATRIX_ELEMENTS                                 10       // end of DM =57+(8*8)= 121

// --------------------------------------------------------------------------------


// * * * Actions * * *
#define ACTION_NOOP							0
#define ACTION_ON							1
#define ACTION_OFF							2
#define ACTION_TOGGLE                                                   4
#define ACTION_PWM                                                      8

// * * * Relay control bits * * *
#define RELAY_CONTROLBIT_NOOP				0x01 	// Noop protection
#define RELAY_CONTROLBIT_ALARM				0x02 	// Send alarm on protection time activation
#define RELAY_CONTROLBIT_PROTECTION			0x04 	// Enable protection timer
#define RELAY_CONTROLBIT_ONEVENT			0x08 	// Send ON event
#define RELAY_CONTROLBIT_OFFEVENT			0x10 	// Semd OFF event
#define RELAY_CONTROLBIT_STARTEVENT			0x20 	// Send START event
#define RELAY_CONTROLBIT_STOPEVENT			0x40 	// Sned STOP event
#define RELAY_CONTROLBIT_ENABLED			0x80 	// Relay enabled

#define INP_CONTROLBIT_REDG                             0x01    //Send event at raising edge
#define INP_CONTROLBIT_RPT                              0x02    //send repeatingly events when input active
#define INP_CONTROLBIT_FEDG                             0x04    //Send event at falling edge


// PWM state machine states
#define PWM_OFF								0
#define PWM_UP								1
#define PWMMED_UP							2
#define PWM_DOWN							3
#define PWM_ON								4

#define NoSound     0
#define Beep1       1
#define Beep2       2
#define Song1       3
#define Song2       4

#endif
