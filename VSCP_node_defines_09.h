/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  2014 David Steeman
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
 *    in a product, an acknowledgement in the product documentation would be
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
 * IO definition for Mespelare v9 board
 */

#ifndef MESPELARE
#define MESPELARE

// Firmware version

#define FIRMWARE_MAJOR_VERSION		1
#define FIRMWARE_MINOR_VERSION		2
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

#define OUT1 LATBbits.LATB7                                
#define OUT2 LATAbits.LATA0                                
#define OUT3 LATAbits.LATA1   //PWM5 out -> output or PWM
#define OUT4 LATAbits.LATA2   //PWM1 out -> output or PWM
#define OUT5 LATAbits.LATA3                                 
#define OUT6 LATAbits.LATA5   //D0                            
#define OUT7 LATEbits.LATE0   //D1                                 
#define OUT8 LATEbits.LATE1   //D1                                 
#define OUT9 LATEbits.LATE2
#define OUT10 LATCbits.LATC0
#define OUT11 LATCbits.LATC1
#define OUT12 LATCbits.LATC2
#define OUT13 LATDbits.LATD0
#define OUT14 LATDbits.LATD1

#define INIT_LED    LATDbits.LATD6                          
#define BUZZER      LATDbits.LATD2                          
#define INIT_BUTTON PORTCbits.RC7                           
#define SDA_PIN     LATCbits.LATC4
#define SCL_PIN     LATCbits.LATC3
#define ONEWIRE_PIN LATCbits.LATC5

//defines for ds18s20.h and ow.h drivers
#define OW_LAT      LATCbits.LATC5
#define OW_PIN      PORTCbits.RC5
#define OW_TRIS     TRISCbits.TRISC5

//defines for ds1820.h and types.h drivers
#define DS1820_DATAPIN  ONEWIRE_PIN

#define INP1 PORTDbits.RD7                                  
#define INP2 PORTBbits.RB0                                  
#define INP3 PORTBbits.RB1
#define INP4 PORTBbits.RB4
#define INP5 PORTBbits.RB5
#define INP6 PORTBbits.RB6
#define INP7 PORTCbits.RC6 
#define INP8 PORTCbits.RC6 // works, mapped to above 



// * * * Values * * *
#define DEBOUNCE_CHECKS     5 // # checks before a switch is debounced

#define INP_RELEASED                                            0x04
#define INP_PRESSED                                             0x01
#define INP_HOLD                                                0x03
#define INP_HOLD_REPEAT                                         0x05
#define INP_KEY							0x02


//Registers 0x00?0x7F (0-127) are application specific. Registers between 0x80?0xFF (128-255) are reserved for VSCP usage. If the node has implemented the decision matrix it is stored in application register space.
//format: #define FIRMWARE_REGISTER_NAME    256 * page + register (app registers = 0 - 127)
//VSCP reserved registers are not set here, there's set in eslsewhere in the code //TODO: where?

// * * * User defined Registers * * *
#define USER_EEPROM_OFFSET                                      VSCP_EEPROM_END

#define REG_DESCISION_MATRIX					256* 0 + 0	// Start of matrix, 256 = page 1:0
#define DESCISION_MATRIX_ELEMENTS                               10              // # of DM registers = # elements * 8

#define EEPROM_ZONE						256* 1 + 0   //Zone node belongs to
#define EEPROM_SUBZONE						256* 1 + 1   //Subzone node belongs to

// David: values below are EEPROM locations (?), not VSCP register values
#define REG_STATUS_OUT1                                         256* 1 + 2
#define REG_STATUS_OUT2                                         256* 1 + 3
#define REG_STATUS_OUT3                                         256* 1 + 4
#define REG_STATUS_OUT4                                         256* 1 + 5
#define REG_STATUS_OUT5                                         256* 1 + 6
#define REG_STATUS_OUT6                                         256* 1 + 7
#define REG_STATUS_OUT7                                         256* 1 + 8
#define REG_STATUS_OUT8                                         256* 1 + 9  //  register stored even though Mespelare hardware doesn't use this output

#define REG_CONTROL_OUT1                                        256* 1 + 10
#define REG_CONTROL_OUT2                                        256* 1 + 11
#define REG_CONTROL_OUT3                                        256* 1 + 12
#define REG_CONTROL_OUT4                                        256* 1 + 13
#define REG_CONTROL_OUT5                                        256* 1 + 14
#define REG_CONTROL_OUT6                                        256* 1 + 15
#define REG_CONTROL_OUT7                                        256* 1 + 16
#define REG_CONTROL_OUT8                                        256* 1 + 17  //  register stored even though Mespelare hardware doesn't use this output

#define REG_SUBZONE_OUT1                                        256* 1 + 18
#define REG_SUBZONE_OUT2                                        256* 1 + 19
#define REG_SUBZONE_OUT3                                        256* 1 + 20
#define REG_SUBZONE_OUT4                                        256* 1 + 21
#define REG_SUBZONE_OUT5                                        256* 1 + 22
#define REG_SUBZONE_OUT6                                        256* 1 + 23
#define REG_SUBZONE_OUT7                                        256* 1 + 24
#define REG_SUBZONE_OUT8                                        256* 1 + 25  //  register stored even though Mespelare hardware doesn't use this output

#define REG_CONTROL_INP1                                        256* 1 + 26
#define REG_CONTROL_INP2                                        256* 1 + 27
#define REG_CONTROL_INP3                                        256* 1 + 28
#define REG_CONTROL_INP4                                        256* 1 + 29
#define REG_CONTROL_INP5                                        256* 1 + 30
#define REG_CONTROL_INP6                                        256* 1 + 31
#define REG_CONTROL_INP7                                        256* 1 + 32
#define REG_CONTROL_INP8                                        256* 1 + 33  //  register stored even though Mespelare hardware doesn't use this input

#define REG_SUBZONE_INP1                                        256* 1 + 34
#define REG_SUBZONE_INP2                                        256* 1 + 35
#define REG_SUBZONE_INP3                                        256* 1 + 36
#define REG_SUBZONE_INP4                                        256* 1 + 37
#define REG_SUBZONE_INP5                                        256* 1 + 38
#define REG_SUBZONE_INP6                                        256* 1 + 39
#define REG_SUBZONE_INP7                                        256* 1 + 40
#define REG_SUBZONE_INP8                                        256* 1 + 41  //  register stored even though Mespelare hardware doesn't use this input

#define REG_PTIME_OUT1                                          256* 1 + 42  // RW Output protection timer in multiples of 64 seconds./n Output will automatically disable after time
#define REG_PTIME_OUT2                                          256* 1 + 43
#define REG_PTIME_OUT3                                          256* 1 + 44
#define REG_PTIME_OUT4                                          256* 1 + 45
#define REG_PTIME_OUT5                                          256* 1 + 46
#define REG_PTIME_OUT6                                          256* 1 + 47
#define REG_PTIME_OUT7                                          256* 1 + 48
#define REG_PTIME_OUT8                                          256* 1 + 49   //  register stored even though Mespelare hardware doesn't use this output

#define REG_STATUS_PWM1                                         256* 1 + 50  // RW Actual status of the PWM output (OUT4 = RB4)
#define REG_STATUS_PWM2                                         256* 1 + 51

#define REG_CONTROL_PWM1                                        256* 1 + 52
#define REG_CONTROL_PWM2                                        256* 1 + 53

#define REG_SUBZONE_PWM1                                        256* 1 + 54  // RW Subzone for PWM.
#define REG_SUBZONE_PWM2                                        256* 1 + 55 

#define REG_ASSOUT_PWM1                                         256* 1 + 56  // RW Output associated with PWM./n Output will go active if PWM > treshold.
#define REG_ASSOUT_PWM2                                         256* 1 + 57

#define REG_TRESH_PWM1                                          256* 1 + 58  // RW Treshold value./n Above this PWM value the associated output will go active.
#define REG_TRESH_PWM2                                          256* 1 + 59  

#define REG_MODE_OUT3                                           256* 1 + 60  // Operation mode of OUT3. 0x01 = output, 0x02 = PWM, 0x03=SoundPlay
#define REG_MODE_OUT4                                           256* 1 + 61  // Operation mode of OUT4. 0x01 = output, 0x02 = PWM,

#define USER_EEPROM_LAST_POS                                    55  //includes DM registers ?

// TODO: tackle problem with vscpworks incorrectly loading paginated registers


// --------------------------------------------------------------------------------


// * * * Actions * * *
#define ACTION_NOOP						 0
#define ACTION_ON						 1
#define ACTION_OFF						 2
#define ACTION_TOGGLE                                            4
#define ACTION_PWM                                               8

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
