/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 * 	2014-12-06 David Steeman
 * 	david@steeman.be
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
*/
/*
 * IO definition for Mespelare
 */

//#ifndef HASSELT_v2
//#define HASSELT_v2
#ifndef MESPELARE_v1
#define MESPELARE_v1

// Firmware version

#define FIRMWARE_MAJOR_VERSION		1
#define FIRMWARE_MINOR_VERSION		1
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

//           Mespelare pins                               
#define OUT1 LATEbits.LATE1                               
#define OUT2 LATEbits.LATE2                               
#define OUT3 LATCbits.LATC0                               //PWM5 out -> output or PWM
#define OUT4 LATCbits.LATC1                               //PWM1 out -> output or PWM
#define OUT5 LATCbits.LATC2                               
#define OUT6 LATDbits.LATD0                               
#define OUT7 LATDbits.LATD1                               
#define OUT8 LATDbits.LATD1                               

//                  Mespelare pins                        
#define INIT_LED    LATDbits.LATD3                        
#define BUZZER      LATDbits.LATD2                        
#define INIT_BUTTON PORTBbits.RB7                         
#define SDA_PIN     LATCbits.LATC4                        
#define SCL_PIN     LATCbits.LATC3                        
#define ONEWIRE_PIN LATCbits.LATC5                        

//           Mespelare pins                               
#define INP1 PORTCbits.RC6                                
#define INP2 PORTBbits.RB6                                
#define INP3 PORTBbits.RB5                                
#define INP4 PORTBbits.RB4                                
#define INP5 PORTBbits.RB1                                  
#define INP6 PORTBbits.RB0                                  
#define INP7 PORTDbits.RD7                                  
#define INP8 PORTDbits.RD7 // this pin is not available on the MEspelare board but is used in the Hasselt firmware, so mapped to above (works)

//           Mespelare pins
#define LED1 LATAbits.LATA0
#define LED2 LATAbits.LATA1
#define LED3 LATAbits.LATA2
#define LED4 LATAbits.LATA3
//#define LED5 LATAbits.LATA4  //pin does not exist on 18F45K80? //TODO: adapt to board v7
#define LED5 LATAbits.LATA5
#define LED6 LATAbits.LATA5
#define LED7 LATAbits.LATA6


// * * * Values * * *
#define DEBOUNCE_CHECKS     5 // # checks before a switch is debounced

#define INP_RELEASED                                            0x04
#define INP_PRESSED                                             0x01
#define INP_HOLD                                                0x03
#define INP_HOLD_REPEAT                                         0x05
#define INP_KEY							0x02


//Registers 0x00?0x7F (0-127) are application specific. Registers between 0x80?0xFF (128-255) are reserved for VSCP usage. If the node has implemented the decision matrix it is stored in application register space.
//format: #define FIRMWARE_REGISTER_NAME    256 * page + register (app registers = 0 - 127)
//Application registers defined here should always start at page 0 register 0
//VSCP reserved registers are not set here, there's set in eslsewhere in the code //TODO: where?

// * * * User defined Registers * * *
//VSCP_EEPROM_END = last register where VSCP reserved registers are stored

#define REG_DESCISION_MATRIX					0	// Start of matrix, 0 = page 0:0
#define DESCISION_MATRIX_ELEMENTS                               10
// # of DM registers = # elements * 8
// last DM register = REG_DECISION_MATRIX + (DECISION_MATRIX_ELEMENTS * 8)- 1

#define EEPROM_ZONE                                             (int)80   //Zone node belongs to
#define EEPROM_SUBZONE                                          81   //Subzone node belongs to

#define REG_STATUS_OUT1                                         82
#define REG_STATUS_OUT2                                         83
#define REG_STATUS_OUT3                                         84
#define REG_STATUS_OUT4                                         85
#define REG_STATUS_OUT5                                         86
#define REG_STATUS_OUT6                                         87
#define REG_STATUS_OUT7                                         88
#define REG_STATUS_OUT8                                         89  //  register stored even though Mespelare hardware doesn't use this output

#define REG_CONTROL_OUT1                                        90
#define REG_CONTROL_OUT2                                        91
#define REG_CONTROL_OUT3                                        92
#define REG_CONTROL_OUT4                                        93
#define REG_CONTROL_OUT5                                        94
#define REG_CONTROL_OUT6                                        95
#define REG_CONTROL_OUT7                                        96
#define REG_CONTROL_OUT8                                        97  //  register stored even though Mespelare hardware doesn't use this output

#define REG_SUBZONE_OUT1                                        98
#define REG_SUBZONE_OUT2                                        99
#define REG_SUBZONE_OUT3                                        100
#define REG_SUBZONE_OUT4                                        101
#define REG_SUBZONE_OUT5                                        102
#define REG_SUBZONE_OUT6                                        103
#define REG_SUBZONE_OUT7                                        104
#define REG_SUBZONE_OUT8                                        105  //  register stored even though Mespelare hardware doesn't use this output

#define REG_CONTROL_INP1                                        106
#define REG_CONTROL_INP2                                        107
#define REG_CONTROL_INP3                                        108
#define REG_CONTROL_INP4                                        109
#define REG_CONTROL_INP5                                        110
#define REG_CONTROL_INP6                                        111
#define REG_CONTROL_INP7                                        112
#define REG_CONTROL_INP8                                        113 //  register stored even though Mespelare hardware doesn't use this input

#define REG_SUBZONE_INP1                                        114
#define REG_SUBZONE_INP2                                        115
#define REG_SUBZONE_INP3                                        116
#define REG_SUBZONE_INP4                                        117
#define REG_SUBZONE_INP5                                        118
#define REG_SUBZONE_INP6                                        119
#define REG_SUBZONE_INP7                                        120
#define REG_SUBZONE_INP8                                        121  //  register stored even though Mespelare hardware doesn't use this input

//page 1
#define REG_PTIME_OUT1                                          256  // RW Output protection timer in multiples of 64 seconds./n Output will automatically disable after time
#define REG_PTIME_OUT2                                          257
#define REG_PTIME_OUT3                                          258
#define REG_PTIME_OUT4                                          259
#define REG_PTIME_OUT5                                          260
#define REG_PTIME_OUT6                                          261
#define REG_PTIME_OUT7                                          262
#define REG_PTIME_OUT8                                          263   //  register stored even though Mespelare hardware doesn't use this output

#define REG_STATUS_PWM1                                         264  // RW Actual status of the PWM output (OUT4 = RB4)
#define REG_STATUS_PWM2                                         265

#define REG_CONTROL_PWM1                                        266
#define REG_CONTROL_PWM2                                        267

#define REG_SUBZONE_PWM1                                        268  // RW Subzone for PWM.
#define REG_SUBZONE_PWM2                                        269

#define REG_ASSOUT_PWM1                                         270  // RW Output associated with PWM./n Output will go active if PWM > treshold.
#define REG_ASSOUT_PWM2                                         271

#define REG_TRESH_PWM1                                          272  // RW Treshold value./n Above this PWM value the associated output will go active.
#define REG_TRESH_PWM2                                          273

#define REG_MODE_OUT3                                           274  // Operation mode of OUT3. 0x01 = output, 0x02 = PWM, 0x03=SoundPlay
#define REG_MODE_OUT4                                           275  // Operation mode of OUT4. 0x01 = output, 0x02 = PWM,

#define USER_EEPROM_LAST_POS                                    276  // Should be equal to the last defined register

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
