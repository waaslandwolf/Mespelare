// File:  main.h

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 * 	Version: See project header
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 1995-2005 Ake Hedman, eurosource
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

#ifndef MAIN_H
#define MAIN_H

//Defines
#define	TRUE			1
#define	FALSE			0

#define TIMER2_RELOAD_VALUE		125 //adapted to Antwerp

//
// Size for array used for temp mean calculations
//
#define TEMPERATURE_ARRAY_SIZE 	0x0A


// EEPROM Storage
#define VSCP_EEPROM_BOOTLOADER_FLAG			0x00	// Reserved for bootloader	 

#define VSCP_EEPROM_NICKNAME				0x01	// Persistant nickname id storage
#define VSCP_EEPROM_SEGMENT_CRC				0x02	// Persistant segment crc storage
#define VSCP_EEPROM_CONTROL				0x03	// Persistant control byte

#define VSCP_EEPROM_REG_USERID				0x04
#define VSCP_EEPROM_REG_USERID1				0x05
#define VSCP_EEPROM_REG_USERID2				0x06
#define VSCP_EEPROM_REG_USERID3				0x07
#define VSCP_EEPROM_REG_USERID4				0x08

// The following can be stored in flash or eeprom

#define VSCP_EEPROM_REG_MANUFACTUR_ID0                  0x09
#define VSCP_EEPROM_REG_MANUFACTUR_ID1          	0x0A
#define VSCP_EEPROM_REG_MANUFACTUR_ID2          	0x0B
#define VSCP_EEPROM_REG_MANUFACTUR_ID3          	0x0C

#define VSCP_EEPROM_REG_MANUFACTUR_SUBID0       	0x0D
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID1               0x0E
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID2               0x0F
#define VSCP_EEPROM_REG_MANUFACTUR_SUBID3               0x10

// The following can be stored in program ROM (recommended) or in EEPROM 

#define VSCP_EEPROM_REG_GUID				0x11	// Start of GUID MSB	
#define VSCP_EEPROM_REG_DEVICE_URL			0x21	// Start of Device URL storage
//#define VSCP_EEPROM_END                                 65	// marks end of VSCP EEPROM usage
#define VSCP_EEPROM_END                                 0x41	// marks end of VSCP EEPROM usage
								// = next free position (so last used register + 1)


// Function Prototypes

void doWork( void );
void init( void );
void init_app_ram( void );
void init_app_eeprom( void ); 
void read_app_register( unsigned char reg );
void write_app_register( unsigned char reg, unsigned char val );
void sendDMatrixInfo( void );
void SendInformationEvent( unsigned char idx, unsigned char eventClass, unsigned char eventTypeId);
void SendInformationEventData( unsigned char idx, unsigned char eventClass, unsigned char eventTypeId, unsigned char data);


void SendProbeAck (unsigned char INPtime);

void doDM( void );

void doActionOn( unsigned char dmflags, unsigned char arg );
void doActionOff( unsigned char dmflags, unsigned char arg );
void doActionToggle( unsigned char dmflags, unsigned char arg );
void doActionPWM( unsigned char dmflags, unsigned char arg );

void doApplicationOneSecondWork( void );

unsigned char DebounceINP1(void);
unsigned char DebounceINP2(void);
unsigned char DebounceINP3(void);
unsigned char DebounceINP4(void);
unsigned char DebounceINP5(void);
unsigned char DebounceINP6(void);
unsigned char DebounceINP7(void);
unsigned char DebounceINP8(void);

void SetOut(uint8_t output, BOOL value);
uint8_t ReadOut(uint8_t output);
uint8_t ReadPWM(uint8_t pwm);
void SetPWM(uint8_t pwm, uint8_t value);
void ProcessInputs(void);
void PlaySound(void);

/*!
	Send Extended ID CAN frame
	@param id CAN extended ID for frame.
	@param size Number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t sendCANFrame( uint32_t id, uint8_t size, uint8_t *pData );

/*!
	Get extended ID CAN frame
	@param pid Pointer to CAN extended ID for frame.
	@param psize Pointer to number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t getCANFrame( uint32_t *pid, uint8_t *psize, uint8_t *pData );



#endif
