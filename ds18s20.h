//downloaded from http://psychoul.com/electronics/1-wire-onewire-c18-library-2

/* Copyright © 2011 Demetris Stavrou
 * version 1.0
 * ---------------------------------
 * Minimal Library for the DS18S20 temperature sensor
 *
 * Just include the ow.h file in your project and define three required
 * names - OW_LAT, OW_PIN, OW_TRIS
 *
 *  ds_get_temp() function will returnt the temperature as is. According to the manufacturer,
 * bit 0 represents the .5 degrees. To get the temperature as an interger you can shift it
 * by 1 to the right.
 *
 * example:
 * mytemperature = ds_get_temp(); // bit 0 represents .5 degress
 * mytemperature = mytemperature >> 1; // bit 0 now represents 1 degree
 *
 * printf("The temperature is %i",mytemperature);
 *
 * 
 * NOTE: This library works for 40MHz Frequency i.e. TCY = 0.1us
 * You can adjust the timings accordingly, at least for now!
 */

#ifndef __DS18S20_H
#define __DS18S20_H

#include "ow.h"

char ds_present(void)
{
    return ow_reset();
}

char ds_get_temp(void) {
    char temp_low;
    char temp_high;
    int temperature=0;

    ow_reset();
    Delay100TCYx(30);
    ow_write_byte(0xCC);
    Delay100TCYx(1);
    ow_write_byte(0x44);
    Delay10KTCYx(255);
    Delay10KTCYx(255);
    Delay10KTCYx(255);
    ow_reset();
    Delay100TCYx(30);
    ow_write_byte(0xCC);
    Delay100TCYx(1);
    ow_write_byte(0xBE);
    Delay100TCYx(1);
    temp_low = ow_read_byte();
    Delay10TCYx(1);
    temp_high = ow_read_byte();
    ow_reset();
    temperature = temperature | temp_low;
    temperature = temperature | (temp_high << 8);
    return temperature;
}

#endif