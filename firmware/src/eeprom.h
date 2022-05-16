/* 
 * File:   eeprom.h
 * Author: RFord7
 *
 * Created on 08 April 2022, 14:50
 */

#ifndef EEPROM_H
#define	EEPROM_H

#define EEPROM_DATA_BUFFER_SIZE 32//No uint reg as eeprom is 16bit wide, bytes is double this value

#include <stdint.h>
#include <string.h>
#include "definitions.h"

#define EEPROM_CODE_ERASE 0b11100000000
#define EEPROM_CODE_ERAL 0b10010000000
#define EEPROM_CODE_EWDS 0b10000000000
#define EEPROM_CODE_EWEN 0b10011000000
#define EEPROM_CODE_READ 0b11000000000
#define EEPROM_CODE_WRITE 0b10100000000
#define EEPROM_CODE_WRAL 0b10001000000

typedef union
{
    uint16_t word[EEPROM_DATA_BUFFER_SIZE];
    struct
    {
        uint16_t dutyBank;
        uint16_t crc;
    };
}DEVICE_PARAMETERS;

typedef enum
{
    EEPROM_STATE_INIT,
    EEPROM_STATE_ERASE,
    EEPROM_STATE_ERAL,
    EEPROM_STATE_EWDS,
    EEPROM_STATE_EWEN,            
    EEPROM_STATE_READ,
    EEPROM_STATE_WRITE,
    EEPROM_STATE_ERROR,
    EEPROM_STATE_WAIT,
    EEPROM_STATE_IDLE,    
}EEPROM_STATES;

typedef struct
{
    EEPROM_STATES state;
    uint16_t TXdata[EEPROM_DATA_BUFFER_SIZE];
    uint16_t RXdata[EEPROM_DATA_BUFFER_SIZE];
    uint16_t TXsize;
    uint16_t RXsize;
    uint16_t address;
    DEVICE_PARAMETERS parameters;
}EEPROM_DATA;

void eepromInit(void);
void eepromTasks(void);
void ram2eeprom(void);  
void eeprom2ram(void);

#endif	/* EEPROM_H */

