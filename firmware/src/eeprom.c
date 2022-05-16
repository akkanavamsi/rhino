#include "eeprom.h"

EEPROM_DATA eepromData;

void eepromClearBuffer(void);

void eepromInit(void)
{
    eepromData.state = EEPROM_STATE_INIT;
}

void eepromTasks(void)
{
    int i;
    switch(eepromData.state)
    {
        case EEPROM_STATE_INIT:
            eepromClearBuffer();
            EPROM_CS_Clear();
            eepromData.state = EEPROM_STATE_IDLE;
            eepromData.parameters.crc = 0;
            eepromData.parameters.dutyBank = 0;
            eeprom2ram();
//            eepromData.parameters.crc = 0x1234;
//            eepromData.parameters.dutyBank = 1;
//            for(i=0;i<32;i++)
//                eepromData.parameters.word[i] = i;
//            ram2eeprom();
            break;
        case EEPROM_STATE_ERASE://Specif register erase, not used
            break;
        case EEPROM_STATE_ERAL:
            if(!SPI2_IsBusy())
            {
                eepromData.TXdata[0] = EEPROM_CODE_ERAL;
                eepromData.TXsize = 1;
                SPI2_Write(eepromData.TXdata, eepromData.TXsize);
                eepromData.state = EEPROM_STATE_WRITE;
            }
            break;
        case EEPROM_STATE_EWDS:
            if(!SPI2_IsBusy())
            {
                eepromData.TXdata[0] = EEPROM_CODE_EWDS;
                eepromData.TXsize = 1;
                EPROM_CS_Clear();
                eepromData.state = EEPROM_STATE_IDLE;
            }
            break;
        case EEPROM_STATE_EWEN:
            if(!SPI2_IsBusy())
            {
                eepromData.TXdata[0] = EEPROM_CODE_EWEN;
                eepromData.TXsize = 1;
                SPI2_Write(eepromData.TXdata, eepromData.TXsize);
                eepromData.state = EEPROM_STATE_ERAL;
            }
            break;
        case EEPROM_STATE_READ:
            EPROM_CS_Set();
            eepromData.TXdata[0] = EEPROM_CODE_READ;
            eepromData.TXsize = 1;
            eepromData.RXsize = EEPROM_DATA_BUFFER_SIZE;
            SPI2_WriteRead (eepromData.TXdata, eepromData.TXsize, eepromData.parameters.word, eepromData.RXsize);
            eepromData.state = EEPROM_STATE_WAIT;
            break;
        case EEPROM_STATE_WRITE:
            if(!SPI2_IsBusy())
            {
                eepromData.TXdata[0] = EEPROM_CODE_WRITE;
                int size = EEPROM_DATA_BUFFER_SIZE -1;
                for(i=0; i < size; i++)
                    eepromData.TXdata[i +1] = eepromData.parameters.word[i];
                eepromData.TXsize = EEPROM_DATA_BUFFER_SIZE;
                SPI2_Write(eepromData.TXdata, eepromData.TXsize);
                eepromData.state = EEPROM_STATE_EWDS;
            }
            break;
        case EEPROM_STATE_WAIT:
            if(!SPI2_IsBusy())
            {
                EPROM_CS_Clear();
                eepromData.state = EEPROM_STATE_IDLE;
            }
            break;
        case EEPROM_STATE_ERROR:
            break;
        case EEPROM_STATE_IDLE:
            default:
            break;
    }
}

void eepromClearBuffer(void)
{
    memset(eepromData.TXdata,0,EEPROM_DATA_BUFFER_SIZE * 2);
    memset(eepromData.RXdata,0,EEPROM_DATA_BUFFER_SIZE * 2);
}

void ram2eeprom(void)
{
    eepromData.state = EEPROM_STATE_ERAL;
}

void eeprom2ram(void)
{
    eepromData.state = EEPROM_STATE_READ;
}