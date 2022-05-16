#ifndef MESSAGING_H
#define	MESSAGING_H

#include <stdint.h>
#include "message_types.h"

#define MEAS_GA_VERSION 1
#define MESSAGE_TYPE_MEAS_GA 1

typedef union
{
    long double value;
    uint8_t bytes[8];
}DOUBLE_2_BYTES;

extern MSG_MEASUREMENT_GA measurement;
extern MSG_PARAMUPD_GA parameterUpdate;
extern MSG_FRMWUPD_GA firmwareUpdate;
extern MSG_CMSGDEV_GA commisioning;
extern MSG_DECMSGDEV_GA decommission;

uint16_t measurement2Bytes(uint8_t * data);
uint16_t parameterUpdate2Bytes(uint8_t * data);
uint16_t firmwareUpdate2Bytes(uint8_t * data);
uint16_t commisioning2Bytes(uint8_t * data);
uint16_t decommission2Bytes(uint8_t * data);
unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec);

#endif	/* MESSAGING_H */

