/* 
 * File:   mdot.h
 * Author: RFord7
 *
 * Created on 18 January 2022, 14:01
 */

#ifndef MDOT_H
#define	MDOT_H


#include<stdint.h>
#include <stdbool.h>

#define MDOT_TX_BUFFER 16
#define MDOT_RX_BUFFER 256
#define MDOT_CMD_STRING_BUFFER 32
#define MDOT_REQ_SIZE 4
#define MDOT_TIME_OUT 500

#define MDOT_FRAME_ERROR -1
#define MDOT_FRAME_OK 1
#define MDOT_FRAME_EMPTY 2

//typedef struct
//{
//    uint8_t V;
//    uint8_t BattStatus;
//    uint8_t Ns;
//    uint8_t Tsrc;
//    uint8_t Mhum;   
//    int8_t Fq;
//    uint16_t GasC;
//    uint16_t stdDevG;
//    uint16_t AvgSW;
//    uint16_t AvgDW;
//    uint16_t StdDevW;
//    uint16_t StdDevDir;
//    uint16_t Msr;
//    uint16_t StdDevSr;
//    uint16_t Prec;
//    uint16_t Cmp;
//    int16_t Alt;
//    int16_t Tilt;
//    uint16_t UnitStatus;
//    uint16_t Curr;
//    uint16_t Volt;
//    uint16_t Mtemp;
//    uint16_t Mpress;
//    uint32_t Id;
//    uint32_t Tstp;
//    double Lat;
//    double Lgt;   
//}NET_PAYLOAD;
//
//typedef union
//{
//    char bytes[sizeof(NET_PAYLOAD)];
//    NET_PAYLOAD values;
//}LW_PAYLOAD;

typedef enum
{
    MDOT_NONE = 0,
    MDOT_TEST,
    MDOT_SET_ECHO,
    MDOT_SAVE,
    MDOT_SET_PING_TIME,
    MDOT_GET_TIME,
    MDOT_GET_BEACON,
    MDOT_SET_BAT,
    MDOT_GET_EUI,
    MDOT_SET_EUI,
    MDOT_SET_NETWORK,
    MDOT_GET_NETWORK,
    MDOT_JOIN_NETWORK,
    MDOT_SET_NET_EUI,
    MDOT_GET_NET_EUI,
    MDOT_SET_NET_PASS,
    MDOT_GET_NET_PASS,
    MDOT_GET_JOIN_STATUS,
    MDOT_SET_CLASS,
    MDOT_SEND,
    MDOT_SENDB,
    MDOT_RECEIVE,
    MDOT_GET_DATA_PENDING,
    MDOT_ESCAPE_SERIAL,
    MDOT_VERSION,
    MDOT_TXDR,
    MDOT_CHM,
    MDOT_OPEN_REC_WIN,
    MDOT_SET_ACK,
    MDOT_SET_JR,    
}MDOT_COMMAND;

typedef enum
{
    MDOT_STATE_INIT,
    MDOT_STATE_WAIT,
    MDOT_STATE_IDLE,
    MDOT_STATE_COMMAND,
    MDOT_STATE_PROCESS,
    MDOT_STATE_ERROR,
    MDOT_STATE_TIMEOUT,
}MDOT_STATE;

typedef struct
{
    MDOT_COMMAND command;
    MDOT_STATE state;
    char sendBuffer[MDOT_TX_BUFFER];
    char receiveBuffer[MDOT_RX_BUFFER];
}MDOT_DATA;

void mDotInit(void);
void mDotTasks(void);
void mDotSetEcho(bool value);
void getMdotMessage(void);
void testMessageGA(void);
void mDotCounters(void);
int parseLoraFrame(void);
void sendMeasurementGA(void);

extern MDOT_DATA mDotData;

#endif	/* MDOT_H */

