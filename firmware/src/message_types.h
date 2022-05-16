#include <stdint.h>

#ifndef MESSAGE_TYPES_H
#define	MESSAGE_TYPES_H

#define MEASUREMENT_GA  0x11
#define PARAMUPD_GA  0x21
#define PARAMUPD_AG  0x22
#define FRMWUPD_GA  0x31
#define FRMWUPDREQ_AG  0x32
#define FRMWUPDCONT_AG  0x42
#define CMSGDEV_GA  0x51
#define CMSGDEV_AG  0x52
#define DECMSGDEV_AG 0x62
#define DECMSGDEV_GA 0x61

#define LORA_HEADER_SIZE 6
#define MEASUREMENT_GA_SIZE 66
#define MSG_PARAMUPD_GA_SIZE 11
#define MSG_FRMWUPD_GA_SIZE 1
#define MSG_CMSGDEV_GA_SIZE 29
#define MSG_DECMSGDEV_GA_SIZE 1

#define LORA_FW_PAYLOAD 200

typedef struct
{
    uint8_t type;
    uint8_t version;
    uint32_t timestamp;
}LORA_HEADER;

typedef struct
{
    LORA_HEADER header;
    int16_t GasC;
    uint16_t StdDevG;
    uint16_t VAvgWS;
    uint16_t VAvgWD;
    uint16_t StdDevWD;
    uint16_t Msr;
    uint16_t StdDevSR;
    uint16_t Prec;
    uint16_t Cmp;
    uint16_t SAvgWS;
    uint16_t StdDevWS;
    int8_t AvgWDSin;
    int8_t AvgWDCos;
    long double Lat;
    long double Lgt;
    int16_t Alt;
    int8_t Fq;
    uint8_t Ns;
    uint8_t Hacc;
    uint8_t Vacc;
    uint8_t Tsrc;
    int16_t Tilt;
    uint16_t BattV;
    uint16_t SpV;
    uint32_t UnitStatus;
    uint16_t Curr;
    uint16_t Volt;
    uint16_t MTemp;
    uint16_t MPress;
    uint8_t MHum;
}MSG_MEASUREMENT_GA;

typedef struct
{
    LORA_HEADER header;
    uint16_t ParamN;
    double Param;
    uint8_t Status;
}MSG_PARAMUPD_GA;

typedef struct
{
    LORA_HEADER header;
    uint16_t ParamN;
    double ParamV;
}MSG_PARAMUPD_AG;

typedef struct
{
    LORA_HEADER header;
    uint8_t Status;
}MSG_FRMWUPD_GA;

typedef struct
{
    LORA_HEADER header;
    uint8_t Version;
    uint8_t Region;
    uint8_t Payl;
    uint16_t CRC;
    uint32_t TPN;
}MSG_FRMWUPDDEVREQ_GA;

typedef struct
{
    LORA_HEADER header;
    uint16_t PN;
    uint8_t Payl[LORA_FW_PAYLOAD];
}MSG_FRMWUPDDEVCONT_AG;

typedef struct
{
    LORA_HEADER header;
    uint16_t FrmwGV;
    uint16_t BootGV;
    uint16_t FrmwWV;
    uint16_t FrmwrMV;
    uint8_t Gen;
    uint8_t Utype;
    uint8_t R;
    int16_t Tno;
    double Lat;
    double Lgt;
}MSG_CMSGDEV_GA;

typedef struct
{
    LORA_HEADER header;
}MSG_DECMSGDEV_AG;

typedef struct
{
    LORA_HEADER header;
    uint8_t Status;
}MSG_DECMSGDEV_GA;

typedef struct
{
    LORA_HEADER header;
    uint8_t Status;
}MSG_CMSGDEV_AG;

#endif	/* MESSAGE_TYPES_H */

