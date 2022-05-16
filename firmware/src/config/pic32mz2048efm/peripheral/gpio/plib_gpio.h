/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for HB pin ***/
#define HB_Set()               (LATGSET = (1<<15))
#define HB_Clear()             (LATGCLR = (1<<15))
#define HB_Toggle()            (LATGINV= (1<<15))
#define HB_OutputEnable()      (TRISGCLR = (1<<15))
#define HB_InputEnable()       (TRISGSET = (1<<15))
#define HB_Get()               ((PORTG >> 15) & 0x1)
#define HB_PIN                  GPIO_PIN_RG15

/*** Macros for GPIO_23 pin ***/
#define GPIO_23_Set()               (LATASET = (1<<5))
#define GPIO_23_Clear()             (LATACLR = (1<<5))
#define GPIO_23_Toggle()            (LATAINV= (1<<5))
#define GPIO_23_OutputEnable()      (TRISACLR = (1<<5))
#define GPIO_23_InputEnable()       (TRISASET = (1<<5))
#define GPIO_23_Get()               ((PORTA >> 5) & 0x1)
#define GPIO_23_PIN                  GPIO_PIN_RA5

/*** Macros for U5_TX pin ***/
#define U5_TX_Get()               ((PORTE >> 5) & 0x1)
#define U5_TX_PIN                  GPIO_PIN_RE5

/*** Macros for GPIO_24 pin ***/
#define GPIO_24_Set()               (LATESET = (1<<6))
#define GPIO_24_Clear()             (LATECLR = (1<<6))
#define GPIO_24_Toggle()            (LATEINV= (1<<6))
#define GPIO_24_OutputEnable()      (TRISECLR = (1<<6))
#define GPIO_24_InputEnable()       (TRISESET = (1<<6))
#define GPIO_24_Get()               ((PORTE >> 6) & 0x1)
#define GPIO_24_PIN                  GPIO_PIN_RE6

/*** Macros for U5_EN pin ***/
#define U5_EN_Set()               (LATESET = (1<<7))
#define U5_EN_Clear()             (LATECLR = (1<<7))
#define U5_EN_Toggle()            (LATEINV= (1<<7))
#define U5_EN_OutputEnable()      (TRISECLR = (1<<7))
#define U5_EN_InputEnable()       (TRISESET = (1<<7))
#define U5_EN_Get()               ((PORTE >> 7) & 0x1)
#define U5_EN_PIN                  GPIO_PIN_RE7

/*** Macros for U5_RX pin ***/
#define U5_RX_Get()               ((PORTC >> 1) & 0x1)
#define U5_RX_PIN                  GPIO_PIN_RC1

/*** Macros for DI2 pin ***/
#define DI2_Set()               (LATJSET = (1<<8))
#define DI2_Clear()             (LATJCLR = (1<<8))
#define DI2_Toggle()            (LATJINV= (1<<8))
#define DI2_OutputEnable()      (TRISJCLR = (1<<8))
#define DI2_InputEnable()       (TRISJSET = (1<<8))
#define DI2_Get()               ((PORTJ >> 8) & 0x1)
#define DI2_PIN                  GPIO_PIN_RJ8
#define DI2_InterruptEnable()   (CNENJSET = (1<<8))
#define DI2_InterruptDisable()  (CNENJCLR = (1<<8))

/*** Macros for DI1 pin ***/
#define DI1_Set()               (LATJSET = (1<<9))
#define DI1_Clear()             (LATJCLR = (1<<9))
#define DI1_Toggle()            (LATJINV= (1<<9))
#define DI1_OutputEnable()      (TRISJCLR = (1<<9))
#define DI1_InputEnable()       (TRISJSET = (1<<9))
#define DI1_Get()               ((PORTJ >> 9) & 0x1)
#define DI1_PIN                  GPIO_PIN_RJ9
#define DI1_InterruptEnable()   (CNENJSET = (1<<9))
#define DI1_InterruptDisable()  (CNENJCLR = (1<<9))

/*** Macros for GPIO_28 pin ***/
#define GPIO_28_Set()               (LATJSET = (1<<12))
#define GPIO_28_Clear()             (LATJCLR = (1<<12))
#define GPIO_28_Toggle()            (LATJINV= (1<<12))
#define GPIO_28_OutputEnable()      (TRISJCLR = (1<<12))
#define GPIO_28_InputEnable()       (TRISJSET = (1<<12))
#define GPIO_28_Get()               ((PORTJ >> 12) & 0x1)
#define GPIO_28_PIN                  GPIO_PIN_RJ12

/*** Macros for GPIO_29 pin ***/
#define GPIO_29_Set()               (LATJSET = (1<<10))
#define GPIO_29_Clear()             (LATJCLR = (1<<10))
#define GPIO_29_Toggle()            (LATJINV= (1<<10))
#define GPIO_29_OutputEnable()      (TRISJCLR = (1<<10))
#define GPIO_29_InputEnable()       (TRISJSET = (1<<10))
#define GPIO_29_Get()               ((PORTJ >> 10) & 0x1)
#define GPIO_29_PIN                  GPIO_PIN_RJ10

/*** Macros for NV_CS pin ***/
#define NV_CS_Set()               (LATCSET = (1<<2))
#define NV_CS_Clear()             (LATCCLR = (1<<2))
#define NV_CS_Toggle()            (LATCINV= (1<<2))
#define NV_CS_OutputEnable()      (TRISCCLR = (1<<2))
#define NV_CS_InputEnable()       (TRISCSET = (1<<2))
#define NV_CS_Get()               ((PORTC >> 2) & 0x1)
#define NV_CS_PIN                  GPIO_PIN_RC2

/*** Macros for FAN2_PWM pin ***/
#define FAN2_PWM_Set()               (LATCSET = (1<<3))
#define FAN2_PWM_Clear()             (LATCCLR = (1<<3))
#define FAN2_PWM_Toggle()            (LATCINV= (1<<3))
#define FAN2_PWM_OutputEnable()      (TRISCCLR = (1<<3))
#define FAN2_PWM_InputEnable()       (TRISCSET = (1<<3))
#define FAN2_PWM_Get()               ((PORTC >> 3) & 0x1)
#define FAN2_PWM_PIN                  GPIO_PIN_RC3

/*** Macros for MOSI1 pin ***/
#define MOSI1_Get()               ((PORTC >> 4) & 0x1)
#define MOSI1_PIN                  GPIO_PIN_RC4

/*** Macros for SCK2 pin ***/
#define SCK2_Get()               ((PORTG >> 6) & 0x1)
#define SCK2_PIN                  GPIO_PIN_RG6

/*** Macros for SDA4 pin ***/
#define SDA4_Get()               ((PORTG >> 7) & 0x1)
#define SDA4_PIN                  GPIO_PIN_RG7

/*** Macros for SCL4 pin ***/
#define SCL4_Get()               ((PORTG >> 8) & 0x1)
#define SCL4_PIN                  GPIO_PIN_RG8

/*** Macros for GPIO_6 pin ***/
#define GPIO_6_Set()               (LATKSET = (1<<0))
#define GPIO_6_Clear()             (LATKCLR = (1<<0))
#define GPIO_6_Toggle()            (LATKINV= (1<<0))
#define GPIO_6_OutputEnable()      (TRISKCLR = (1<<0))
#define GPIO_6_InputEnable()       (TRISKSET = (1<<0))
#define GPIO_6_Get()               ((PORTK >> 0) & 0x1)
#define GPIO_6_PIN                  GPIO_PIN_RK0

/*** Macros for U6_RX pin ***/
#define U6_RX_Get()               ((PORTG >> 9) & 0x1)
#define U6_RX_PIN                  GPIO_PIN_RG9

/*** Macros for U2_EN pin ***/
#define U2_EN_Set()               (LATASET = (1<<0))
#define U2_EN_Clear()             (LATACLR = (1<<0))
#define U2_EN_Toggle()            (LATAINV= (1<<0))
#define U2_EN_OutputEnable()      (TRISACLR = (1<<0))
#define U2_EN_InputEnable()       (TRISASET = (1<<0))
#define U2_EN_Get()               ((PORTA >> 0) & 0x1)
#define U2_EN_PIN                  GPIO_PIN_RA0

/*** Macros for GPIO_14 pin ***/
#define GPIO_14_Set()               (LATESET = (1<<8))
#define GPIO_14_Clear()             (LATECLR = (1<<8))
#define GPIO_14_Toggle()            (LATEINV= (1<<8))
#define GPIO_14_OutputEnable()      (TRISECLR = (1<<8))
#define GPIO_14_InputEnable()       (TRISESET = (1<<8))
#define GPIO_14_Get()               ((PORTE >> 8) & 0x1)
#define GPIO_14_PIN                  GPIO_PIN_RE8

/*** Macros for FAN1_PWM pin ***/
#define FAN1_PWM_Set()               (LATESET = (1<<9))
#define FAN1_PWM_Clear()             (LATECLR = (1<<9))
#define FAN1_PWM_Toggle()            (LATEINV= (1<<9))
#define FAN1_PWM_OutputEnable()      (TRISECLR = (1<<9))
#define FAN1_PWM_InputEnable()       (TRISESET = (1<<9))
#define FAN1_PWM_Get()               ((PORTE >> 9) & 0x1)
#define FAN1_PWM_PIN                  GPIO_PIN_RE9

/*** Macros for U3_TX pin ***/
#define U3_TX_Get()               ((PORTB >> 5) & 0x1)
#define U3_TX_PIN                  GPIO_PIN_RB5

/*** Macros for U3_EN pin ***/
#define U3_EN_Set()               (LATBSET = (1<<4))
#define U3_EN_Clear()             (LATBCLR = (1<<4))
#define U3_EN_Toggle()            (LATBINV= (1<<4))
#define U3_EN_OutputEnable()      (TRISBCLR = (1<<4))
#define U3_EN_InputEnable()       (TRISBSET = (1<<4))
#define U3_EN_Get()               ((PORTB >> 4) & 0x1)
#define U3_EN_PIN                  GPIO_PIN_RB4

/*** Macros for FIG_3V3_EN pin ***/
#define FIG_3V3_EN_Set()               (LATJSET = (1<<11))
#define FIG_3V3_EN_Clear()             (LATJCLR = (1<<11))
#define FIG_3V3_EN_Toggle()            (LATJINV= (1<<11))
#define FIG_3V3_EN_OutputEnable()      (TRISJCLR = (1<<11))
#define FIG_3V3_EN_InputEnable()       (TRISJSET = (1<<11))
#define FIG_3V3_EN_Get()               ((PORTJ >> 11) & 0x1)
#define FIG_3V3_EN_PIN                  GPIO_PIN_RJ11

/*** Macros for DO2 pin ***/
#define DO2_Set()               (LATJSET = (1<<13))
#define DO2_Clear()             (LATJCLR = (1<<13))
#define DO2_Toggle()            (LATJINV= (1<<13))
#define DO2_OutputEnable()      (TRISJCLR = (1<<13))
#define DO2_InputEnable()       (TRISJSET = (1<<13))
#define DO2_Get()               ((PORTJ >> 13) & 0x1)
#define DO2_PIN                  GPIO_PIN_RJ13

/*** Macros for DO1 pin ***/
#define DO1_Set()               (LATJSET = (1<<14))
#define DO1_Clear()             (LATJCLR = (1<<14))
#define DO1_Toggle()            (LATJINV= (1<<14))
#define DO1_OutputEnable()      (TRISJCLR = (1<<14))
#define DO1_InputEnable()       (TRISJSET = (1<<14))
#define DO1_Get()               ((PORTJ >> 14) & 0x1)
#define DO1_PIN                  GPIO_PIN_RJ14

/*** Macros for FIG_EN pin ***/
#define FIG_EN_Set()               (LATJSET = (1<<15))
#define FIG_EN_Clear()             (LATJCLR = (1<<15))
#define FIG_EN_Toggle()            (LATJINV= (1<<15))
#define FIG_EN_OutputEnable()      (TRISJCLR = (1<<15))
#define FIG_EN_InputEnable()       (TRISJSET = (1<<15))
#define FIG_EN_Get()               ((PORTJ >> 15) & 0x1)
#define FIG_EN_PIN                  GPIO_PIN_RJ15

/*** Macros for GPIO_13 pin ***/
#define GPIO_13_Set()               (LATBSET = (1<<3))
#define GPIO_13_Clear()             (LATBCLR = (1<<3))
#define GPIO_13_Toggle()            (LATBINV= (1<<3))
#define GPIO_13_OutputEnable()      (TRISBCLR = (1<<3))
#define GPIO_13_InputEnable()       (TRISBSET = (1<<3))
#define GPIO_13_Get()               ((PORTB >> 3) & 0x1)
#define GPIO_13_PIN                  GPIO_PIN_RB3

/*** Macros for AD5940_INTRPT pin ***/
#define AD5940_INTRPT_Set()               (LATBSET = (1<<2))
#define AD5940_INTRPT_Clear()             (LATBCLR = (1<<2))
#define AD5940_INTRPT_Toggle()            (LATBINV= (1<<2))
#define AD5940_INTRPT_OutputEnable()      (TRISBCLR = (1<<2))
#define AD5940_INTRPT_InputEnable()       (TRISBSET = (1<<2))
#define AD5940_INTRPT_Get()               ((PORTB >> 2) & 0x1)
#define AD5940_INTRPT_PIN                  GPIO_PIN_RB2
#define AD5940_INTRPT_InterruptEnable()   (CNENBSET = (1<<2))
#define AD5940_INTRPT_InterruptDisable()  (CNENBCLR = (1<<2))

/*** Macros for FAN2_EN pin ***/
#define FAN2_EN_Set()               (LATASET = (1<<10))
#define FAN2_EN_Clear()             (LATACLR = (1<<10))
#define FAN2_EN_Toggle()            (LATAINV= (1<<10))
#define FAN2_EN_OutputEnable()      (TRISACLR = (1<<10))
#define FAN2_EN_InputEnable()       (TRISASET = (1<<10))
#define FAN2_EN_Get()               ((PORTA >> 10) & 0x1)
#define FAN2_EN_PIN                  GPIO_PIN_RA10

/*** Macros for FAN2_FB pin ***/
#define FAN2_FB_Set()               (LATHSET = (1<<0))
#define FAN2_FB_Clear()             (LATHCLR = (1<<0))
#define FAN2_FB_Toggle()            (LATHINV= (1<<0))
#define FAN2_FB_OutputEnable()      (TRISHCLR = (1<<0))
#define FAN2_FB_InputEnable()       (TRISHSET = (1<<0))
#define FAN2_FB_Get()               ((PORTH >> 0) & 0x1)
#define FAN2_FB_PIN                  GPIO_PIN_RH0

/*** Macros for FAN1_EN pin ***/
#define FAN1_EN_Set()               (LATHSET = (1<<1))
#define FAN1_EN_Clear()             (LATHCLR = (1<<1))
#define FAN1_EN_Toggle()            (LATHINV= (1<<1))
#define FAN1_EN_OutputEnable()      (TRISHCLR = (1<<1))
#define FAN1_EN_InputEnable()       (TRISHSET = (1<<1))
#define FAN1_EN_Get()               ((PORTH >> 1) & 0x1)
#define FAN1_EN_PIN                  GPIO_PIN_RH1

/*** Macros for FAN1_FB pin ***/
#define FAN1_FB_Set()               (LATHSET = (1<<2))
#define FAN1_FB_Clear()             (LATHCLR = (1<<2))
#define FAN1_FB_Toggle()            (LATHINV= (1<<2))
#define FAN1_FB_OutputEnable()      (TRISHCLR = (1<<2))
#define FAN1_FB_InputEnable()       (TRISHSET = (1<<2))
#define FAN1_FB_Get()               ((PORTH >> 2) & 0x1)
#define FAN1_FB_PIN                  GPIO_PIN_RH2

/*** Macros for FAT_CTL pin ***/
#define FAT_CTL_Set()               (LATHSET = (1<<3))
#define FAT_CTL_Clear()             (LATHCLR = (1<<3))
#define FAT_CTL_Toggle()            (LATHINV= (1<<3))
#define FAT_CTL_OutputEnable()      (TRISHCLR = (1<<3))
#define FAT_CTL_InputEnable()       (TRISHSET = (1<<3))
#define FAT_CTL_Get()               ((PORTH >> 3) & 0x1)
#define FAT_CTL_PIN                  GPIO_PIN_RH3

/*** Macros for V12_EN pin ***/
#define V12_EN_Set()               (LATBSET = (1<<8))
#define V12_EN_Clear()             (LATBCLR = (1<<8))
#define V12_EN_Toggle()            (LATBINV= (1<<8))
#define V12_EN_OutputEnable()      (TRISBCLR = (1<<8))
#define V12_EN_InputEnable()       (TRISBSET = (1<<8))
#define V12_EN_Get()               ((PORTB >> 8) & 0x1)
#define V12_EN_PIN                  GPIO_PIN_RB8

/*** Macros for V12_STATE pin ***/
#define V12_STATE_Set()               (LATBSET = (1<<11))
#define V12_STATE_Clear()             (LATBCLR = (1<<11))
#define V12_STATE_Toggle()            (LATBINV= (1<<11))
#define V12_STATE_OutputEnable()      (TRISBCLR = (1<<11))
#define V12_STATE_InputEnable()       (TRISBSET = (1<<11))
#define V12_STATE_Get()               ((PORTB >> 11) & 0x1)
#define V12_STATE_PIN                  GPIO_PIN_RB11

/*** Macros for LS_N pin ***/
#define LS_N_Set()               (LATKSET = (1<<1))
#define LS_N_Clear()             (LATKCLR = (1<<1))
#define LS_N_Toggle()            (LATKINV= (1<<1))
#define LS_N_OutputEnable()      (TRISKCLR = (1<<1))
#define LS_N_InputEnable()       (TRISKSET = (1<<1))
#define LS_N_Get()               ((PORTK >> 1) & 0x1)
#define LS_N_PIN                  GPIO_PIN_RK1

/*** Macros for LS pin ***/
#define LS_Set()               (LATKSET = (1<<2))
#define LS_Clear()             (LATKCLR = (1<<2))
#define LS_Toggle()            (LATKINV= (1<<2))
#define LS_OutputEnable()      (TRISKCLR = (1<<2))
#define LS_InputEnable()       (TRISKSET = (1<<2))
#define LS_Get()               ((PORTK >> 2) & 0x1)
#define LS_PIN                  GPIO_PIN_RK2

/*** Macros for GPIO_26 pin ***/
#define GPIO_26_Set()               (LATKSET = (1<<3))
#define GPIO_26_Clear()             (LATKCLR = (1<<3))
#define GPIO_26_Toggle()            (LATKINV= (1<<3))
#define GPIO_26_OutputEnable()      (TRISKCLR = (1<<3))
#define GPIO_26_InputEnable()       (TRISKSET = (1<<3))
#define GPIO_26_Get()               ((PORTK >> 3) & 0x1)
#define GPIO_26_PIN                  GPIO_PIN_RK3

/*** Macros for AD5940_CS pin ***/
#define AD5940_CS_Set()               (LATASET = (1<<1))
#define AD5940_CS_Clear()             (LATACLR = (1<<1))
#define AD5940_CS_Toggle()            (LATAINV= (1<<1))
#define AD5940_CS_OutputEnable()      (TRISACLR = (1<<1))
#define AD5940_CS_InputEnable()       (TRISASET = (1<<1))
#define AD5940_CS_Get()               ((PORTA >> 1) & 0x1)
#define AD5940_CS_PIN                  GPIO_PIN_RA1

/*** Macros for GPIO_22 pin ***/
#define GPIO_22_Set()               (LATFSET = (1<<12))
#define GPIO_22_Clear()             (LATFCLR = (1<<12))
#define GPIO_22_Toggle()            (LATFINV= (1<<12))
#define GPIO_22_OutputEnable()      (TRISFCLR = (1<<12))
#define GPIO_22_InputEnable()       (TRISFSET = (1<<12))
#define GPIO_22_Get()               ((PORTF >> 12) & 0x1)
#define GPIO_22_PIN                  GPIO_PIN_RF12

/*** Macros for AD5940_RESET pin ***/
#define AD5940_RESET_Set()               (LATBSET = (1<<12))
#define AD5940_RESET_Clear()             (LATBCLR = (1<<12))
#define AD5940_RESET_Toggle()            (LATBINV= (1<<12))
#define AD5940_RESET_OutputEnable()      (TRISBCLR = (1<<12))
#define AD5940_RESET_InputEnable()       (TRISBSET = (1<<12))
#define AD5940_RESET_Get()               ((PORTB >> 12) & 0x1)
#define AD5940_RESET_PIN                  GPIO_PIN_RB12

/*** Macros for GPIO_27 pin ***/
#define GPIO_27_Set()               (LATBSET = (1<<13))
#define GPIO_27_Clear()             (LATBCLR = (1<<13))
#define GPIO_27_Toggle()            (LATBINV= (1<<13))
#define GPIO_27_OutputEnable()      (TRISBCLR = (1<<13))
#define GPIO_27_InputEnable()       (TRISBSET = (1<<13))
#define GPIO_27_Get()               ((PORTB >> 13) & 0x1)
#define GPIO_27_PIN                  GPIO_PIN_RB13

/*** Macros for GPIO_15 pin ***/
#define GPIO_15_Set()               (LATBSET = (1<<14))
#define GPIO_15_Clear()             (LATBCLR = (1<<14))
#define GPIO_15_Toggle()            (LATBINV= (1<<14))
#define GPIO_15_OutputEnable()      (TRISBCLR = (1<<14))
#define GPIO_15_InputEnable()       (TRISBSET = (1<<14))
#define GPIO_15_Get()               ((PORTB >> 14) & 0x1)
#define GPIO_15_PIN                  GPIO_PIN_RB14

/*** Macros for GPIO_R12 pin ***/
#define GPIO_R12_Set()               (LATBSET = (1<<15))
#define GPIO_R12_Clear()             (LATBCLR = (1<<15))
#define GPIO_R12_Toggle()            (LATBINV= (1<<15))
#define GPIO_R12_OutputEnable()      (TRISBCLR = (1<<15))
#define GPIO_R12_InputEnable()       (TRISBSET = (1<<15))
#define GPIO_R12_Get()               ((PORTB >> 15) & 0x1)
#define GPIO_R12_PIN                  GPIO_PIN_RB15

/*** Macros for GPIO_30 pin ***/
#define GPIO_30_Set()               (LATHSET = (1<<4))
#define GPIO_30_Clear()             (LATHCLR = (1<<4))
#define GPIO_30_Toggle()            (LATHINV= (1<<4))
#define GPIO_30_OutputEnable()      (TRISHCLR = (1<<4))
#define GPIO_30_InputEnable()       (TRISHSET = (1<<4))
#define GPIO_30_Get()               ((PORTH >> 4) & 0x1)
#define GPIO_30_PIN                  GPIO_PIN_RH4

/*** Macros for NN_EN pin ***/
#define NN_EN_Set()               (LATHSET = (1<<5))
#define NN_EN_Clear()             (LATHCLR = (1<<5))
#define NN_EN_Toggle()            (LATHINV= (1<<5))
#define NN_EN_OutputEnable()      (TRISHCLR = (1<<5))
#define NN_EN_InputEnable()       (TRISHSET = (1<<5))
#define NN_EN_Get()               ((PORTH >> 5) & 0x1)
#define NN_EN_PIN                  GPIO_PIN_RH5

/*** Macros for ADC_CS pin ***/
#define ADC_CS_Set()               (LATHSET = (1<<6))
#define ADC_CS_Clear()             (LATHCLR = (1<<6))
#define ADC_CS_Toggle()            (LATHINV= (1<<6))
#define ADC_CS_OutputEnable()      (TRISHCLR = (1<<6))
#define ADC_CS_InputEnable()       (TRISHSET = (1<<6))
#define ADC_CS_Get()               ((PORTH >> 6) & 0x1)
#define ADC_CS_PIN                  GPIO_PIN_RH6

/*** Macros for EPROM_CS pin ***/
#define EPROM_CS_Set()               (LATHSET = (1<<7))
#define EPROM_CS_Clear()             (LATHCLR = (1<<7))
#define EPROM_CS_Toggle()            (LATHINV= (1<<7))
#define EPROM_CS_OutputEnable()      (TRISHCLR = (1<<7))
#define EPROM_CS_InputEnable()       (TRISHSET = (1<<7))
#define EPROM_CS_Get()               ((PORTH >> 7) & 0x1)
#define EPROM_CS_PIN                  GPIO_PIN_RH7

/*** Macros for GPIO_10 pin ***/
#define GPIO_10_Set()               (LATDSET = (1<<15))
#define GPIO_10_Clear()             (LATDCLR = (1<<15))
#define GPIO_10_Toggle()            (LATDINV= (1<<15))
#define GPIO_10_OutputEnable()      (TRISDCLR = (1<<15))
#define GPIO_10_InputEnable()       (TRISDSET = (1<<15))
#define GPIO_10_Get()               ((PORTD >> 15) & 0x1)
#define GPIO_10_PIN                  GPIO_PIN_RD15

/*** Macros for POSC pin ***/
#define POSC_Get()               ((PORTC >> 12) & 0x1)
#define POSC_PIN                  GPIO_PIN_RC12

/*** Macros for GPIO_1 pin ***/
#define GPIO_1_Set()               (LATHSET = (1<<10))
#define GPIO_1_Clear()             (LATHCLR = (1<<10))
#define GPIO_1_Toggle()            (LATHINV= (1<<10))
#define GPIO_1_OutputEnable()      (TRISHCLR = (1<<10))
#define GPIO_1_InputEnable()       (TRISHSET = (1<<10))
#define GPIO_1_Get()               ((PORTH >> 10) & 0x1)
#define GPIO_1_PIN                  GPIO_PIN_RH10

/*** Macros for GPIO_11 pin ***/
#define GPIO_11_Set()               (LATFSET = (1<<4))
#define GPIO_11_Clear()             (LATFCLR = (1<<4))
#define GPIO_11_Toggle()            (LATFINV= (1<<4))
#define GPIO_11_OutputEnable()      (TRISFCLR = (1<<4))
#define GPIO_11_InputEnable()       (TRISFSET = (1<<4))
#define GPIO_11_Get()               ((PORTF >> 4) & 0x1)
#define GPIO_11_PIN                  GPIO_PIN_RF4

/*** Macros for GPIO_4 pin ***/
#define GPIO_4_Set()               (LATKSET = (1<<4))
#define GPIO_4_Clear()             (LATKCLR = (1<<4))
#define GPIO_4_Toggle()            (LATKINV= (1<<4))
#define GPIO_4_OutputEnable()      (TRISKCLR = (1<<4))
#define GPIO_4_InputEnable()       (TRISKSET = (1<<4))
#define GPIO_4_Get()               ((PORTK >> 4) & 0x1)
#define GPIO_4_PIN                  GPIO_PIN_RK4

/*** Macros for GPIO_5 pin ***/
#define GPIO_5_Set()               (LATKSET = (1<<5))
#define GPIO_5_Clear()             (LATKCLR = (1<<5))
#define GPIO_5_Toggle()            (LATKINV= (1<<5))
#define GPIO_5_OutputEnable()      (TRISKCLR = (1<<5))
#define GPIO_5_InputEnable()       (TRISKSET = (1<<5))
#define GPIO_5_Get()               ((PORTK >> 5) & 0x1)
#define GPIO_5_PIN                  GPIO_PIN_RK5

/*** Macros for GPIO_7 pin ***/
#define GPIO_7_Set()               (LATKSET = (1<<6))
#define GPIO_7_Clear()             (LATKCLR = (1<<6))
#define GPIO_7_Toggle()            (LATKINV= (1<<6))
#define GPIO_7_OutputEnable()      (TRISKCLR = (1<<6))
#define GPIO_7_InputEnable()       (TRISKSET = (1<<6))
#define GPIO_7_Get()               ((PORTK >> 6) & 0x1)
#define GPIO_7_PIN                  GPIO_PIN_RK6

/*** Macros for GPIO_9 pin ***/
#define GPIO_9_Set()               (LATASET = (1<<14))
#define GPIO_9_Clear()             (LATACLR = (1<<14))
#define GPIO_9_Toggle()            (LATAINV= (1<<14))
#define GPIO_9_OutputEnable()      (TRISACLR = (1<<14))
#define GPIO_9_InputEnable()       (TRISASET = (1<<14))
#define GPIO_9_Get()               ((PORTA >> 14) & 0x1)
#define GPIO_9_PIN                  GPIO_PIN_RA14

/*** Macros for GPIO_8 pin ***/
#define GPIO_8_Set()               (LATASET = (1<<15))
#define GPIO_8_Clear()             (LATACLR = (1<<15))
#define GPIO_8_Toggle()            (LATAINV= (1<<15))
#define GPIO_8_OutputEnable()      (TRISACLR = (1<<15))
#define GPIO_8_InputEnable()       (TRISASET = (1<<15))
#define GPIO_8_Get()               ((PORTA >> 15) & 0x1)
#define GPIO_8_PIN                  GPIO_PIN_RA15

/*** Macros for MDOT_EN pin ***/
#define MDOT_EN_Set()               (LATDSET = (1<<9))
#define MDOT_EN_Clear()             (LATDCLR = (1<<9))
#define MDOT_EN_Toggle()            (LATDINV= (1<<9))
#define MDOT_EN_OutputEnable()      (TRISDCLR = (1<<9))
#define MDOT_EN_InputEnable()       (TRISDSET = (1<<9))
#define MDOT_EN_Get()               ((PORTD >> 9) & 0x1)
#define MDOT_EN_PIN                  GPIO_PIN_RD9

/*** Macros for SD_CS pin ***/
#define SD_CS_Set()               (LATHSET = (1<<12))
#define SD_CS_Clear()             (LATHCLR = (1<<12))
#define SD_CS_Toggle()            (LATHINV= (1<<12))
#define SD_CS_OutputEnable()      (TRISHCLR = (1<<12))
#define SD_CS_InputEnable()       (TRISHSET = (1<<12))
#define SD_CS_Get()               ((PORTH >> 12) & 0x1)
#define SD_CS_PIN                  GPIO_PIN_RH12

/*** Macros for MDOT_WAKE pin ***/
#define MDOT_WAKE_Set()               (LATHSET = (1<<13))
#define MDOT_WAKE_Clear()             (LATHCLR = (1<<13))
#define MDOT_WAKE_Toggle()            (LATHINV= (1<<13))
#define MDOT_WAKE_OutputEnable()      (TRISHCLR = (1<<13))
#define MDOT_WAKE_InputEnable()       (TRISHSET = (1<<13))
#define MDOT_WAKE_Get()               ((PORTH >> 13) & 0x1)
#define MDOT_WAKE_PIN                  GPIO_PIN_RH13

/*** Macros for MDOT_SLEEP pin ***/
#define MDOT_SLEEP_Set()               (LATHSET = (1<<14))
#define MDOT_SLEEP_Clear()             (LATHCLR = (1<<14))
#define MDOT_SLEEP_Toggle()            (LATHINV= (1<<14))
#define MDOT_SLEEP_OutputEnable()      (TRISHCLR = (1<<14))
#define MDOT_SLEEP_InputEnable()       (TRISHSET = (1<<14))
#define MDOT_SLEEP_Get()               ((PORTH >> 14) & 0x1)
#define MDOT_SLEEP_PIN                  GPIO_PIN_RH14

/*** Macros for MDOT_ASSOC pin ***/
#define MDOT_ASSOC_Set()               (LATHSET = (1<<15))
#define MDOT_ASSOC_Clear()             (LATHCLR = (1<<15))
#define MDOT_ASSOC_Toggle()            (LATHINV= (1<<15))
#define MDOT_ASSOC_OutputEnable()      (TRISHCLR = (1<<15))
#define MDOT_ASSOC_InputEnable()       (TRISHSET = (1<<15))
#define MDOT_ASSOC_Get()               ((PORTH >> 15) & 0x1)
#define MDOT_ASSOC_PIN                  GPIO_PIN_RH15

/*** Macros for MDOT_RST pin ***/
#define MDOT_RST_Set()               (LATJSET = (1<<0))
#define MDOT_RST_Clear()             (LATJCLR = (1<<0))
#define MDOT_RST_Toggle()            (LATJINV= (1<<0))
#define MDOT_RST_OutputEnable()      (TRISJCLR = (1<<0))
#define MDOT_RST_InputEnable()       (TRISJSET = (1<<0))
#define MDOT_RST_Get()               ((PORTJ >> 0) & 0x1)
#define MDOT_RST_PIN                  GPIO_PIN_RJ0

/*** Macros for MDOT_PQR pin ***/
#define MDOT_PQR_Set()               (LATJSET = (1<<1))
#define MDOT_PQR_Clear()             (LATJCLR = (1<<1))
#define MDOT_PQR_Toggle()            (LATJINV= (1<<1))
#define MDOT_PQR_OutputEnable()      (TRISJCLR = (1<<1))
#define MDOT_PQR_InputEnable()       (TRISJSET = (1<<1))
#define MDOT_PQR_Get()               ((PORTJ >> 1) & 0x1)
#define MDOT_PQR_PIN                  GPIO_PIN_RJ1

/*** Macros for LED2_BL pin ***/
#define LED2_BL_Set()               (LATJSET = (1<<2))
#define LED2_BL_Clear()             (LATJCLR = (1<<2))
#define LED2_BL_Toggle()            (LATJINV= (1<<2))
#define LED2_BL_OutputEnable()      (TRISJCLR = (1<<2))
#define LED2_BL_InputEnable()       (TRISJSET = (1<<2))
#define LED2_BL_Get()               ((PORTJ >> 2) & 0x1)
#define LED2_BL_PIN                  GPIO_PIN_RJ2

/*** Macros for LED2_GR pin ***/
#define LED2_GR_Set()               (LATJSET = (1<<3))
#define LED2_GR_Clear()             (LATJCLR = (1<<3))
#define LED2_GR_Toggle()            (LATJINV= (1<<3))
#define LED2_GR_OutputEnable()      (TRISJCLR = (1<<3))
#define LED2_GR_InputEnable()       (TRISJSET = (1<<3))
#define LED2_GR_Get()               ((PORTJ >> 3) & 0x1)
#define LED2_GR_PIN                  GPIO_PIN_RJ3

/*** Macros for PT_EOC pin ***/
#define PT_EOC_Set()               (LATFSET = (1<<0))
#define PT_EOC_Clear()             (LATFCLR = (1<<0))
#define PT_EOC_Toggle()            (LATFINV= (1<<0))
#define PT_EOC_OutputEnable()      (TRISFCLR = (1<<0))
#define PT_EOC_InputEnable()       (TRISFSET = (1<<0))
#define PT_EOC_Get()               ((PORTF >> 0) & 0x1)
#define PT_EOC_PIN                  GPIO_PIN_RF0

/*** Macros for PT_RES pin ***/
#define PT_RES_Set()               (LATFSET = (1<<1))
#define PT_RES_Clear()             (LATFCLR = (1<<1))
#define PT_RES_Toggle()            (LATFINV= (1<<1))
#define PT_RES_OutputEnable()      (TRISFCLR = (1<<1))
#define PT_RES_InputEnable()       (TRISFSET = (1<<1))
#define PT_RES_Get()               ((PORTF >> 1) & 0x1)
#define PT_RES_PIN                  GPIO_PIN_RF1

/*** Macros for LED2_RD pin ***/
#define LED2_RD_Set()               (LATKSET = (1<<7))
#define LED2_RD_Clear()             (LATKCLR = (1<<7))
#define LED2_RD_Toggle()            (LATKINV= (1<<7))
#define LED2_RD_OutputEnable()      (TRISKCLR = (1<<7))
#define LED2_RD_InputEnable()       (TRISKSET = (1<<7))
#define LED2_RD_Get()               ((PORTK >> 7) & 0x1)
#define LED2_RD_PIN                  GPIO_PIN_RK7

/*** Macros for LED1_BL pin ***/
#define LED1_BL_Set()               (LATGSET = (1<<1))
#define LED1_BL_Clear()             (LATGCLR = (1<<1))
#define LED1_BL_Toggle()            (LATGINV= (1<<1))
#define LED1_BL_OutputEnable()      (TRISGCLR = (1<<1))
#define LED1_BL_InputEnable()       (TRISGSET = (1<<1))
#define LED1_BL_Get()               ((PORTG >> 1) & 0x1)
#define LED1_BL_PIN                  GPIO_PIN_RG1

/*** Macros for GPS_U_INT pin ***/
#define GPS_U_INT_Set()               (LATJSET = (1<<4))
#define GPS_U_INT_Clear()             (LATJCLR = (1<<4))
#define GPS_U_INT_Toggle()            (LATJINV= (1<<4))
#define GPS_U_INT_OutputEnable()      (TRISJCLR = (1<<4))
#define GPS_U_INT_InputEnable()       (TRISJSET = (1<<4))
#define GPS_U_INT_Get()               ((PORTJ >> 4) & 0x1)
#define GPS_U_INT_PIN                  GPIO_PIN_RJ4

/*** Macros for GPS_U_RST pin ***/
#define GPS_U_RST_Set()               (LATJSET = (1<<5))
#define GPS_U_RST_Clear()             (LATJCLR = (1<<5))
#define GPS_U_RST_Toggle()            (LATJINV= (1<<5))
#define GPS_U_RST_OutputEnable()      (TRISJCLR = (1<<5))
#define GPS_U_RST_InputEnable()       (TRISJSET = (1<<5))
#define GPS_U_RST_Get()               ((PORTJ >> 5) & 0x1)
#define GPS_U_RST_PIN                  GPIO_PIN_RJ5

/*** Macros for LED1_GR pin ***/
#define LED1_GR_Set()               (LATJSET = (1<<6))
#define LED1_GR_Clear()             (LATJCLR = (1<<6))
#define LED1_GR_Toggle()            (LATJINV= (1<<6))
#define LED1_GR_OutputEnable()      (TRISJCLR = (1<<6))
#define LED1_GR_InputEnable()       (TRISJSET = (1<<6))
#define LED1_GR_Get()               ((PORTJ >> 6) & 0x1)
#define LED1_GR_PIN                  GPIO_PIN_RJ6

/*** Macros for LED1_RD pin ***/
#define LED1_RD_Set()               (LATJSET = (1<<7))
#define LED1_RD_Clear()             (LATJCLR = (1<<7))
#define LED1_RD_Toggle()            (LATJINV= (1<<7))
#define LED1_RD_OutputEnable()      (TRISJCLR = (1<<7))
#define LED1_RD_InputEnable()       (TRISJSET = (1<<7))
#define LED1_RD_Get()               ((PORTJ >> 7) & 0x1)
#define LED1_RD_PIN                  GPIO_PIN_RJ7

/*** Macros for GPS_U_TIME pin ***/
#define GPS_U_TIME_Set()               (LATESET = (1<<0))
#define GPS_U_TIME_Clear()             (LATECLR = (1<<0))
#define GPS_U_TIME_Toggle()            (LATEINV= (1<<0))
#define GPS_U_TIME_OutputEnable()      (TRISECLR = (1<<0))
#define GPS_U_TIME_InputEnable()       (TRISESET = (1<<0))
#define GPS_U_TIME_Get()               ((PORTE >> 0) & 0x1)
#define GPS_U_TIME_PIN                  GPIO_PIN_RE0

/*** Macros for GPIO_20 pin ***/
#define GPIO_20_Set()               (LATESET = (1<<1))
#define GPIO_20_Clear()             (LATECLR = (1<<1))
#define GPIO_20_Toggle()            (LATEINV= (1<<1))
#define GPIO_20_OutputEnable()      (TRISECLR = (1<<1))
#define GPIO_20_InputEnable()       (TRISESET = (1<<1))
#define GPIO_20_Get()               ((PORTE >> 1) & 0x1)
#define GPIO_20_PIN                  GPIO_PIN_RE1

/*** Macros for GPIO_21 pin ***/
#define GPIO_21_Set()               (LATESET = (1<<2))
#define GPIO_21_Clear()             (LATECLR = (1<<2))
#define GPIO_21_Toggle()            (LATEINV= (1<<2))
#define GPIO_21_OutputEnable()      (TRISECLR = (1<<2))
#define GPIO_21_InputEnable()       (TRISESET = (1<<2))
#define GPIO_21_Get()               ((PORTE >> 2) & 0x1)
#define GPIO_21_PIN                  GPIO_PIN_RE2

/*** Macros for GPIO_25 pin ***/
#define GPIO_25_Set()               (LATESET = (1<<4))
#define GPIO_25_Clear()             (LATECLR = (1<<4))
#define GPIO_25_Toggle()            (LATEINV= (1<<4))
#define GPIO_25_OutputEnable()      (TRISECLR = (1<<4))
#define GPIO_25_InputEnable()       (TRISESET = (1<<4))
#define GPIO_25_Get()               ((PORTE >> 4) & 0x1)
#define GPIO_25_PIN                  GPIO_PIN_RE4


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
    GPIO_PORT_H = 7,
    GPIO_PORT_J = 8,
    GPIO_PORT_K = 9,
} GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA2 = 2,
    GPIO_PIN_RA3 = 3,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA5 = 5,
    GPIO_PIN_RA6 = 6,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA9 = 9,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA14 = 14,
    GPIO_PIN_RA15 = 15,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC3 = 35,
    GPIO_PIN_RC4 = 36,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC14 = 46,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD0 = 48,
    GPIO_PIN_RD1 = 49,
    GPIO_PIN_RD2 = 50,
    GPIO_PIN_RD3 = 51,
    GPIO_PIN_RD4 = 52,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD7 = 55,
    GPIO_PIN_RD9 = 57,
    GPIO_PIN_RD10 = 58,
    GPIO_PIN_RD11 = 59,
    GPIO_PIN_RD12 = 60,
    GPIO_PIN_RD13 = 61,
    GPIO_PIN_RD14 = 62,
    GPIO_PIN_RD15 = 63,
    GPIO_PIN_RE0 = 64,
    GPIO_PIN_RE1 = 65,
    GPIO_PIN_RE2 = 66,
    GPIO_PIN_RE3 = 67,
    GPIO_PIN_RE4 = 68,
    GPIO_PIN_RE5 = 69,
    GPIO_PIN_RE6 = 70,
    GPIO_PIN_RE7 = 71,
    GPIO_PIN_RE8 = 72,
    GPIO_PIN_RE9 = 73,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RF2 = 82,
    GPIO_PIN_RF3 = 83,
    GPIO_PIN_RF4 = 84,
    GPIO_PIN_RF5 = 85,
    GPIO_PIN_RF8 = 88,
    GPIO_PIN_RF12 = 92,
    GPIO_PIN_RF13 = 93,
    GPIO_PIN_RG0 = 96,
    GPIO_PIN_RG1 = 97,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,
    GPIO_PIN_RG12 = 108,
    GPIO_PIN_RG13 = 109,
    GPIO_PIN_RG14 = 110,
    GPIO_PIN_RG15 = 111,
    GPIO_PIN_RH0 = 112,
    GPIO_PIN_RH1 = 113,
    GPIO_PIN_RH2 = 114,
    GPIO_PIN_RH3 = 115,
    GPIO_PIN_RH4 = 116,
    GPIO_PIN_RH5 = 117,
    GPIO_PIN_RH6 = 118,
    GPIO_PIN_RH7 = 119,
    GPIO_PIN_RH8 = 120,
    GPIO_PIN_RH9 = 121,
    GPIO_PIN_RH10 = 122,
    GPIO_PIN_RH11 = 123,
    GPIO_PIN_RH12 = 124,
    GPIO_PIN_RH13 = 125,
    GPIO_PIN_RH14 = 126,
    GPIO_PIN_RH15 = 127,
    GPIO_PIN_RJ0 = 128,
    GPIO_PIN_RJ1 = 129,
    GPIO_PIN_RJ2 = 130,
    GPIO_PIN_RJ3 = 131,
    GPIO_PIN_RJ4 = 132,
    GPIO_PIN_RJ5 = 133,
    GPIO_PIN_RJ6 = 134,
    GPIO_PIN_RJ7 = 135,
    GPIO_PIN_RJ8 = 136,
    GPIO_PIN_RJ9 = 137,
    GPIO_PIN_RJ10 = 138,
    GPIO_PIN_RJ11 = 139,
    GPIO_PIN_RJ12 = 140,
    GPIO_PIN_RJ13 = 141,
    GPIO_PIN_RJ14 = 142,
    GPIO_PIN_RJ15 = 143,
    GPIO_PIN_RK0 = 144,
    GPIO_PIN_RK1 = 145,
    GPIO_PIN_RK2 = 146,
    GPIO_PIN_RK3 = 147,
    GPIO_PIN_RK4 = 148,
    GPIO_PIN_RK5 = 149,
    GPIO_PIN_RK6 = 150,
    GPIO_PIN_RK7 = 151,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;

typedef  void (*GPIO_PIN_CALLBACK) ( GPIO_PIN pin, uintptr_t context);

void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptDisable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* target pin */
    GPIO_PIN                 pin;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK        callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

#define GPIO_PinInterruptEnable(pin)       GPIO_PinIntEnable(pin, GPIO_INTERRUPT_ON_MISMATCH)
#define GPIO_PinInterruptDisable(pin)      GPIO_PinIntDisable(pin)

void GPIO_PinIntEnable(GPIO_PIN pin, GPIO_INTERRUPT_STYLE style);
void GPIO_PinIntDisable(GPIO_PIN pin);

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const   GPIO_PIN_CALLBACK callBack,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
