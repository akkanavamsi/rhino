/* 
 * File:   console.h
 * Author: RFord7
 *
 * Created on April 20, 2022, 9:50 AM
 */

#ifndef CONSOLE_H
#define	CONSOLE_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
    
void printConsole(float f);
void printConsoleStr(char *data);
void printConsole2(char *data, int length);
void createHexString(const unsigned char *src, int len, unsigned char *dest);

#ifdef	__cplusplus
}
#endif

#endif	/* CONSOLE_H */

