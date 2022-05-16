/* 
 * File:   stringbuffer.h
 * Author: RFord7
 *
 * Created on 18 January 2022, 14:04
 */

#ifndef STRINGBUFFER_H
#define	STRINGBUFFER_H

#include <string.h>
#include "mdot.h"

#define STRING_BUFFER_SIZE 200
#define STRING_BUFFER_COUNT 15

typedef struct
{
    MDOT_COMMAND command[STRING_BUFFER_COUNT];
    int head;
    int tail;
    char buffer[STRING_BUFFER_COUNT][STRING_BUFFER_SIZE];
}STRING_BUFFER;

void initBuffer(STRING_BUFFER *sb);
size_t push(STRING_BUFFER *sb, char *data, MDOT_COMMAND com); 
unsigned char available(STRING_BUFFER *sb);
size_t pop(STRING_BUFFER *sb, char *dest, MDOT_COMMAND *com);

#endif	/* STRINGBUFFER_H */

