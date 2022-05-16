#include "stringbuffer.h"

void initBuffer(STRING_BUFFER *sb)
{
    memset(sb->buffer,0,STRING_BUFFER_SIZE * STRING_BUFFER_COUNT);
    int i;
    for(i=0;i<STRING_BUFFER_COUNT;i++)
        sb->command[i] = MDOT_NONE;
    sb->head = 0;
    sb->tail = 0;
}

size_t push(STRING_BUFFER *sb, char *data,  MDOT_COMMAND com)
{
    size_t ret = strlen(data);
    strcpy(&(sb->buffer[sb->head][0]), data);
     sb->command[sb->head] = com;
    if( ++sb->head >= STRING_BUFFER_COUNT)
        sb->head = 0;
    return ret;
}

size_t pop(STRING_BUFFER *sb, char *dest, MDOT_COMMAND *com)
{
    size_t ret = strlen(&(sb->buffer[sb->tail][0]));
    strcpy(dest, &(sb->buffer[sb->tail][0]));
    *com = sb->command[sb->tail];
    if( ++sb->tail >= STRING_BUFFER_COUNT)
        sb->tail = 0;
    return ret;
}

unsigned char available(STRING_BUFFER *sb)
{
    return((sb->head - sb->tail) % STRING_BUFFER_COUNT);//Size of data in buffer.
}

