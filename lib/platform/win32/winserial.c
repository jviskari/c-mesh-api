/* Joonas Viskari licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */

#define LOG_MODULE_NAME "SERIAL"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"


/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/
int Serial_open(const char * port_name, unsigned long bitrate)
{
    return 0;
}

int Serial_close()
{
    return 0;
}


int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    return 0;
}

int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    return 0;
}
