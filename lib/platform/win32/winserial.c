/* Joonas Viskari licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */

#include "rs232.h"

#define LOG_MODULE_NAME "SERIAL"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"

/*port*/
static int cport_nr = -1;

/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/
int Serial_open(const char * port_name, unsigned long bitrate)
{
    char mode[]={'8','N','1',0};

    LOGI("Opening port %s at %d\n", port_name, bitrate);

    cport_nr = RS232_GetPortnr(port_name);

    if(RS232_OpenComport(cport_nr, bitrate, mode, 0))
    {
        LOGE("Can not open comport\n");
        return(-1);
    }

    LOGI("Comport %s opened\n", port_name);

    return 0;
}

int Serial_close()
{
    RS232_CloseComport(cport_nr);
    LOGI("Comport %d closed\n", cport_nr);
    return 0;
}


static ssize_t get_single_char(unsigned char * c, unsigned int timeout_ms)
{
    static unsigned char m_buffer[128];
    static uint8_t m_elem = 0;
    static uint8_t m_read = 0;
    ssize_t read_bytes;
    uint16_t attempts;

    // Do we still have char buffered?
    if (m_elem > 0)
    {
        *c = m_buffer[m_read++];
        m_elem--;
        return 1;
    }

    // Convert timeout_ms in attempt of 100ms (timeout set)
    attempts = ((timeout_ms + 99) / 100) - 1;

    // Local buffer is empty, refill it
    do
    {
        read_bytes =  RS232_PollComport(cport_nr, m_buffer, sizeof(m_buffer));

        if (read_bytes > 0)
        {
            m_elem = read_bytes;
            m_read = 0;
            *c = m_buffer[m_read++];
            m_elem--;
            return 1;
        }
    } while (attempts-- > 0);

    LOGW("Timeout to wait for char on serial line\n");
    return 0;
}


int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    if (cport_nr < 0)
    {
        LOGE("No serial link opened\n");
        return -1;
    }

    return get_single_char(c, timeout_ms);
}

int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    if (cport_nr < 0)
    {
        /*TODO: Find port and re-open*/
        LOGE("No serial link opened\n");
        return 0;
    }

    ssize_t ret = RS232_SendBuf(cport_nr, buffer, buffer_size);

    if (ret < 0)
    {
        LOGE("Error in write: %d\n", errno);
        LOGE("Close connection\n");
        Serial_close();

        // Connection will be checked reopen at next try
        return 0;
    }

    return ret;
}

