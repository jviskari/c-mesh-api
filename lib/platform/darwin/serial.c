/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <IOKit/serial/ioss.h>

#include "platform.h"

#define LOG_MODULE_NAME "SERIAL"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"

static int fd = -1;

/** \brief Forward declaration of internal open */
static int int_open();

/** \brief  Port name to open */
static char m_port_name[256];

/** \brief Bitrate to use */
static unsigned long m_bitrate;

static int set_interface_attribs(int fd, unsigned long bitrate, int parity)
{
    struct termios tty;
    speed_t speed = bitrate;

    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        LOGE("Error %d from tcgetattr", errno);
        return -1;
    }

    // 8-bit chars
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    // disable break processing
    tty.c_iflag &= ~IGNBRK;
    // disable CR -> NL translation
    tty.c_iflag &= ~ICRNL;
    // no signaling chars, no echo, no canonical processing
    tty.c_lflag = 0;
    // no remapping, no delays
    tty.c_oflag = 0;
    // VMIN=0, VTIME=0 => non-blocking
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // shut off xon/xoff ctrl
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // ignore modem controls, enable reading
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        LOGE("Error %d from tcsetattr", errno);
        return -1;
    }

    // Set the speed using IOSSIOSPEED for non-standard rates support
    if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
    {
        LOGE("Error %d from IOSSIOSPEED", errno);
        return -1;
    }

    LOGD("Custom bitrate set: %lu\n", bitrate);

    return 0;
}

static int int_open()
{
    fd = open(m_port_name, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0)
    {
        LOGE("Error %d opening serial link %s: %s\n", errno, m_port_name, strerror(errno));
        return -1;
    }

    // set the requested bitrate, 8n1, no parity
    if (set_interface_attribs(fd, m_bitrate, 0) < 0)
    {
        close(fd);
        fd = -1;
        return -1;
    }

    // Remove non-blocking flag after open to behave nicely with select/read logic if needed,
    // but VMIN=0/VTIME=0 essentially makes read non-blocking anyway.
    // However, to be safe and consistent, we can leave O_NONBLOCK or clear it.
    // With VMIN=0 VTIME=0, read returns immediately.
    // Let's clear O_NONBLOCK to ensure standard behavior if we ever change VMIN/VTIME.
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    LOGD("Serial opened\n");
    return 0;
}
/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/
int Serial_open(const char * port_name, unsigned long bitrate)
{
    // Copy the settings locally
    strcpy(m_port_name, port_name);
    m_bitrate = bitrate;

    return int_open();
}

int Serial_close()
{
    if (fd < 0)
    {
        LOGW("Link already closed\n");
        return -1;
    }

    if (close(fd) < 0)
    {
        LOGW("Error %d closing serial link: %s\n", errno, strerror(errno));
        return -1;
    }

    fd = -1;
    LOGD("Serial closed\n");
    return 0;
}

static ssize_t get_single_char(unsigned char * c, unsigned int timeout_ms)
{
    static unsigned char m_buffer[128];
    static uint8_t m_elem = 0;
    static uint8_t m_read = 0;
    ssize_t read_bytes;
    fd_set readfs;
    struct timeval tv;
    int ret;

    // Do we still have char buffered?
    if (m_elem > 0)
    {
        *c = m_buffer[m_read++];
        m_elem--;
        return 1;
    }

    // Use select to wait for data
    FD_ZERO(&readfs);
    FD_SET(fd, &readfs);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &readfs, NULL, NULL, &tv);

    if (ret < 0)
    {
        LOGE("Error in select: %d\n", errno);
        return 0; // Treat error as timeout/no-data for this simplified API
    }
    else if (ret == 0)
    {
        // Timeout
        return 0;
    }

    // Data available
    read_bytes = read(fd, m_buffer, sizeof(m_buffer));
    if (read_bytes > 0)
    {
        m_elem = read_bytes;
        m_read = 0;
        *c = m_buffer[m_read++];
        m_elem--;
        return 1;
    }
    else if (read_bytes < 0)
    {
         LOGE("Error in read: %d\n", errno);
         // If read fails, it might be due to disconnect, handled in write usually
         return 0;
    }

    return 0;
}

int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        return -1;
    }

    return get_single_char(c, timeout_ms);
}

int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        // Try to reopen
        if (int_open() < 0)
        {
            // Wait a bit before next try
            usleep(1000 * 1000);
            return 0;
        }
        LOGI("Serial reopened\n");
    }

    ssize_t ret = write(fd, buffer, buffer_size);
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
