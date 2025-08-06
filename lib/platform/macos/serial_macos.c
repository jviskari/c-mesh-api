#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>  // <-- Add this
#endif

#include "platform.h"

#define LOG_MODULE_NAME "MACOS_SERIAL"
#define MAX_LOG_LEVEL DEBUG_LOG_LEVEL
#include "logger.h"

#ifndef __APPLE__
#error "This file is only for macOS platform"
#endif

static int fd = -1;

static int int_open();

static char m_port_name[256];
static unsigned long m_bitrate;

static int set_interface_attribs(int fd, unsigned long bitrate, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0)
    {
        LOGE("Error %d from tcgetattr", errno);
        return -1;
    }

    // Set baud rate (standard values)
    speed_t speed;
    switch (bitrate)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            // Use closest standard for tcsetattr, real speed set via IOSSIOSPEED
            speed = B230400;
            break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= parity;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        LOGE("Error %d from tcsetattr", errno);
        return -1;
    }

#ifdef __APPLE__
    // Support non-standard baud rate (e.g., 125000)
    if (bitrate != 9600 && bitrate != 19200 && bitrate != 38400 &&
        bitrate != 57600 && bitrate != 115200 && bitrate != 230400)
    {
        speed_t custom_speed = bitrate;
        if (ioctl(fd, IOSSIOSPEED, &custom_speed) < 0)
        {
            LOGE("Failed to set custom baud rate %lu: %s\n", bitrate, strerror(errno));
            return -1;
        }
        LOGD("Custom baud rate %lu set using IOSSIOSPEED\n", bitrate);
    }
#endif

    return 0;
}

static int int_open()
{
    fd = open(m_port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        LOGE("Error %d opening serial link %s: %s\n", errno, m_port_name, strerror(errno));
        return -1;
    }

    if (set_interface_attribs(fd, m_bitrate, 0) < 0)
    {
        close(fd);
        fd = -1;
        return -1;
    }

    LOGD("Serial opened\n");
    return 0;
}

int Serial_open(const char * port_name, unsigned long bitrate)
{
    LOGD("MACOS Serial_open\n");
    strncpy(m_port_name, port_name, sizeof(m_port_name) - 1);
    m_port_name[sizeof(m_port_name) - 1] = '\0';
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
    int attempts;

    if (m_elem > 0)
    {
        *c = m_buffer[m_read++];
        m_elem--;
        return 1;
    }

    // Avoid unsigned underflow
    attempts = (timeout_ms < 100) ? 0 : ((timeout_ms + 99) / 100) - 1;

    do
    {
        read_bytes = read(fd, m_buffer, sizeof(m_buffer));
        if (read_bytes > 0)
        {
            m_elem = read_bytes;
            m_read = 0;
            *c = m_buffer[m_read++];
            m_elem--;
            return 1;
        }
    } while (attempts-- > 0);

    LOGD("Timeout to wait for char on serial line\n");
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

        if (int_open() < 0)
        {
            usleep(1000 * 1000);
            return -1;
        }
        LOGI("Serial reopened\n");
    }

    ssize_t ret = write(fd, buffer, buffer_size);
    if (ret < 0)
    {
        LOGE("Error in write: %d (%s)\n", errno, strerror(errno));
        LOGE("Close connection\n");
        Serial_close();
        return -1;
    }
    return ret;
}