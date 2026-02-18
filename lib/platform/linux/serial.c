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
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <pthread.h>
#include <linux/serial.h>

#include "serial_termios2.h"
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

/** \brief Mutex for thread safety */
static pthread_mutex_t serial_mutex = PTHREAD_MUTEX_INITIALIZER;

/** \brief Buffer state for get_single_char */
static unsigned char m_buffer[128];
static uint8_t m_elem = 0;
static uint8_t m_read = 0;

/** \brief Validate serial port name */
static int validate_port_name(const char * port_name)
{
    if (port_name == NULL)
    {
        LOGE("Port name is NULL\n");
        return -1;
    }

    size_t len = strlen(port_name);
    if (len == 0 || len >= sizeof(m_port_name))
    {
        LOGE("Port name invalid length: %zu (max: %zu)\n", len, sizeof(m_port_name) - 1);
        return -1;
    }

    // On Linux, serial ports should be under /dev/
    if (strncmp(port_name, "/dev/", 5) != 0)
    {
        LOGE("Invalid port name format (must start with /dev/)\n");
        return -1;
    }

    // Additional check: should be ttyUSB*, ttyACM*, ttyS*, or ttyAMA*
    const char *device_name = port_name + 5;
    if (strncmp(device_name, "ttyUSB", 6) != 0 &&
        strncmp(device_name, "ttyACM", 6) != 0 &&
        strncmp(device_name, "ttyS", 4) != 0 &&
        strncmp(device_name, "ttyAMA", 6) != 0)
    {
        LOGW("Warning: port name doesn't match typical Linux serial pattern (/dev/ttyUSB*, /dev/ttyACM*, /dev/ttyS*, /dev/ttyAMA*)\n");
    }

    return 0;
}

/** \brief Validate bitrate */
static int validate_bitrate(unsigned long bitrate)
{
    // Common valid bitrates (allow custom rates but check for reasonable range)
    if (bitrate < 300 || bitrate > 12000000)
    {
        LOGE("Bitrate %lu out of reasonable range (300 - 12000000)\n", bitrate);
        return -1;
    }

    return 0;
}

static int set_interface_attribs(int fd, unsigned long bitrate, int parity)
{
    struct termios tty;
    struct serial_struct serial_s;

    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        LOGE("Error %d from tcgetattr", errno);
        return -1;
    }

    // default to 9600 bps, but use TCSETS2 to set the actual bitrate
    // use a bitrate that is not 115200 or 125000 bps here, to make sure
    // TCSETS2 actually sets the bitrate
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

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
    // VMIN=0, VTIME=0 => non-blocking (we use select for timeout control)
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

    if (Serial_set_termios2_bitrate(fd, bitrate) != 0)
    {
        return -1;
    }

    // Set low latency flag to serial
    if (ioctl(fd, TIOCGSERIAL, &serial_s) < 0)
    {
        LOGE("Error %d from TIOCGSERIAL", errno);
        return -1;
    }

    serial_s.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(fd, TIOCSSERIAL, &serial_s) < 0)
    {
        LOGE("Error %d from TIOCSSERIAL", errno);
        return -1;
    }

    return 0;
}

static int int_open()
{
    int temp_fd = open(m_port_name, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (temp_fd < 0)
    {
        LOGE("Error %d opening serial link %s: %s\n", errno, m_port_name, strerror(errno));
        return -1;
    }

    // set the requested bitrate, 8n1, no parity
    if (set_interface_attribs(temp_fd, m_bitrate, 0) < 0)
    {
        close(temp_fd);
        return -1;
    }

    // Remove non-blocking flag after open to behave nicely with select/read logic.
    // With VMIN=0 VTIME=0, read returns immediately anyway, but clearing O_NONBLOCK
    // ensures standard behavior if we ever change VMIN/VTIME.
    int flags = fcntl(temp_fd, F_GETFL, 0);
    if (flags >= 0)
    {
        fcntl(temp_fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    // Flush any stale data in buffers from previous connections
    tcflush(temp_fd, TCIOFLUSH);

    fd = temp_fd;
    LOGD("Serial opened\n");
    return 0;
}

/** \brief Reset internal buffer state */
static void reset_buffer_state()
{
    m_elem = 0;
    m_read = 0;
    memset(m_buffer, 0, sizeof(m_buffer));
}

/****************************************************************************/
/*                Public method implementation                              */
/****************************************************************************/

/**
 * \brief   Open a serial link to the Wirepas Mesh MCU
 *
 * Opens and configures a serial port with the specified bitrate.
 * If a port is already open, it will be closed first.
 *
 * Port names must be in /dev/ format (typically /dev/ttyUSB*, /dev/ttyACM*,
 * /dev/ttyS*, or /dev/ttyAMA* on Linux).
 * Bitrate must be in range 300 to 12,000,000 bps.
 *
 * Thread-safe: This function uses internal locking and can be called
 * concurrently with other Serial_* functions.
 *
 * \param   port_name   Path to the serial device (e.g., "/dev/ttyUSB0")
 * \param   bitrate     Desired bitrate in bps (supports custom rates via termios2)
 * \return  0 on success, -1 on error
 */
int Serial_open(const char * port_name, unsigned long bitrate)
{
    int ret;

    pthread_mutex_lock(&serial_mutex);

    // Validate inputs
    if (validate_port_name(port_name) < 0)
    {
        pthread_mutex_unlock(&serial_mutex);
        return -1;
    }

    if (validate_bitrate(bitrate) < 0)
    {
        pthread_mutex_unlock(&serial_mutex);
        return -1;
    }

    // Check if already open
    if (fd >= 0)
    {
        LOGW("Serial port already open, closing first\n");
        close(fd);
        fd = -1;
    }

    // Copy the settings locally with bounds checking
    strncpy(m_port_name, port_name, sizeof(m_port_name) - 1);
    m_port_name[sizeof(m_port_name) - 1] = '\0';
    m_bitrate = bitrate;

    // Reset buffer state
    reset_buffer_state();

    ret = int_open();

    pthread_mutex_unlock(&serial_mutex);
    return ret;
}

/**
 * \brief   Close a serial link previously opened with Serial_open
 *
 * Closes the serial port and resets internal buffer state.
 * After closing, Serial_open must be called before using read/write.
 *
 * Thread-safe: This function uses internal locking and can be called
 * concurrently with other Serial_* functions.
 *
 * \return  0 on success, -1 if port was not open or close failed
 */
int Serial_close()
{
    int ret = 0;

    pthread_mutex_lock(&serial_mutex);

    if (fd < 0)
    {
        LOGW("Link already closed\n");
        pthread_mutex_unlock(&serial_mutex);
        return -1;
    }

    if (close(fd) < 0)
    {
        LOGW("Error %d closing serial link: %s\n", errno, strerror(errno));
        ret = -1;
    }
    else
    {
        LOGD("Serial closed\n");
    }

    fd = -1;
    reset_buffer_state();

    pthread_mutex_unlock(&serial_mutex);
    return ret;
}

static int get_single_char(unsigned char * c, unsigned int timeout_ms)
{
    ssize_t read_bytes;
    fd_set readfs;
    struct timeval tv;
    int ret;

    if (c == NULL)
    {
        LOGE("NULL pointer passed to get_single_char\n");
        return -1;
    }

    // Do we still have char buffered?
    if (m_elem > 0)
    {
        *c = m_buffer[m_read++];
        m_elem--;
        return 1;
    }

    // Use select to wait for data with precise timeout
    FD_ZERO(&readfs);
    FD_SET(fd, &readfs);

    // Protect against overflow
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &readfs, NULL, NULL, &tv);

    if (ret < 0)
    {
        LOGE("Error in select: %d\n", errno);
        return -1;
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
         return -1;
    }

    // read returned 0 (EOF/disconnect)
    return 0;
}

/**
 * \brief   Read data from the serial link
 *
 * This function reads a single character from the serial port with timeout.
 * Uses internal buffering for efficiency.
 *
 * Thread-safe: This function uses internal locking.
 *
 * \param   c           Pointer to store the read character
 * \param   timeout_ms  Timeout in milliseconds
 * \return  1 if a character was read successfully
 *          0 on timeout
 *         -1 on error (including NULL pointer or no open port)
 */
int Serial_read(unsigned char * c, unsigned int timeout_ms)
{
    int ret;

    pthread_mutex_lock(&serial_mutex);

    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        pthread_mutex_unlock(&serial_mutex);
        return -1;
    }

    ret = get_single_char(c, timeout_ms);

    pthread_mutex_unlock(&serial_mutex);
    return ret;
}

/**
 * \brief   Write data to the serial link
 *
 * This function writes data to the serial port, handling partial writes automatically.
 * It will retry writing remaining data until all bytes are written or an error occurs.
 *
 * In case of fatal errors (EBADF, ENXIO, EIO), the connection is automatically closed
 * as the serial port is considered unusable. For transient errors (EINTR, EAGAIN),
 * the connection remains open and the function retries automatically.
 *
 * Thread-safe: This function uses internal locking.
 *
 * \param   buffer      Pointer to data to write
 * \param   buffer_size Number of bytes to write
 * \return  Number of bytes written on success (always equals buffer_size), -1 on error.
 *          After returning -1, caller should check if Serial_open is needed
 *          by attempting another operation or explicitly checking connection state.
 */
int Serial_write(const unsigned char * buffer, unsigned int buffer_size)
{
    ssize_t ret;
    size_t total_written = 0;
    const unsigned char *write_ptr = buffer;
    size_t remaining = buffer_size;

    if (buffer == NULL || buffer_size == 0)
    {
        LOGE("Invalid buffer or size\n");
        return -1;
    }

    pthread_mutex_lock(&serial_mutex);

    if (fd < 0)
    {
        LOGE("No serial link opened\n");
        pthread_mutex_unlock(&serial_mutex);
        return -1;
    }

    // Handle partial writes - loop until all data is written
    while (remaining > 0)
    {
        ret = write(fd, write_ptr, remaining);

        if (ret < 0)
        {
            // Check if this is a transient or fatal error
            bool fatal_error = false;

            switch (errno)
            {
                case EINTR:   // Interrupted system call - retry
                    LOGD("Write interrupted (EINTR), retrying...\n");
                    continue;

                case EAGAIN:  // Resource temporarily unavailable - retry
                    LOGW("Write would block (EAGAIN), retrying...\n");
                    // Brief yield to avoid busy-wait
                    usleep(1000); // 1ms
                    continue;

                case EBADF:   // Bad file descriptor
                case ENXIO:   // Device not configured
                case EIO:     // Input/output error
                case ENOTTY:  // Not a terminal device
                case ENODEV:  // No such device
                default:
                    // Fatal error - connection is broken
                    LOGE("Fatal write error: %d (%s), closing connection\n",
                         errno, strerror(errno));
                    fatal_error = true;
                    break;
            }

            if (fatal_error)
            {
                close(fd);
                fd = -1;
                reset_buffer_state();
                pthread_mutex_unlock(&serial_mutex);
                return -1;
            }
        }
        else if (ret == 0)
        {
            // write() returned 0 - this shouldn't happen for regular files/devices
            LOGE("Write returned 0 bytes (unexpected), closing connection\n");
            close(fd);
            fd = -1;
            reset_buffer_state();
            pthread_mutex_unlock(&serial_mutex);
            return -1;
        }
        else
        {
            // Successful partial or complete write
            total_written += ret;
            write_ptr += ret;
            remaining -= ret;

            if (ret < (ssize_t)remaining)
            {
                LOGD("Partial write: %zd of %zu bytes, continuing...\n", ret, remaining + ret);
            }
        }
    }

    pthread_mutex_unlock(&serial_mutex);
    return total_written;
}
