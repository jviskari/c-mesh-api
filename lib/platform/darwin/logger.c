/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#include <stdint.h>
#include <stdio.h>

#include <time.h>

/* Full log level string  */
static char DEBUG[] = "DEBUG";
static char INFO[] = "INFO";
static char WARNING[] = "WARNING";
static char ERROR[] = "ERROR";
static char UNKNOWN[] = "UNKNOWN";

static inline void get_timestamp(char * timestamp, size_t size)
{
    struct tm result;
    uint16_t ms;
    time_t s;
    struct timespec spec;

    if (!timestamp || size < 24)
    {
        return;
    }

    clock_gettime(CLOCK_REALTIME, &spec);

    s = spec.tv_sec;
    localtime_r(&s, &result);

    ms = (uint16_t) (spec.tv_nsec / 1.0e6);  // Convert nanoseconds to milliseconds

    // Use snprintf for safety
    snprintf(timestamp,
             size,
             "%04d-%02d-%02d %02d:%02d:%02d,%03d",
             result.tm_year + 1900,  // tm_year is in year - 1900
             result.tm_mon + 1,      // tm_mon is in [0-11]
             result.tm_mday,
             result.tm_hour,
             result.tm_min,
             result.tm_sec,
             ms);
}

static inline void print_prefix(char level, char * module)
{
    // Timestamp should always feat to 23 char, but some margins
    char timestamp[50];
    char * full_level;

    if (!module)
    {
        return;
    }

    switch (level)
    {
        case ('D'):
            full_level = DEBUG;
            break;
        case ('I'):
            full_level = INFO;
            break;
        case ('W'):
            full_level = WARNING;
            break;
        case ('E'):
            full_level = ERROR;
            break;
        default:
            full_level = UNKNOWN;
    }
    get_timestamp(timestamp, sizeof(timestamp));
    printf("%s | [%s] %s:", timestamp, full_level, module);
}

void Platform_LOG(char level, char * module, char * format, va_list args)
{
    if (!module || !format)
    {
        return;
    }

    print_prefix(level, module);
    vprintf(format, args);
    printf("\n");
}

void Platform_print_buffer(uint8_t * buffer, int size)
{
    int i;

    if (!buffer || size <= 0)
    {
        return;
    }

    for (i = 0; i < size; i++)
    {
        printf("%02x ", buffer[i]);
        if ((i & 0xF) == 0xF)
            printf("\n");
    }
    printf("\n");
}
