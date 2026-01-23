/* Wirepas Oy licensed under Apache License, Version 2.0
 *
 * See file LICENSE for full license details.
 *
 */
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdint.h>

#define LOG_MODULE_NAME "darwin_plat"
#define MAX_LOG_LEVEL INFO_LOG_LEVEL
#include "logger.h"
#include "platform.h"
#include "reassembly.h"
#include "wpc_proto.h"

// Maximum number of indication to be retrieved from a single poll
#define MAX_NUMBER_INDICATION 30U

// Polling interval to check for indication
#define POLLING_INTERVAL_MS 20

// Wakeup timeout for dispatch thread, mainly for garbage collection of fragments
#define DISPATCH_WAKEUP_TIMEOUT_S 5

// Maximum timestamp value to prevent overflow (year 2262)
#define MAX_SAFE_TIMESTAMP_SEC ((unsigned long long)((ULLONG_MAX / 1000) - 1))

// Mutex for sending, ie serial access
static pthread_mutex_t sending_mutex;

// This thread is used to poll for indication
static pthread_t thread_polling;

typedef enum {
    POLLING_THREAD_RUN,
    POLLING_THREAD_STOP,
    POLLING_THREAD_STOP_REQUESTED
} polling_thread_state_t;

// Request to handle polling thread state - protected by state_mutex
static polling_thread_state_t m_polling_thread_state_request = POLLING_THREAD_STOP;

// Mutex to protect polling thread state
static pthread_mutex_t m_state_mutex;

// This thread is used to dispatch indication
static pthread_t thread_dispatch;

// Set to false to stop dispatch thread execution - protected by state_mutex
static bool m_dispatch_thread_running = false;

// Track if platform has been initialized
static bool m_platform_initialized = false;

// Service to call to get indication from a node
static Platform_get_indication_f m_get_indication_f = NULL;

// Service to call to dispatch/handle indication from node
static Platform_dispatch_indication_f m_dispatch_indication_f = NULL;

/*****************************************************************************/
/*                Indication queue related variables                        */
/*****************************************************************************/

// Size of the queue between getter and dispatcher of indication
// In most of the cases, the dispatcher is supposed to be faster and the
// queue will have only one element
// But if some execution handling are too long or require to send something over
// UART, the dispatching thread will be hanged until the poll thread finish its
// work
#define MAX_NUMBER_INDICATION_QUEUE (MAX_NUMBER_INDICATION * 2)

// Struct that describes a received frame with its timestamp
typedef struct
{
    wpc_frame_t frame;                      //< The received frame
    unsigned long long timestamp_ms_epoch;  //< The timestamp of reception
} timestamped_frame_t;

// Indications queue
static timestamped_frame_t m_indications_queue[MAX_NUMBER_INDICATION_QUEUE];

// Head of the queue for the polling thread to write
static unsigned int m_ind_queue_write = 0;

// Tail of the queue to read from dispatching thread
static unsigned int m_ind_queue_read = 0;

// Is queue empty?
static bool m_queue_empty = true;

// Mutex and condition variable used for the dispatching queue to wait
static pthread_mutex_t m_queue_mutex;
static pthread_cond_t m_queue_not_empty_cond = PTHREAD_COND_INITIALIZER;

/*****************************************************************************/
/*                Helper functions for thread-safe state access              */
/*****************************************************************************/
static polling_thread_state_t get_polling_thread_state()
{
    polling_thread_state_t state;
    pthread_mutex_lock(&m_state_mutex);
    state = m_polling_thread_state_request;
    pthread_mutex_unlock(&m_state_mutex);
    return state;
}

static void set_polling_thread_state(polling_thread_state_t state)
{
    pthread_mutex_lock(&m_state_mutex);
    m_polling_thread_state_request = state;
    pthread_mutex_unlock(&m_state_mutex);
}

static bool get_dispatch_thread_running()
{
    bool running;
    pthread_mutex_lock(&m_state_mutex);
    running = m_dispatch_thread_running;
    pthread_mutex_unlock(&m_state_mutex);
    return running;
}

static void set_dispatch_thread_running(bool running)
{
    pthread_mutex_lock(&m_state_mutex);
    m_dispatch_thread_running = running;
    pthread_mutex_unlock(&m_state_mutex);
}

/*****************************************************************************/
/*                Dispatch indication Thread implementation                  */
/*****************************************************************************/
/**
 * \brief   Thread to dispatch indication in a non locked environment
 */
static void * dispatch_indication(void * unused)
{
    (void) unused;
    struct timespec ts;

    pthread_mutex_lock(&m_queue_mutex);
    while (get_dispatch_thread_running())
    {
        if (m_queue_empty)
        {
            // Queue is empty, wait
            if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
            {
                LOGE("Failed to get time\n");
                break;
            }
            ts.tv_sec += DISPATCH_WAKEUP_TIMEOUT_S;  // 5 second timeout
            pthread_cond_timedwait(&m_queue_not_empty_cond, &m_queue_mutex, &ts);

            // Force a garbage collect (to be sure it's called even if no frag are received)
            reassembly_garbage_collect();

            // Check if we wake up but nothing in queue
            if (m_queue_empty)
            {
                // Continue to evaluate the stop condition
                continue;
            }
        }

        // Get the oldest indication
        timestamped_frame_t * ind = &m_indications_queue[m_ind_queue_read];

        // Copy the indication to dispatch it without holding the lock
        timestamped_frame_t ind_copy;
        memcpy(&ind_copy, ind, sizeof(timestamped_frame_t));

        // Handle the indication without the queue lock
        pthread_mutex_unlock(&m_queue_mutex);

        // Validate function pointer before calling
        if (m_dispatch_indication_f != NULL)
        {
            m_dispatch_indication_f(&ind_copy.frame, ind_copy.timestamp_ms_epoch);
        }
        else
        {
            LOGE("Dispatch function is NULL\n");
        }

        // Take the lock back to update empty status and wait on cond again in
        // next loop iteration
        pthread_mutex_lock(&m_queue_mutex);

        m_ind_queue_read = (m_ind_queue_read + 1) % MAX_NUMBER_INDICATION_QUEUE;
        if (m_ind_queue_read == m_ind_queue_write)
        {
            // Indication was the last one
            m_queue_empty = true;
        }
    }

    pthread_mutex_unlock(&m_queue_mutex);

    LOGW("Exiting dispatch thread\n");
    return NULL;
}

/*****************************************************************************/
/*                Polling Thread implementation                              */
/*****************************************************************************/
static void onIndicationReceivedLocked(wpc_frame_t * frame, unsigned long long timestamp_ms)
{
    if (frame == NULL)
    {
        LOGE("Received NULL frame pointer\n");
        return;
    }

    LOGD("Frame received with timestamp = %lld\n", timestamp_ms);
    pthread_mutex_lock(&m_queue_mutex);

    // Check if queue is full
    if (!m_queue_empty && (m_ind_queue_write == m_ind_queue_read))
    {
        // Queue is FULL
        LOGE("No more room for indications! Dropping frame!\n");
        pthread_mutex_unlock(&m_queue_mutex);
        return;
    }

    // Insert our received indication
    timestamped_frame_t * ind = &m_indications_queue[m_ind_queue_write];
    memcpy(&ind->frame, frame, sizeof(wpc_frame_t));
    ind->timestamp_ms_epoch = timestamp_ms;

    m_ind_queue_write = (m_ind_queue_write + 1) % MAX_NUMBER_INDICATION_QUEUE;
    // At least one indication ready, signal it
    m_queue_empty = false;
    pthread_cond_signal(&m_queue_not_empty_cond);

    pthread_mutex_unlock(&m_queue_mutex);
}

/**
 * \brief   Polling tread.
 *          This thread polls for indication and insert them to the queue
 *          shared with the dispatcher thread
 */
static void * poll_for_indication(void * unused)
{
    (void) unused;
    unsigned int max_num_indication, free_buffer_room;
    int get_ind_res;
    // Initially wait for 500ms before any polling
    uint32_t wait_before_next_polling_ms = 500;
    polling_thread_state_t current_state;

    set_polling_thread_state(POLLING_THREAD_RUN);

    while ((current_state = get_polling_thread_state()) != POLLING_THREAD_STOP)
    {
        usleep(wait_before_next_polling_ms * 1000);

        if(current_state == POLLING_THREAD_STOP_REQUESTED)
        {
            pthread_mutex_lock(&m_queue_mutex);
            bool queue_empty_local = m_queue_empty;
            pthread_mutex_unlock(&m_queue_mutex);

            if (!queue_empty_local)
            {
                // Dispatch did not process all indications. Just wait for it to complete.
                wait_before_next_polling_ms = POLLING_INTERVAL_MS;
                continue;
            }

            if (reassembly_is_queue_empty())
            {
                LOGI("Reassembly queue is empty, exiting polling thread\n");
                set_polling_thread_state(POLLING_THREAD_STOP);
                break;
            }
        }

        // Get the number of free buffers in the indication queue
        pthread_mutex_lock(&m_queue_mutex);
        
        /* Ask for maximum room in buffer queue and less than MAX */
        if (!m_queue_empty && (m_ind_queue_write == m_ind_queue_read))
        {
            // Queue is FULL, wait for POLLING INTERVALL to give some
            // time for the dispatching thread to handle them
            LOGW("Queue is full, do not poll\n");
            pthread_mutex_unlock(&m_queue_mutex);
            wait_before_next_polling_ms = POLLING_INTERVAL_MS;
            continue;
        }
        else if (m_queue_empty)
        {
            // Queue is empty
            free_buffer_room = MAX_NUMBER_INDICATION_QUEUE;
        }
        else
        {
            // Queue has elements, determine the number of free buffers
            if (m_ind_queue_write > m_ind_queue_read)
            {
                free_buffer_room = MAX_NUMBER_INDICATION_QUEUE -
                                   (m_ind_queue_write - m_ind_queue_read);
            }
            else
            {
                free_buffer_room = m_ind_queue_read - m_ind_queue_write;
            }
        }
        
        pthread_mutex_unlock(&m_queue_mutex);

        if (current_state == POLLING_THREAD_STOP_REQUESTED)
        {
            // In case we are about to stop, let's poll only one by one to have more chance to
            // finish uncomplete fragmented packet and not start to receive a new one
            max_num_indication = 1;
            LOGD("Poll for one more fragment to empty reassembly queue\n");
        }
        else
        {
            // Let's read max indications that can fit in the queue
            max_num_indication = MIN(MAX_NUMBER_INDICATION, free_buffer_room);
        }

        LOGD("Poll for %d indications\n", max_num_indication);

        // Validate function pointer before calling
        if (m_get_indication_f == NULL)
        {
            LOGE("Get indication function is NULL\n");
            break;
        }

        get_ind_res = m_get_indication_f(max_num_indication, onIndicationReceivedLocked);

        current_state = get_polling_thread_state();

        if ((get_ind_res == 1) && (current_state != POLLING_THREAD_STOP_REQUESTED))
        {
            // Still pending indication, only wait 1 ms to give a chance
            // to other threads but not more to have better throughput
            wait_before_next_polling_ms = 1;
        }
        else
        {
            // In case of error or if no more indication, just wait
            // the POLLING INTERVAL to avoid polling all the time
            // In case of stop request, wait for to give time to push data received
            wait_before_next_polling_ms = POLLING_INTERVAL_MS;
        }
    }

    LOGW("Exiting polling thread\n");

    return NULL;
}

bool Platform_lock_request()
{
    int res = pthread_mutex_lock(&sending_mutex);
    if (res != 0)
    {
        // It must never happen but add a check and
        // return to avoid a deadlock
        if (res == EINVAL)
        {
            LOGW("Mutex no longer exists (destroyed)\n");
        }
        else
        {
            LOGE("Mutex lock failed %d\n", res);
        }
        return false;
    }
    return true;
}

void Platform_unlock_request()
{
    pthread_mutex_unlock(&sending_mutex);
}

unsigned long long Platform_get_timestamp_ms_epoch()
{
    struct timespec spec;

    // Get timestamp in ms since epoch
    if (clock_gettime(CLOCK_REALTIME, &spec) != 0)
    {
        LOGE("Failed to get REALTIME clock\n");
        return 0;
    }

    // Check for potential overflow
    if ((unsigned long long)spec.tv_sec > MAX_SAFE_TIMESTAMP_SEC)
    {
        LOGE("Timestamp overflow risk detected\n");
        return ULLONG_MAX;
    }

    return ((unsigned long long) spec.tv_sec) * 1000 + (spec.tv_nsec) / 1000000;
}

unsigned long long Platform_get_timestamp_ms_monotonic()
{
    struct timespec spec;

    // Get timestamp in ms since epoch
    if (clock_gettime(CLOCK_MONOTONIC, &spec) != 0)
    {
        LOGE("Failed to get MONOTONIC clock\n");
        return 0;
    }

    // Check for potential overflow
    if ((unsigned long long)spec.tv_sec > MAX_SAFE_TIMESTAMP_SEC)
    {
        LOGE("Timestamp overflow risk detected\n");
        return ULLONG_MAX;
    }

    return ((unsigned long long) spec.tv_sec) * 1000 + (spec.tv_nsec) / 1000000;
}

void * Platform_malloc(size_t size)
{
    LOGD("M: %d\n", size);
    return malloc(size);
}

void Platform_free(void *ptr, size_t size)
{
    (void) size;
    free(ptr);
    LOGD("F: %d\n", size);
}

bool Platform_init(Platform_get_indication_f get_indication_f,
                   Platform_dispatch_indication_f dispatch_indication_f)
{
    /* This linux implementation uses a dedicated thread
     * to poll for indication. The indication are then handled
     * by this thread.
     * All the other API calls can be made on different threads
     * as this platform implements the lock mechanism in order
     * to protect the access to critical sections.
     */
    pthread_mutexattr_t attr;

    if (get_indication_f == NULL || dispatch_indication_f == NULL)
    {
        LOGE("Invalid parameters\n");
        return false;
    }

    if (m_platform_initialized)
    {
        LOGE("Platform already initialized\n");
        return false;
    }

    m_get_indication_f = get_indication_f;
    m_dispatch_indication_f = dispatch_indication_f;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);

    // Initialize mutex for state protection
    if (pthread_mutex_init(&m_state_mutex, &attr) != 0)
    {
        LOGE("State Mutex init failed\n");
        pthread_mutexattr_destroy(&attr);
        return false;
    }

    // Initialize mutex to access critical section
    if (pthread_mutex_init(&sending_mutex, &attr) != 0)
    {
        LOGE("Sending Mutex init failed\n");
        pthread_mutex_destroy(&m_state_mutex);
        pthread_mutexattr_destroy(&attr);
        return false;
    }

    // Initialize mutex to access queue
    if (pthread_mutex_init(&m_queue_mutex, &attr) != 0)
    {
        LOGE("Queue Mutex init failed\n");
        pthread_mutex_destroy(&sending_mutex);
        pthread_mutex_destroy(&m_state_mutex);
        pthread_mutexattr_destroy(&attr);
        return false;
    }

    pthread_mutexattr_destroy(&attr);

    // Start a thread to poll for indication
    if (pthread_create(&thread_polling, NULL, poll_for_indication, NULL) != 0)
    {
        LOGE("Cannot create polling thread\n");
        pthread_mutex_destroy(&m_queue_mutex);
        pthread_mutex_destroy(&sending_mutex);
        pthread_mutex_destroy(&m_state_mutex);
        return false;
    }

    set_dispatch_thread_running(true);
    
    // Start a thread to dispatch indication
    if (pthread_create(&thread_dispatch, NULL, dispatch_indication, NULL) != 0)
    {
        LOGE("Cannot create dispatch thread\n");
        
        // Stop polling thread gracefully
        set_polling_thread_state(POLLING_THREAD_STOP);
        pthread_join(thread_polling, NULL);
        
        pthread_mutex_destroy(&m_queue_mutex);
        pthread_mutex_destroy(&sending_mutex);
        pthread_mutex_destroy(&m_state_mutex);
        return false;
    }

    m_platform_initialized = true;
    return true;
}

void Platform_close()
{
    void * res;
    pthread_t cur_thread = pthread_self();

    if (!m_platform_initialized)
    {
        LOGW("Platform not initialized, nothing to close\n");
        return;
    }

    // Signal our polling thread to stop
    // No need to signal it as it will wakeup periodically
    set_polling_thread_state(POLLING_THREAD_STOP_REQUESTED);

    // Wait for polling tread to finish
    if (cur_thread != thread_polling)
    {
        pthread_join(thread_polling, &res);
    }

    // Signal our dispatch thread to stop
    set_dispatch_thread_running(false);
    
    // Signal condition to wakeup thread
    pthread_cond_signal(&m_queue_not_empty_cond);

    // Wait for dispatch tread to finish
    if (cur_thread != thread_dispatch)
    {
        pthread_join(thread_dispatch, &res);
    }

    // Destroy our mutexes
    pthread_mutex_destroy(&m_queue_mutex);
    pthread_mutex_destroy(&sending_mutex);
    pthread_mutex_destroy(&m_state_mutex);

    // Reset function pointers
    m_get_indication_f = NULL;
    m_dispatch_indication_f = NULL;

    m_platform_initialized = false;
}
