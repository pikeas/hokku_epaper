#pragma once
#include "FreeRTOS.h"
#include "../esp_timer.h"   /* _mock_timer_us, so a scripted wait can advance the clock */

typedef uint32_t EventBits_t;
typedef void    *EventGroupHandle_t;

/* ── Programmable mock ──────────────────────────────────────────────────
 * xEventGroupWaitBits returns the next value pushed with mock_eg_push(); an
 * empty queue returns 0 (a timeout, matching the original always-0 stub, so
 * suites that never touch these controls are unaffected). Each wait can also
 * advance the mock monotonic clock (mock_eg_set_advance_us) to exercise
 * deadline logic, and every wait is counted for bounded-retry assertions. */
#define MOCK_EG_QUEUE_MAX 32
static EventBits_t _mock_eg_queue[MOCK_EG_QUEUE_MAX];
static int         _mock_eg_queue_len;
static int         _mock_eg_queue_pos;
static int64_t     _mock_eg_advance_us;
static int         _mock_eg_wait_calls;

static inline void mock_eg_reset(void) {
    _mock_eg_queue_len = 0;
    _mock_eg_queue_pos = 0;
    _mock_eg_advance_us = 0;
    _mock_eg_wait_calls = 0;
}
static inline void mock_eg_push(EventBits_t bits) {
    if (_mock_eg_queue_len < MOCK_EG_QUEUE_MAX)
        _mock_eg_queue[_mock_eg_queue_len++] = bits;
}
static inline void mock_eg_set_advance_us(int64_t us) { _mock_eg_advance_us = us; }

static inline EventGroupHandle_t xEventGroupCreate(void) { return (void *)1; }
static inline EventBits_t xEventGroupSetBits(EventGroupHandle_t g, EventBits_t b) {
    (void)g; return b;
}
static inline EventBits_t xEventGroupClearBits(EventGroupHandle_t g, EventBits_t b) {
    (void)g; return b;
}
static inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t g, EventBits_t bits,
                                               BaseType_t clear, BaseType_t all,
                                               TickType_t t) {
    (void)g; (void)bits; (void)clear; (void)all; (void)t;
    _mock_eg_wait_calls++;
    _mock_timer_us += _mock_eg_advance_us;
    if (_mock_eg_queue_pos < _mock_eg_queue_len)
        return _mock_eg_queue[_mock_eg_queue_pos++];
    return 0;
}
