#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_timer.h"   /* _mock_timer_us, so a scripted transfer can age past its deadline */

typedef void *esp_http_client_handle_t;

typedef enum {
    HTTP_EVENT_ERROR = 0,
    HTTP_EVENT_ON_CONNECTED,
    HTTP_EVENT_HEADERS_SENT,
    HTTP_EVENT_ON_HEADER,
    HTTP_EVENT_ON_DATA,
    HTTP_EVENT_ON_FINISH,
    HTTP_EVENT_DISCONNECTED,
} esp_http_client_event_id_t;

typedef struct {
    esp_http_client_event_id_t  event_id;
    esp_http_client_handle_t    client;
    void   *data;
    int     data_len;
    char   *header_key;
    char   *header_value;
    void   *user_data;
} esp_http_client_event_t;

typedef int (*http_event_handle_cb)(esp_http_client_event_t *);

typedef struct {
    const char           *url;
    http_event_handle_cb  event_handler;
    void                 *user_data;
    int                   timeout_ms;
    int                   buffer_size;
    int                   buffer_size_tx;
    bool                  disable_auto_redirect;
} esp_http_client_config_t;

typedef enum {
    HTTP_METHOD_GET = 0,
    HTTP_METHOD_POST,
    HTTP_METHOD_PUT,
    HTTP_METHOD_DELETE,
} esp_http_client_method_t;

/* ── Programmable mock ──────────────────────────────────────────────────
 * esp_http_client_perform replays a test-scripted sequence of events into the
 * handler registered at init (mock_http_push_* to build it), then returns a
 * scripted result + status code. Defaults (empty script, result -1, status 0)
 * reproduce the original stub so suites that don't touch these controls are
 * unaffected. esp_http_client_close is counted. */
#define MOCK_HTTP_MAX_EVENTS 16
typedef struct {
    esp_http_client_event_id_t id;
    char *key;
    char *val;
    void *data;
    int   data_len;
} mock_http_event_t;

static struct {
    http_event_handle_cb handler;
    void   *user_data;
    mock_http_event_t script[MOCK_HTTP_MAX_EVENTS];
    int     script_len;
    int     perform_result;   /* returned by esp_http_client_perform */
    int     status_code;      /* returned by esp_http_client_get_status_code */
    int     close_calls;
    int64_t advance_us;       /* added to the mock clock before each replayed event */
    bool    disable_auto_redirect_seen;
} _mock_http;

static inline void mock_http_reset(void) {
    _mock_http.handler = NULL;
    _mock_http.user_data = NULL;
    _mock_http.script_len = 0;
    _mock_http.perform_result = -1;
    _mock_http.status_code = 0;
    _mock_http.close_calls = 0;
    _mock_http.advance_us = 0;
    _mock_http.disable_auto_redirect_seen = false;
}
static inline void mock_http_set_advance_us(int64_t us) { _mock_http.advance_us = us; }
static inline void mock_http_set_result(int perform_result, int status_code) {
    _mock_http.perform_result = perform_result;
    _mock_http.status_code = status_code;
}
static inline void mock_http_push_event(esp_http_client_event_id_t id, char *key,
                                        char *val, void *data, int data_len) {
    if (_mock_http.script_len >= MOCK_HTTP_MAX_EVENTS) return;
    mock_http_event_t *e = &_mock_http.script[_mock_http.script_len++];
    e->id = id; e->key = key; e->val = val; e->data = data; e->data_len = data_len;
}
static inline void mock_http_push_header(char *key, char *val) {
    mock_http_push_event(HTTP_EVENT_ON_HEADER, key, val, NULL, 0);
}
static inline void mock_http_push_data(void *data, int len) {
    mock_http_push_event(HTTP_EVENT_ON_DATA, NULL, NULL, data, len);
}

static inline esp_http_client_handle_t esp_http_client_init(const esp_http_client_config_t *c) {
    if (c) {
        _mock_http.handler = c->event_handler;
        _mock_http.user_data = c->user_data;
        _mock_http.disable_auto_redirect_seen = c->disable_auto_redirect;
    }
    return (void *)1;
}
static inline int esp_http_client_set_header(esp_http_client_handle_t c, const char *k, const char *v) {
    (void)c; (void)k; (void)v; return 0;
}
static inline int esp_http_client_perform(esp_http_client_handle_t c) {
    if (_mock_http.handler) {
        for (int i = 0; i < _mock_http.script_len; i++) {
            _mock_timer_us += _mock_http.advance_us;   /* age the transfer per event */
            mock_http_event_t *s = &_mock_http.script[i];
            esp_http_client_event_t evt = {
                .event_id = s->id, .client = c, .user_data = _mock_http.user_data,
                .header_key = s->key, .header_value = s->val,
                .data = s->data, .data_len = s->data_len,
            };
            _mock_http.handler(&evt);
        }
    }
    return _mock_http.perform_result;
}
static inline int esp_http_client_get_status_code(esp_http_client_handle_t c) {
    (void)c; return _mock_http.status_code;
}
static inline int esp_http_client_cleanup(esp_http_client_handle_t c) { (void)c; return 0; }
static inline int esp_http_client_close(esp_http_client_handle_t c) {
    (void)c; _mock_http.close_calls++; return 0;
}
static inline int esp_http_client_set_method(esp_http_client_handle_t c, esp_http_client_method_t m) {
    (void)c; (void)m; return 0;
}
static inline int esp_http_client_set_post_field(esp_http_client_handle_t c, const char *data, int len) {
    (void)c; (void)data; (void)len; return 0;
}
