#pragma once
#include <stdint.h>
#include <string.h>

typedef void *esp_netif_t;

#define ESP_ERR_ESP_NETIF_INVALID_PARAMS ((esp_err_t)0x5001)

/* Mirror the implementation's current private cap without exporting IDF-private names. */
#define MOCK_NETIF_HOSTNAME_MAX_LEN 32

typedef struct { uint32_t addr; } esp_ip4_addr_t;
typedef struct { esp_ip4_addr_t ip, netmask, gw; } esp_netif_ip_info_t;
typedef struct { esp_netif_ip_info_t ip_info; } ip_event_got_ip_t;

#define IPSTR     "%d.%d.%d.%d"
#define IP2STR(a) ((int)(((a)->addr) & 0xFF)), ((int)((((a)->addr) >> 8) & 0xFF)), \
                  ((int)((((a)->addr) >> 16) & 0xFF)), ((int)((((a)->addr) >> 24) & 0xFF))

static esp_netif_t _mock_sta_netif;
static esp_err_t _mock_netif_set_hostname_result = ESP_OK;
static int _mock_netif_set_hostname_calls;

static inline int  esp_netif_init(void) { return 0; }
static inline esp_netif_t *esp_netif_create_default_wifi_sta(void) {
    return &_mock_sta_netif;
}
static inline esp_err_t esp_netif_set_hostname(esp_netif_t *n, const char *hostname) {
    (void)n;
    _mock_netif_set_hostname_calls++;
    if (hostname == NULL || strlen(hostname) > MOCK_NETIF_HOSTNAME_MAX_LEN) {
        return ESP_ERR_ESP_NETIF_INVALID_PARAMS;
    }
    return _mock_netif_set_hostname_result;
}
static inline int  esp_netif_get_ip_info(esp_netif_t *n, esp_netif_ip_info_t *i) {
    (void)n; (void)i; return 0;
}
