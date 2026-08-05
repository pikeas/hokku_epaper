#pragma once
/* Minimal stub of esp_ota_ops.h for host-side unit tests. */

#include <stddef.h>
/* esp_err_t / ESP_OK / ESP_FAIL come from freertos/FreeRTOS.h which is
   always included before this header in the firmware translation units. */
#include "esp_partition.h"

typedef int esp_ota_handle_t;

typedef enum {
    ESP_OTA_IMG_NEW         = 0x0,
    ESP_OTA_IMG_PENDING_VERIFY,
    ESP_OTA_IMG_VALID,
    ESP_OTA_IMG_INVALID,
    ESP_OTA_IMG_ABORTED,
    ESP_OTA_IMG_UNDEFINED   = 0xFFFFFFFF,
} esp_ota_img_states_t;

#define OTA_SIZE_UNKNOWN 0xFFFFFFFF

static esp_partition_t _mock_ota_running_partition;
static bool _mock_ota_running_present;
static esp_err_t _mock_ota_state_result = ESP_FAIL;
static esp_ota_img_states_t _mock_ota_state = ESP_OTA_IMG_UNDEFINED;
static int _mock_ota_mark_valid_calls;

static inline void mock_ota_reset(void) {
    _mock_ota_running_present = false;
    _mock_ota_state_result = ESP_FAIL;
    _mock_ota_state = ESP_OTA_IMG_UNDEFINED;
    _mock_ota_mark_valid_calls = 0;
}
static inline void mock_ota_set_running_state(esp_ota_img_states_t state) {
    _mock_ota_running_present = true;
    _mock_ota_state_result = ESP_OK;
    _mock_ota_state = state;
}

static inline const esp_partition_t *esp_ota_get_running_partition(void) {
    return _mock_ota_running_present ? &_mock_ota_running_partition : NULL;
}
static inline const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *start) { (void)start; return NULL; }
static inline esp_err_t esp_ota_begin(const esp_partition_t *partition, size_t image_size, esp_ota_handle_t *out_handle) { (void)partition; (void)image_size; (void)out_handle; return ESP_OK; }
static inline esp_err_t esp_ota_write(esp_ota_handle_t handle, const void *data, size_t size) { (void)handle; (void)data; (void)size; return ESP_OK; }
static inline esp_err_t esp_ota_end(esp_ota_handle_t handle) { (void)handle; return ESP_OK; }
static inline esp_err_t esp_ota_abort(esp_ota_handle_t handle) { (void)handle; return ESP_OK; }
static inline esp_err_t esp_ota_set_boot_partition(const esp_partition_t *partition) { (void)partition; return ESP_OK; }
static inline esp_err_t esp_ota_get_state_partition(const esp_partition_t *partition, esp_ota_img_states_t *ota_state) { (void)partition; if (ota_state) *ota_state = _mock_ota_state; return _mock_ota_state_result; }
static inline esp_err_t esp_ota_mark_app_valid_cancel_rollback(void) { _mock_ota_mark_valid_calls++; return ESP_OK; }
