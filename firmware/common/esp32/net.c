#include "net.h"
#include "log.h"    /* hokku_log_snapshot / hokku_log_reset / HOKKU_LOG_MAX_UPLOAD */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "esp_http_client.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#define IMAGE_TRANSFER_DEADLINE_MS  90000

typedef struct {
    uint8_t *buf;
    size_t   received;
    size_t   capacity;
    int64_t  transfer_start_us;
    bool     deadline_hit;
    /* Response-header captures, populated from HTTP_EVENT_ON_HEADER and read
     * after perform(). (Capturing from the event stream is the only correct
     * way — esp_http_client_get_header() reads REQUEST headers, not response.) */
    char     sleep_seconds_hdr[32];
    char     server_epoch_hdr[32];
    char     content_id_hdr[16];
    /* X-Firmware-Update: <version> — present when the server wants this device
     * to OTA. The body (image) is ignored when set. */
    char     fw_update_hdr[48];
} http_download_ctx_t;

void hokku_enforce_transfer_deadline(int64_t start_us, int32_t deadline_ms,
                                     esp_http_client_handle_t client,
                                     bool *deadline_hit, const char *what)
{
    if ((esp_timer_get_time() - start_us) / 1000 <= deadline_ms) return;
    if (deadline_hit && !*deadline_hit) {
        ESP_LOGW("hokku", "Transfer deadline (%d s) exceeded; aborting %s",
                 (int)(deadline_ms / 1000), what);
        *deadline_hit = true;
    }
    esp_http_client_close(client);
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_download_ctx_t *ctx = (http_download_ctx_t *)evt->user_data;
    if (!ctx) return ESP_OK;

    switch (evt->event_id) {
        case HTTP_EVENT_ON_CONNECTED:
            /* Reset buffer on each new connection (handles redirects): without
             * this, a 308 redirect's body accumulates before the real image
             * data, causing a size mismatch. Header captures reset too so we
             * only see the final response's values. */
            ctx->received = 0;
            ctx->sleep_seconds_hdr[0] = '\0';
            ctx->server_epoch_hdr[0]  = '\0';
            ctx->content_id_hdr[0]    = '\0';
            ctx->fw_update_hdr[0]     = '\0';
            break;
        case HTTP_EVENT_ON_HEADER:
            hokku_enforce_transfer_deadline(ctx->transfer_start_us,
                                            IMAGE_TRANSFER_DEADLINE_MS, evt->client,
                                            &ctx->deadline_hit, "download");
            if (evt->header_key && evt->header_value) {
                if (strcasecmp(evt->header_key, "X-Sleep-Seconds") == 0) {
                    strncpy(ctx->sleep_seconds_hdr, evt->header_value,
                            sizeof(ctx->sleep_seconds_hdr) - 1);
                    ctx->sleep_seconds_hdr[sizeof(ctx->sleep_seconds_hdr) - 1] = '\0';
                } else if (strcasecmp(evt->header_key, "X-Server-Time-Epoch") == 0) {
                    strncpy(ctx->server_epoch_hdr, evt->header_value,
                            sizeof(ctx->server_epoch_hdr) - 1);
                    ctx->server_epoch_hdr[sizeof(ctx->server_epoch_hdr) - 1] = '\0';
                } else if (strcasecmp(evt->header_key, "X-Content-Id") == 0) {
                    strncpy(ctx->content_id_hdr, evt->header_value,
                            sizeof(ctx->content_id_hdr) - 1);
                    ctx->content_id_hdr[sizeof(ctx->content_id_hdr) - 1] = '\0';
                } else if (strcasecmp(evt->header_key, "X-Firmware-Update") == 0) {
                    strncpy(ctx->fw_update_hdr, evt->header_value,
                            sizeof(ctx->fw_update_hdr) - 1);
                    ctx->fw_update_hdr[sizeof(ctx->fw_update_hdr) - 1] = '\0';
                }
            }
            break;
        case HTTP_EVENT_ON_DATA:
            hokku_enforce_transfer_deadline(ctx->transfer_start_us,
                                            IMAGE_TRANSFER_DEADLINE_MS, evt->client,
                                            &ctx->deadline_hit, "download");
            if (ctx->received + evt->data_len <= ctx->capacity) {
                memcpy(ctx->buf + ctx->received, evt->data, evt->data_len);
                ctx->received += evt->data_len;
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

bool hokku_http_fetch_image(uint8_t *buf, size_t expect_bytes,
                            const char *url, const char *screen_name,
                            const char *screen_model, const char *frame_state,
                            const char *fw_build, const char *if_content_id,
                            hokku_fetch_out_t *out)
{
    http_download_ctx_t ctx = {
        .buf = buf,
        .received = 0,
        .capacity = expect_bytes,
        .transfer_start_us = esp_timer_get_time(),
        .deadline_hit = false,
    };

    esp_http_client_config_t http_cfg = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .timeout_ms = HTTP_TIMEOUT_MS,
        .buffer_size = 4096,
        /* TX (request) buffer. The default is DEFAULT_HTTP_BUF_SIZE = 512,
         * which our request headers exceed: X-Frame-State alone is up to the
         * caller's ~384-byte JSON, on top of X-Screen-Name/Model and the two
         * firmware headers. esp_http_client tolerates that by writing headers
         * across several passes, but it logs an ESP_LOGE each time and — if any
         * SINGLE header ever exceeds this buffer — silently drops that header
         * and every one after it. Size it well past our largest single header. */
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE("hokku", "esp_http_client_init failed (OOM?)");
        return false;
    }

    /* POST so the ring-buffer log can travel as the request body. */
    esp_http_client_set_method(client, HTTP_METHOD_POST);

    if (screen_name && screen_name[0] != '\0')
        esp_http_client_set_header(client, "X-Screen-Name", screen_name);
    if (screen_model && screen_model[0] != '\0')
        esp_http_client_set_header(client, "X-Screen-Model", screen_model);
    esp_http_client_set_header(client, "X-Frame-State", frame_state);

    const esp_app_desc_t *app = esp_app_get_description();
    const char *fw_ver = (app && app->version[0]) ? app->version : "unknown";
    esp_http_client_set_header(client, "X-Firmware-Version", fw_ver);
    esp_http_client_set_header(client, "X-Firmware-Build", fw_build);
    if (if_content_id != NULL && if_content_id[0] != '\0')
        esp_http_client_set_header(client, "X-Content-Id", if_content_id);

    /* Attach the log ring (carry + active, joined) as the POST body. Size the
     * receiving buffer to the max joined length so nothing is truncated.
     * Allocated in PSRAM: it's up to 22 KB and this runs during the WiFi+HTTP
     * window, the peak internal-DRAM-pressure moment (WiFi/lwIP live in DRAM). */
    char *log_body = heap_caps_malloc(HOKKU_LOG_MAX_UPLOAD, MALLOC_CAP_SPIRAM);
    int   log_body_len = 0;
    if (log_body) {
        log_body_len = (int)hokku_log_snapshot(log_body, HOKKU_LOG_MAX_UPLOAD);
        if (log_body_len == 0) { free(log_body); log_body = NULL; }  /* free() is valid on heap_caps mem */
    }
    if (log_body) {
        esp_http_client_set_header(client, "Content-Type", "text/plain");
        esp_http_client_set_post_field(client, log_body, log_body_len);
    }

    int64_t perform_start_us = esp_timer_get_time();
    esp_err_t err = esp_http_client_perform(client);
    unsigned long perform_elapsed_ms = (unsigned long)
        ((esp_timer_get_time() - perform_start_us) / 1000);
    if (ctx.deadline_hit) {
        err = ESP_FAIL;
    }
    int status = esp_http_client_get_status_code(client);

    /* Response headers captured during perform() — safe to read now (copied
     * into ctx, not pointers into esp_http_client internals). */
    if (ctx.sleep_seconds_hdr[0] != '\0' && out && out->out_sleep_seconds) {
        int32_t secs = atoi(ctx.sleep_seconds_hdr);
        if (secs > 0) {
            *out->out_sleep_seconds = secs;
            ESP_LOGI("hokku", "X-Sleep-Seconds: %d", secs);
        } else {
            ESP_LOGW("hokku", "X-Sleep-Seconds present but non-positive: '%s'", ctx.sleep_seconds_hdr);
        }
    } else {
        ESP_LOGW("hokku", "X-Sleep-Seconds header missing (status=%d)", status);
    }
    if (ctx.server_epoch_hdr[0] != '\0' && out && out->out_server_epoch) {
        int64_t epoch = atoll(ctx.server_epoch_hdr);
        if (epoch > 0) {
            *out->out_server_epoch = epoch;
            /* Set the system clock to server time (RTC-backed — survives deep
             * sleep + esp_restart). Next X-Frame-State reports time(NULL). */
            struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            struct tm t;
            gmtime_r(&tv.tv_sec, &t);
            char timestamp[40];
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S UTC", &t);
            ESP_LOGI("hokku", "X-Server-Time-Epoch: %lld — clock set to %s", epoch, timestamp);
        } else {
            ESP_LOGW("hokku", "X-Server-Time-Epoch non-positive: '%s'", ctx.server_epoch_hdr);
        }
    } else {
        ESP_LOGW("hokku", "X-Server-Time-Epoch header missing (status=%d)", status);
    }

    /* Surface any OTA-update request (server sends it only on the 200). */
    if (out && out->out_fw_update && out->fw_update_buflen > 0) {
        out->out_fw_update[0] = '\0';
        if (ctx.fw_update_hdr[0] != '\0') {
            strncpy(out->out_fw_update, ctx.fw_update_hdr, out->fw_update_buflen - 1);
            out->out_fw_update[out->fw_update_buflen - 1] = '\0';
            ESP_LOGI("hokku", "X-Firmware-Update: %s (server requested OTA)", out->out_fw_update);
        }
    }
    if (out && out->out_content_id && out->content_id_buflen > 0) {
        out->out_content_id[0] = '\0';
        strncpy(out->out_content_id, ctx.content_id_hdr, out->content_id_buflen - 1);
        out->out_content_id[out->content_id_buflen - 1] = '\0';
    }

    esp_http_client_cleanup(client);
    free(log_body);

    if (out && out->out_http_status) *out->out_http_status = status;

    if (err == ESP_OK && status == 204) {
        /* Not a complete image; bool contract unchanged. Logged by the board's
         * 204 branch AFTER its ring reset so the line survives into the next
         * POSTed ring (a pre-reset log is wiped before the server ever sees it —
         * same reason the success line below sits after hokku_log_reset). */
        return false;
    }

    if (err != ESP_OK || status != 200) {
        ESP_LOGE("hokku", "HTTP download failed: err=%s status=%d", esp_err_to_name(err), status);
        return false;
    }

    /* Log upload succeeded with the image: reset the ring so the next cycle
     * starts fresh rather than re-uploading the same content. */
    hokku_log_reset();

    if (ctx.received != expect_bytes) {
        ESP_LOGE("hokku", "Image size mismatch: got %d, expected %d",
                 (int)ctx.received, (int)expect_bytes);
        return false;
    }

    ESP_LOGI("hokku", "Downloaded %d bytes in %lu ms",
             (int)ctx.received, perform_elapsed_ms);
    return true;
}
