/*
 * test_logic.c — host-side unit tests for the shared ESP-IDF modules in
 * firmware/common/esp32/ (config, state, scheduler, log), plus a compile+link
 * check of the whole common/esp32 layer (wifi, net, ota) against the shared
 * mock kit. This tests the shared code in ISOLATION — no firmware board layer.
 *
 * Same technique as the firmware suites: include the mock headers before
 * redefining `static`, then include the shared .c files so their static
 * functions/globals become regular symbols in this translation unit.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <time.h>

#include "mocks/freertos/FreeRTOS.h"
#include "mocks/freertos/task.h"
#include "mocks/freertos/event_groups.h"
#include "mocks/driver/gpio.h"
#include "mocks/driver/spi_master.h"
#include "mocks/driver/rtc_io.h"
#include "mocks/esp_adc/adc_oneshot.h"
#include "mocks/esp_adc/adc_cali.h"
#include "mocks/esp_adc/adc_cali_scheme.h"
#include "mocks/esp_log.h"
#include "mocks/esp_sleep.h"
#include "mocks/esp_wifi.h"
#include "mocks/esp_event.h"
#include "mocks/esp_netif.h"
#include "mocks/esp_http_client.h"
#include "mocks/esp_heap_caps.h"
#include "mocks/nvs_flash.h"
#include "mocks/esp_timer.h"
#include "mocks/esp_app_desc.h"
#include "mocks/esp_ota_ops.h"
#include "mocks/esp_partition.h"

#define static

/* common/all deps first (pure), then the common/esp32 modules. Including
 * wifi/net/ota proves the whole shared ESP32 layer compiles + links against the
 * mocks, even though the logic tests below focus on config/state/scheduler/log. */
#include "../../all/logbuf.c"
#include "../../all/json_util.c"
#include "../../all/firmware_url.c"
#include "../../all/frame_state.c"
#include "config.c"
#include "state.c"
#include "scheduler.c"
#include "log.c"
#include "wifi.c"
#include "net.c"
#include "ota.c"

/* ── Minimal test framework ── */
static int g_pass = 0;
static int g_fail = 0;
#define CHECK(cond, name) do {                              \
    if (cond) { printf("PASS  %s\n", name); g_pass++; }     \
    else      { printf("FAIL  %s\n", name); g_fail++; }     \
} while (0)

/* ═══ scheduler.c ═══ */
static void test_now_epoch_post_2020(void)
{
    CHECK(now_epoch() > 1577836800LL, "scheduler: now_epoch returns a post-2020 timestamp");
}
static void test_refresh_due(void)
{
    next_refresh_epoch = 0;
    CHECK(refresh_due(), "scheduler: refresh_due true when unscheduled (0)");
    next_refresh_epoch = 1;
    CHECK(refresh_due(), "scheduler: refresh_due true when epoch in the past");
    next_refresh_epoch = 9999999999LL;   /* year 2286 */
    CHECK(!refresh_due(), "scheduler: refresh_due false when epoch far in the future");
}
static void test_schedule_retry_in(void)
{
    next_refresh_epoch = 0;
    pre_sleep_server_epoch = 123;
    last_sleep_err_known = true;
    time_t before = time(NULL);
    schedule_retry_in(60, "test");
    time_t after = time(NULL);
    CHECK(next_refresh_epoch >= (int64_t)before + 60 && next_refresh_epoch <= (int64_t)after + 60,
          "scheduler: schedule_retry_in sets next_refresh_epoch to now + seconds");
    CHECK(pre_sleep_server_epoch == 0 && !last_sleep_err_known,
          "scheduler: schedule_retry_in clears the sleep-error snapshot");
}
static void test_save_pre_sleep_epoch(void)
{
    save_pre_sleep_epoch(0, 0);
    CHECK(pre_sleep_server_epoch == 0 && !last_sleep_err_known,
          "scheduler: save_pre_sleep_epoch(0) clears the snapshot");
    save_pre_sleep_epoch(1700000000LL, esp_timer_get_time());
    CHECK(pre_sleep_server_epoch >= 1700000000LL,
          "scheduler: save_pre_sleep_epoch stores a server-anchored epoch");
}

/* ═══ log.c (single crash-safe RTC ring) ═══ */
static int call_log(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int n = log_vprintf(fmt, ap);
    va_end(ap);
    return n;
}
static void test_log_ring_lifecycle(void)
{
    s_log_ring_head = 0; s_log_ring_used = 0;
    hokku_log_init();
    call_log("aa"); call_log("bb");
    CHECK(s_log_ring_used > 0, "log: append persists ring position to RTC each line");
    char body[HOKKU_LOG_MAX_UPLOAD];
    size_t n = hokku_log_snapshot(body, sizeof(body)); body[n] = '\0';
    CHECK(strcmp(body, "aabb") == 0, "log: snapshot returns the ring contents");

    /* Reconstruct from RTC (simulated reboot) — pre-reboot logs must survive. */
    hokku_log_init();
    call_log("cc");
    n = hokku_log_snapshot(body, sizeof(body)); body[n] = '\0';
    CHECK(strcmp(body, "aabbcc") == 0, "log: RTC ring survives a reboot (crash-safe)");

    hokku_log_reset();
    n = hokku_log_snapshot(body, sizeof(body));
    CHECK(n == 0 && s_log_ring_used == 0, "log: reset clears the ring");
}

/* ═══ config.c ═══ */
static void test_config_valid(void)
{
    memset(&config, 0, sizeof(config));
    config.cfg_ver = CONFIG_VERSION;
    CHECK(!config_is_valid(), "config: invalid with no SSID / image_url");
    strcpy(config.wifi_ssid[0], "net");
    strcpy(config.image_url, "http://h/hokku/screen/");
    CHECK(config_is_valid(), "config: valid with primary SSID + image_url + cfg_ver");
    config.cfg_ver = CONFIG_VERSION + 1;
    CHECK(!config_is_valid(), "config: invalid on cfg_ver mismatch");
}

/* ═══ Behavioral tests (R7): wifi.c two-phase wait + ladder, net.c 204 +
 * transfer deadline. These drive the programmable mocks (event-group return
 * queue, controllable clock, AP-info result, scripted HTTP perform). ═══ */

static void reset_mocks(void)
{
    mock_eg_reset();
    mock_http_reset();
    _mock_timer_us = 0;
    _mock_ap_info_result = -1;
    memset(&_mock_ap_record, 0, sizeof(_mock_ap_record));

    has_wifi_cache = false;
    wifi_channel = 0;
    last_wifi_index = 0;
    memset(wifi_bssid, 0, sizeof(wifi_bssid));
    last_wifi_used_cache = false;

    memset(&config, 0, sizeof(config));
    config.cfg_ver = CONFIG_VERSION;
    config.wifi_order = WIFI_ORDER_PRIMARY_FIRST;
    strcpy(config.wifi_ssid[0], "net0");          /* one primary network */
    strcpy(config.image_url, "http://h/hokku/screen/");
}

/* ── wifi.c: two-phase association/DHCP wait ── */
static void test_wifi_two_phase_wait(void)
{
    const int64_t dl = 75000000;  /* 75 s ahead of the mock clock (starts at 0) */

    reset_mocks(); mock_eg_push(WIFI_CONNECTED_BIT);
    CHECK(wifi_wait_for_ip(dl), "wifi: phase-1 GOT_IP -> connected");

    reset_mocks(); mock_eg_push(WIFI_FAIL_BIT);
    CHECK(!wifi_wait_for_ip(dl), "wifi: phase-1 disconnect -> fail");

    reset_mocks(); mock_eg_push(0);  /* association timeout: no L2, no IP, no fail */
    CHECK(!wifi_wait_for_ip(dl), "wifi: no association within budget -> fail");

    reset_mocks(); mock_eg_push(WIFI_L2_BIT); mock_eg_push(WIFI_CONNECTED_BIT);
    CHECK(wifi_wait_for_ip(dl), "wifi: L2 then DHCP GOT_IP -> connected");

    reset_mocks(); mock_eg_push(WIFI_L2_BIT); mock_eg_push(0);  /* DHCP times out */
    CHECK(!wifi_wait_for_ip(dl), "wifi: L2 up then DHCP timeout -> fail (assoc not torn down early)");

    reset_mocks(); mock_eg_push(WIFI_L2_BIT); mock_eg_push(WIFI_FAIL_BIT);
    CHECK(!wifi_wait_for_ip(dl), "wifi: L2 up then disconnect during DHCP -> fail");
}

static void test_wifi_deadline_epsilon_guard(void)
{
    reset_mocks();
    mock_eg_push(WIFI_CONNECTED_BIT);        /* would succeed IF a wait happened */
    CHECK(!wifi_wait_for_ip(_mock_timer_us), /* deadline == now: remaining <= epsilon */
          "wifi: past-deadline epsilon guard fails without waiting");
    CHECK(_mock_eg_wait_calls == 0, "wifi: epsilon guard skips the association wait entirely");
}

/* ── wifi.c: full connect ladder ── */
static void test_wifi_cache_fallback(void)
{
    reset_mocks();
    has_wifi_cache = true; wifi_channel = 6; last_wifi_index = 0;
    mock_wifi_set_ap_info(0, 11);            /* AP live on channel 11 */
    mock_eg_push(WIFI_FAIL_BIT);             /* cached-BSSID attempt fails */
    mock_eg_push(0);                         /* wifi_disconnect_settle's FAIL-wait */
    mock_eg_push(WIFI_CONNECTED_BIT);        /* full-scan attempt gets IP */

    CHECK(wifi_connect(), "wifi: cached-BSSID miss falls back to full scan and connects");
    CHECK(has_wifi_cache && wifi_channel == 11, "wifi: full-scan success re-caches the AP");
    CHECK(!last_wifi_used_cache, "wifi: fallback success reports cache-not-used");
}

static void test_wifi_assoc_vanish_clears_cache(void)
{
    reset_mocks();
    has_wifi_cache = true; wifi_channel = 6; last_wifi_index = 0;
    mock_wifi_set_ap_info(-1, 0);            /* AP-info query fails after GOT_IP */
    mock_eg_push(WIFI_CONNECTED_BIT);        /* cached attempt appears to get IP... */

    CHECK(!wifi_connect(), "wifi: AP vanished after GOT_IP -> connect fails");
    CHECK(!has_wifi_cache, "wifi: AP-info failure clears the BSSID cache (forces a scan next time)");
}

static void test_wifi_ladder_deadline_exhaustion(void)
{
    reset_mocks();
    mock_eg_set_advance_us(40000000);        /* each wait burns 40 s of the 75 s budget */
    /* queue empty -> every wait times out; the ladder must stop on the deadline,
     * not run all four rounds. */
    CHECK(!wifi_connect(), "wifi: dead network fails the connect");
    CHECK(_mock_eg_wait_calls <= 3,
          "wifi: 75 s deadline bounds the ladder (far fewer than four full rounds)");
}

/* ── net.c: response-header capture + transfer deadline (handler-direct) ── */
static void test_net_header_capture(void)
{
    reset_mocks();
    http_download_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));                 /* transfer_start_us = 0, clock = 0 */
    esp_http_client_event_t evt = { .client = (void *)1, .user_data = &ctx,
                                     .event_id = HTTP_EVENT_ON_HEADER };

    evt.header_key = (char *)"X-Content-Id"; evt.header_value = (char *)"cafe1234abcd";
    http_event_handler(&evt);
    CHECK(strcmp(ctx.content_id_hdr, "cafe1234abcd") == 0, "net: captures X-Content-Id header");

    evt.header_key = (char *)"X-Sleep-Seconds"; evt.header_value = (char *)"120";
    http_event_handler(&evt);
    CHECK(strcmp(ctx.sleep_seconds_hdr, "120") == 0, "net: captures X-Sleep-Seconds header");

    evt.header_key = (char *)"X-Firmware-Update"; evt.header_value = (char *)"1.2.99";
    http_event_handler(&evt);
    CHECK(strcmp(ctx.fw_update_hdr, "1.2.99") == 0, "net: captures X-Firmware-Update header");
    CHECK(_mock_http.close_calls == 0, "net: no deadline close while inside the budget");
}

static void test_net_transfer_deadline(void)
{
    reset_mocks();
    http_download_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));                 /* transfer_start_us = 0 */
    _mock_timer_us = 91000000;                    /* 91 s > 90 s image deadline */
    esp_http_client_event_t evt = { .client = (void *)1, .user_data = &ctx,
                                     .event_id = HTTP_EVENT_ON_HEADER };  /* NULL key: no capture */

    http_event_handler(&evt);
    CHECK(ctx.deadline_hit, "net: an event past the 90 s deadline sets deadline_hit");
    CHECK(_mock_http.close_calls == 1, "net: overdue event closes the client");
    http_event_handler(&evt);
    CHECK(_mock_http.close_calls == 2, "net: closes on every overdue event (re-bounds retries)");
    CHECK(ctx.deadline_hit, "net: deadline flag latches across overdue events");
}

/* ── net.c: hokku_http_fetch_image status contract ── */
static void test_net_204_skip(void)
{
    reset_mocks();
    s_log_ring_head = 0; s_log_ring_used = 0;
    hokku_log_init();
    call_log("ride-along");                       /* rode the request as the POST body */
    size_t used_before = s_log_ring_used;

    mock_http_push_event(HTTP_EVENT_ON_CONNECTED, NULL, NULL, NULL, 0);
    mock_http_push_header((char *)"X-Sleep-Seconds", (char *)"300");
    mock_http_push_header((char *)"X-Content-Id", (char *)"aabbccdd1122");
    mock_http_set_result(ESP_OK, 204);

    uint8_t buf[8];
    int32_t sleep_s = 0; int64_t epoch = 0; int status = 0; char cid[16] = {0};
    hokku_fetch_out_t out = { .out_sleep_seconds = &sleep_s, .out_server_epoch = &epoch,
                              .out_http_status = &status, .out_content_id = cid,
                              .content_id_buflen = sizeof(cid) };

    bool r = hokku_http_fetch_image(buf, sizeof(buf), "http://h/", "name", "MODEL",
                                    "{}", "build", NULL, &out);
    CHECK(!r, "net: 204 returns false (a no-op never satisfies pending-OTA verification)");
    CHECK(status == 204, "net: 204 surfaces out_http_status = 204");
    CHECK(sleep_s == 300, "net: 204 still applies X-Sleep-Seconds (server owns cadence)");
    CHECK(strcmp(cid, "aabbccdd1122") == 0, "net: 204 captures X-Content-Id");
    CHECK(s_log_ring_used == used_before, "net: 204 does NOT reset the log ring (Huessen's branch owns that)");
}

static void test_net_200_paint(void)
{
    reset_mocks();
    s_log_ring_head = 0; s_log_ring_used = 0;
    hokku_log_init();
    call_log("pre-200");

    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t buf[8] = {0};
    mock_http_push_event(HTTP_EVENT_ON_CONNECTED, NULL, NULL, NULL, 0);
    mock_http_push_header((char *)"X-Content-Id", (char *)"deadbeef0001");
    mock_http_push_data(payload, sizeof(payload));
    mock_http_set_result(ESP_OK, 200);

    int status = 0; char cid[16] = {0};
    hokku_fetch_out_t out = { .out_http_status = &status, .out_content_id = cid,
                              .content_id_buflen = sizeof(cid) };

    bool r = hokku_http_fetch_image(buf, sizeof(buf), "http://h/", "n", "M", "{}", "b", NULL, &out);
    CHECK(r, "net: 200 with exactly expect_bytes returns true");
    CHECK(status == 200, "net: 200 surfaces out_http_status = 200");
    CHECK(memcmp(buf, payload, sizeof(payload)) == 0, "net: 200 delivers the image bytes into buf");
    CHECK(strcmp(cid, "deadbeef0001") == 0, "net: 200 captures X-Content-Id");
    CHECK(s_log_ring_used == 0, "net: 200 resets the log ring");
}

/* perform() completes a full 200 body but the transfer ran past the 90 s
 * deadline: the post-perform deadline_hit fold must force failure and skip the
 * ring reset, discarding the completed-but-overdue image (R6 reviewer). */
static void test_net_completed_but_overdue_discarded(void)
{
    reset_mocks();
    s_log_ring_head = 0; s_log_ring_used = 0;
    hokku_log_init();
    call_log("overdue");
    size_t used_before = s_log_ring_used;

    uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t buf[8] = {0};
    mock_http_set_advance_us(100000000);   /* +100 s per event -> trips the 90 s deadline mid-replay */
    mock_http_push_event(HTTP_EVENT_ON_CONNECTED, NULL, NULL, NULL, 0);
    mock_http_push_data(payload, sizeof(payload));   /* full body still delivered */
    mock_http_set_result(ESP_OK, 200);     /* server returned a clean 200 */

    int status = 0;
    hokku_fetch_out_t out = { .out_http_status = &status };
    bool r = hokku_http_fetch_image(buf, sizeof(buf), "http://h/", "n", "M", "{}", "b", NULL, &out);
    CHECK(!r, "net: completed-but-overdue 200 is discarded (deadline_hit folds to failure)");
    CHECK(status == 200, "net: overdue transfer still surfaces the server's 200");
    CHECK(s_log_ring_used == used_before, "net: overdue transfer does NOT reset the log ring");
}

/* wifi_disconnect_settle clamps its FAIL-wait and delay to the shared deadline;
 * past the deadline it must not wait at all (R2 reviewer: settle-clamp race). */
static void test_wifi_settle_respects_deadline(void)
{
    reset_mocks();
    _mock_timer_us = 100000;                    /* now = 0.1 s */
    wifi_disconnect_settle(50000);              /* deadline already 0.05 s in the past */
    CHECK(_mock_eg_wait_calls == 0, "wifi: settle past the deadline skips its clamped FAIL-wait");

    reset_mocks();
    wifi_disconnect_settle(75000000);           /* deadline 75 s ahead */
    CHECK(_mock_eg_wait_calls == 1, "wifi: settle within budget performs its one FAIL-wait");
}

int main(void)
{
    printf("=== test_logic (common/esp32) ===\n\n");
    test_now_epoch_post_2020();
    test_refresh_due();
    test_schedule_retry_in();
    test_save_pre_sleep_epoch();
    test_log_ring_lifecycle();
    test_config_valid();
    test_wifi_two_phase_wait();
    test_wifi_deadline_epsilon_guard();
    test_wifi_cache_fallback();
    test_wifi_assoc_vanish_clears_cache();
    test_wifi_ladder_deadline_exhaustion();
    test_net_header_capture();
    test_net_transfer_deadline();
    test_net_204_skip();
    test_net_200_paint();
    test_net_completed_but_overdue_discarded();
    test_wifi_settle_respects_deadline();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
