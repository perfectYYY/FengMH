/*
 * task_usb_cdc_test.c - USB CDC board smoke test task
 *
 * Stage APP_BRINGUP_STAGE_USB_CDC_TEST validates USB CDC enumeration, TX, RX,
 * frame parsing, and checksum before higher-level IMU telemetry depends on it.
 */
#include "task_usb_cdc_test.h"

#include "bsp_time.h"
#include "bsp_usb_cdc.h"
#include "config.h"
#include "log.h"
#include "proto_defs.h"
#include "proto_frame.h"

#include <string.h>

#if APP_TARGET_MCU
#include "cmsis_os.h"
#endif

static const char* TAG = "USBTEST";

#define USB_CDC_TEST_PERIOD_MS       5U
#define USB_CDC_TEST_BOOT_WAIT_MS  800U
#define USB_CDC_TEST_STATE_MS      200U
#define USB_CDC_TEST_LOG_MS       1000U

#pragma pack(push, 1)
typedef struct {
    uint32_t seq;
    uint32_t uptime_ms;
    uint32_t rx_good;
    uint32_t rx_bad;
    uint32_t tx_ok;
    uint32_t tx_busy;
    uint32_t tx_err;
    uint32_t pong_tx;
    uint32_t last_ping_seq;
    uint32_t last_ping_ms;
} payload_usb_cdc_state_t;

typedef struct {
    uint32_t ping_seq;
    uint32_t uptime_ms;
    uint32_t rx_good;
} payload_usb_cdc_pong_t;
#pragma pack(pop)

static proto_frame_parser_t s_parser;
static uint32_t s_seq;
static uint32_t s_tx_ok;
static uint32_t s_tx_busy;
static uint32_t s_tx_err;
static uint32_t s_pong_tx;
static uint32_t s_last_ping_seq;
static uint32_t s_last_ping_ms;
static volatile uint8_t s_pong_pending;
static volatile uint32_t s_pending_ping_seq;

static app_err_t usb_test_send(uint8_t func, const void* payload, uint8_t len) {
    uint8_t frame[80];
    int n = proto_frame_build(func,
                              (const uint8_t*)payload,
                              len,
                              frame,
                              sizeof(frame));
    if (n <= 0) {
        s_tx_err++;
        return APP_ERR_PROTO;
    }

    app_err_t err = bsp_usb_cdc_send(frame, (uint32_t)n);
    if (err == APP_OK) {
        s_tx_ok++;
    } else if (err == APP_ERR_BUSY) {
        s_tx_busy++;
    } else {
        s_tx_err++;
    }
    return err;
}

static void usb_test_send_state(uint32_t now_ms) {
    payload_usb_cdc_state_t payload;
    memset(&payload, 0, sizeof(payload));
    payload.seq = s_seq;
    payload.uptime_ms = now_ms;
    payload.rx_good = s_parser.good_cnt;
    payload.rx_bad = s_parser.bad_cnt;
    payload.tx_ok = s_tx_ok;
    payload.tx_busy = s_tx_busy;
    payload.tx_err = s_tx_err;
    payload.pong_tx = s_pong_tx;
    payload.last_ping_seq = s_last_ping_seq;
    payload.last_ping_ms = s_last_ping_ms;

    (void)usb_test_send(PROTO_FUNC_USB_CDC_STATE,
                        &payload,
                        (uint8_t)sizeof(payload));
}

static void usb_test_note_ping(const uint8_t* payload, uint8_t len) {
    uint32_t ping_seq = 0U;

    if (len >= sizeof(ping_seq)) {
        memcpy(&ping_seq, payload, sizeof(ping_seq));
    }

    s_pending_ping_seq = ping_seq;
    s_pong_pending = 1U;
}

static void usb_test_send_pong(uint32_t ping_seq) {
    payload_usb_cdc_pong_t pong;

    s_last_ping_seq = ping_seq;
    s_last_ping_ms = (uint32_t)bsp_time_now_ms();

    memset(&pong, 0, sizeof(pong));
    pong.ping_seq = ping_seq;
    pong.uptime_ms = s_last_ping_ms;
    pong.rx_good = s_parser.good_cnt;

    if (usb_test_send(PROTO_FUNC_USB_CDC_PONG,
                      &pong,
                      (uint8_t)sizeof(pong)) == APP_OK) {
        s_pong_tx++;
    }
}

static void usb_test_on_frame(const proto_frame_t* frame, void* user) {
    (void)user;
    if (!frame) return;

    if (frame->func_id == PROTO_FUNC_USB_CDC_PING) {
        usb_test_note_ping(frame->payload, frame->len);
    }
}

static void usb_test_on_rx(const uint8_t* data, uint32_t len, void* user) {
    (void)user;
    proto_frame_feed(&s_parser, data, len);
}

void task_usb_cdc_test_init(void) {
    proto_frame_init(&s_parser, usb_test_on_frame, NULL);
    s_seq = 0U;
    s_tx_ok = 0U;
    s_tx_busy = 0U;
    s_tx_err = 0U;
    s_pong_tx = 0U;
    s_last_ping_seq = 0U;
    s_last_ping_ms = 0U;
    s_pong_pending = 0U;
    s_pending_ping_seq = 0U;
    bsp_usb_cdc_attach_rx(usb_test_on_rx, NULL);
    LOGI("usb cdc board test init");
}

void task_usb_cdc_test_entry(void* arg) {
    (void)arg;
    LOGI("usb cdc board test started");

#if APP_TARGET_MCU
    osDelay(USB_CDC_TEST_BOOT_WAIT_MS);

    uint32_t last_state_ms = 0U;
    uint32_t last_log_ms = 0U;

    for (;;) {
        uint32_t now_ms = (uint32_t)bsp_time_now_ms();

        if (s_pong_pending) {
            uint32_t ping_seq = s_pending_ping_seq;
            s_pong_pending = 0U;
            usb_test_send_pong(ping_seq);
        }

        if ((now_ms - last_state_ms) >= USB_CDC_TEST_STATE_MS) {
            s_seq++;
            usb_test_send_state(now_ms);
            last_state_ms = now_ms;
        }

        if ((now_ms - last_log_ms) >= USB_CDC_TEST_LOG_MS) {
            LOGI("seq=%lu rx_good=%lu rx_bad=%lu tx_ok=%lu tx_busy=%lu tx_err=%lu pong=%lu last_ping=%lu",
                 (unsigned long)s_seq,
                 (unsigned long)s_parser.good_cnt,
                 (unsigned long)s_parser.bad_cnt,
                 (unsigned long)s_tx_ok,
                 (unsigned long)s_tx_busy,
                 (unsigned long)s_tx_err,
                 (unsigned long)s_pong_tx,
                 (unsigned long)s_last_ping_seq);
            last_log_ms = now_ms;
        }

        osDelay(USB_CDC_TEST_PERIOD_MS);
    }
#endif
}
