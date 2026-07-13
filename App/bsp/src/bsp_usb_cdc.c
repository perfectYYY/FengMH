/*
 * bsp_usb_cdc.c - host mock and persistent MCU USB CDC transmit queues.
 */
#include "bsp_usb_cdc.h"
#include "config.h"
#include "log.h"

#include <string.h>

#if APP_TARGET_MCU
#include "stm32h7xx_hal.h"
#endif

static const char* TAG = "USBCDC";

#define BSP_USB_TX_MAX 512U
#define BSP_USB_TX_FRAME_MAX 80U
#define BSP_USB_TX_QUEUE_DEPTH 32U
#define BSP_USB_TX_COMMAND_DEPTH 8U
#define BSP_USB_TX_SYSTEM_DEPTH 4U
#define BSP_USB_TX_NORMAL_DEPTH \
    (BSP_USB_TX_QUEUE_DEPTH - BSP_USB_TX_COMMAND_DEPTH - \
     BSP_USB_TX_SYSTEM_DEPTH)

static bsp_usb_rx_cb_t s_rx_cb;
static void* s_rx_user;

static uint8_t tx_frame_priority(const uint8_t* data, uint32_t len) {
    if (len < 5U || data[0] != 0x55U || data[1] != 0xAAU) return 0U;
    if (data[2] == 0x88U) return 2U; /* Competition V2 command status. */
    if (data[2] == 0x89U) return 1U; /* Periodic system status. */
    return 0U;
}

#if APP_TARGET_MCU
/* Provided by usbd_cdc_if.c. */
extern uint8_t CDC_Transmit_HS(uint8_t* buf, uint16_t len);

typedef struct {
    uint8_t data[BSP_USB_TX_FRAME_MAX];
    uint16_t len;
} usb_tx_entry_t;

typedef struct {
    usb_tx_entry_t* entries;
    uint8_t capacity;
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} usb_tx_ring_t;

static usb_tx_entry_t s_command_entries[BSP_USB_TX_COMMAND_DEPTH];
static usb_tx_entry_t s_system_entries[BSP_USB_TX_SYSTEM_DEPTH];
static usb_tx_entry_t s_normal_entries[BSP_USB_TX_NORMAL_DEPTH];
static usb_tx_ring_t s_command_queue = {
    .entries = s_command_entries,
    .capacity = BSP_USB_TX_COMMAND_DEPTH,
};
static usb_tx_ring_t s_system_queue = {
    .entries = s_system_entries,
    .capacity = BSP_USB_TX_SYSTEM_DEPTH,
};
static usb_tx_ring_t s_normal_queue = {
    .entries = s_normal_entries,
    .capacity = BSP_USB_TX_NORMAL_DEPTH,
};
static uint8_t s_tx_in_flight;
static uint8_t s_tx_in_flight_class;

static uint32_t tx_critical_enter(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void tx_critical_exit(uint32_t primask) {
    if (primask == 0U) __enable_irq();
}

static void tx_ring_reset(usb_tx_ring_t* queue) {
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
}

static void tx_reset_locked(void) {
    tx_ring_reset(&s_command_queue);
    tx_ring_reset(&s_system_queue);
    tx_ring_reset(&s_normal_queue);
    s_tx_in_flight = 0U;
    s_tx_in_flight_class = 0U;
}

static void tx_kick_locked(void) {
    if (s_tx_in_flight) return;

    usb_tx_ring_t* queue = NULL;
    uint8_t priority_class = 0U;
    if (s_command_queue.count > 0U) {
        queue = &s_command_queue;
        priority_class = 2U;
    } else if (s_system_queue.count > 0U) {
        queue = &s_system_queue;
        priority_class = 1U;
    } else if (s_normal_queue.count > 0U) {
        queue = &s_normal_queue;
    }
    if (!queue) return;

    usb_tx_entry_t* entry = &queue->entries[queue->head];
    if (CDC_Transmit_HS(entry->data, entry->len) == 0U) {
        s_tx_in_flight = 1U;
        s_tx_in_flight_class = priority_class;
    }
}

static app_err_t tx_enqueue_locked(usb_tx_ring_t* queue,
                                   const uint8_t* data,
                                   uint32_t len) {
    if (queue->count >= queue->capacity) return APP_ERR_BUSY;
    usb_tx_entry_t* entry = &queue->entries[queue->tail];
    memcpy(entry->data, data, len);
    entry->len = (uint16_t)len;
    queue->tail = (uint8_t)((queue->tail + 1U) % queue->capacity);
    queue->count++;
    return APP_OK;
}

static void tx_complete_locked(usb_tx_ring_t* queue) {
    if (queue->count == 0U) return;
    queue->head = (uint8_t)((queue->head + 1U) % queue->capacity);
    queue->count--;
}
#endif

#if APP_TARGET_HOST
static uint8_t s_tx_buf[BSP_USB_TX_MAX];
static uint32_t s_tx_len;
#endif

app_err_t bsp_usb_cdc_init(void) {
    s_rx_cb = NULL;
    s_rx_user = NULL;
#if APP_TARGET_HOST
    s_tx_len = 0U;
#else
    uint32_t critical = tx_critical_enter();
    tx_reset_locked();
    memset(s_command_entries, 0, sizeof(s_command_entries));
    memset(s_system_entries, 0, sizeof(s_system_entries));
    memset(s_normal_entries, 0, sizeof(s_normal_entries));
    tx_critical_exit(critical);
#endif
    LOGI("init ok (host=%d)", (int)APP_TARGET_HOST);
    return APP_OK;
}

app_err_t bsp_usb_cdc_attach_rx(bsp_usb_rx_cb_t cb, void* user) {
    s_rx_cb = cb;
    s_rx_user = user;
    return APP_OK;
}

app_err_t bsp_usb_cdc_send(const uint8_t* data, uint32_t len) {
    if (!data || len == 0U) return APP_ERR_INVALID_ARG;
#if APP_TARGET_HOST
    if (s_tx_len + len > BSP_USB_TX_MAX) return APP_ERR_OVERFLOW;
    memcpy(s_tx_buf + s_tx_len, data, len);
    s_tx_len += len;
    return APP_OK;
#else
    if (len > BSP_USB_TX_FRAME_MAX) return APP_ERR_OVERFLOW;
    uint32_t critical = tx_critical_enter();
    const uint8_t priority_class = tx_frame_priority(data, len);
    usb_tx_ring_t* queue = priority_class == 2U ? &s_command_queue :
                           priority_class == 1U ? &s_system_queue :
                                                  &s_normal_queue;
    app_err_t result = tx_enqueue_locked(queue, data, len);
    if (result == APP_OK) tx_kick_locked();
    tx_critical_exit(critical);
    return result;
#endif
}

void bsp_usb_cdc_process(void) {
#if APP_TARGET_MCU
    uint32_t critical = tx_critical_enter();
    tx_kick_locked();
    tx_critical_exit(critical);
#endif
}

void bsp_usb_cdc_on_tx_complete(void) {
#if APP_TARGET_MCU
    uint32_t critical = tx_critical_enter();
    if (s_tx_in_flight) {
        usb_tx_ring_t* queue = s_tx_in_flight_class == 2U ?
                               &s_command_queue :
                               s_tx_in_flight_class == 1U ?
                               &s_system_queue : &s_normal_queue;
        tx_complete_locked(queue);
    }
    s_tx_in_flight = 0U;
    s_tx_in_flight_class = 0U;
    tx_kick_locked();
    tx_critical_exit(critical);
#endif
}

void bsp_usb_cdc_on_disconnect(void) {
#if APP_TARGET_MCU
    uint32_t critical = tx_critical_enter();
    tx_reset_locked();
    tx_critical_exit(critical);
#endif
}

void bsp_usb_cdc_on_rx(const uint8_t* data, uint32_t len) {
    if (s_rx_cb && data && len) s_rx_cb(data, len, s_rx_user);
}

uint8_t bsp_usb_cdc_test_priority_class(const uint8_t* data, uint32_t len) {
    if (!data) return 0U;
    return tx_frame_priority(data, len);
}

#if APP_TARGET_HOST
void bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len) {
    bsp_usb_cdc_on_rx(data, len);
}

uint32_t bsp_usb_cdc_test_tx_size(void) { return s_tx_len; }

uint32_t bsp_usb_cdc_test_read_tx(uint8_t* out, uint32_t max_len) {
    if (!out) return 0U;
    uint32_t n = (s_tx_len < max_len) ? s_tx_len : max_len;
    memcpy(out, s_tx_buf, n);
    return n;
}

void bsp_usb_cdc_test_reset(void) { s_tx_len = 0U; }
#else
void bsp_usb_cdc_test_inject_rx(const uint8_t* data, uint32_t len) {
    (void)data;
    (void)len;
}
uint32_t bsp_usb_cdc_test_tx_size(void) { return 0U; }
uint32_t bsp_usb_cdc_test_read_tx(uint8_t* out, uint32_t max_len) {
    (void)out;
    (void)max_len;
    return 0U;
}
void bsp_usb_cdc_test_reset(void) {}
#endif
