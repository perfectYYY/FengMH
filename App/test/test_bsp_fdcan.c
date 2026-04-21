/*
 * test_bsp_fdcan.c — host mock 行为测试
 */
#include "test_util.h"
#include "bsp_fdcan.h"
#include "log.h"

#include <string.h>

static int  s_rx_calls;
static bsp_fdcan_frame_t s_last_rx;
static bsp_fdcan_bus_t   s_last_bus;

static void on_rx(bsp_fdcan_bus_t bus, const bsp_fdcan_frame_t* f, void* user) {
    (void)user;
    s_rx_calls++;
    s_last_bus = bus;
    s_last_rx = *f;
}

static void t_send_then_pop(void) {
    bsp_fdcan_init();
    bsp_fdcan_test_reset();

    bsp_fdcan_frame_t f = { .can_id = 0x200, .dlc = 8 };
    for (int i = 0; i < 8; i++) f.data[i] = (uint8_t)i;

    TEST_ASSERT_EQUAL_INT(APP_OK, bsp_fdcan_send(BSP_FDCAN_1, &f));
    TEST_ASSERT_EQUAL_UINT(1, bsp_fdcan_test_tx_count(BSP_FDCAN_1));

    bsp_fdcan_frame_t out;
    TEST_ASSERT_EQUAL_INT(APP_OK, bsp_fdcan_test_pop_tx(BSP_FDCAN_1, &out));
    TEST_ASSERT_EQUAL_UINT(0x200, out.can_id);
    TEST_ASSERT_EQUAL_INT(8, out.dlc);
    TEST_ASSERT_EQUAL_INT(7, out.data[7]);
    TEST_ASSERT_EQUAL_UINT(0, bsp_fdcan_test_tx_count(BSP_FDCAN_1));
}

static void t_inject_rx(void) {
    bsp_fdcan_init();
    bsp_fdcan_attach_rx(BSP_FDCAN_2, on_rx, NULL);

    bsp_fdcan_frame_t f = { .can_id = 0x203, .dlc = 4, .data = {0xAA,0xBB,0xCC,0xDD} };
    s_rx_calls = 0;
    bsp_fdcan_test_inject_rx(BSP_FDCAN_2, &f);
    TEST_ASSERT_EQUAL_INT(1, s_rx_calls);
    TEST_ASSERT_EQUAL_INT(BSP_FDCAN_2, s_last_bus);
    TEST_ASSERT_EQUAL_HEX8(0xAA, s_last_rx.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xDD, s_last_rx.data[3]);
}

static void t_invalid_args(void) {
    bsp_fdcan_init();
    bsp_fdcan_frame_t f = { .can_id = 0, .dlc = 9 };
    TEST_ASSERT_EQUAL_INT(APP_ERR_INVALID_ARG, bsp_fdcan_send(BSP_FDCAN_1, &f));

    f.dlc = 8;
    TEST_ASSERT_EQUAL_INT(APP_ERR_INVALID_ARG, bsp_fdcan_send((bsp_fdcan_bus_t)9, &f));
}

int main(void) {
    log_init();
    log_set_global_level(LOG_LVL_ERR);
    TU_RUN(t_send_then_pop);
    TU_RUN(t_inject_rx);
    TU_RUN(t_invalid_args);
    TU_MAIN_EPILOGUE();
}
