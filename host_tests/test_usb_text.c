#include "bsp_usb_cdc.h"
#include "err.h"
#include "usb_debug_text.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int main(void) {
    uint8_t out[APP_USB_TEXT_LINE_MAX];
    uint32_t size;

    assert(bsp_usb_cdc_init() == APP_OK);
    assert(app_usb_text_send("BMI,n=%u,s=%u,e=%d\r\n", 1U, 1U, -14) == APP_OK);
    size = bsp_usb_cdc_test_read_tx(out, sizeof(out));
    assert(size > 0U && size < APP_USB_TEXT_LINE_MAX);
    for (uint32_t i = 0U; i < size; i++) {
        assert(out[i] == '\r' || out[i] == '\n' || (out[i] >= 0x20U && out[i] <= 0x7EU));
    }

    bsp_usb_cdc_test_reset();
    assert(app_usb_text_send("012345678901234567890123456789012345678901234567890123456789012345678901234")
           == APP_ERR_OVERFLOW);
    assert(bsp_usb_cdc_test_tx_size() == 0U);
    return 0;
}
