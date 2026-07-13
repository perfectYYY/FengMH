/* USB CDC text output shared by startup and RTOS diagnostics. */
#include "usb_debug_text.h"
#include "bsp_usb_cdc.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

app_err_t app_usb_text_send(const char* format, ...) {
    char line[APP_USB_TEXT_LINE_MAX];
    va_list args;
    int length;

    if (!format) return APP_ERR_INVALID_ARG;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length <= 0 || (uint32_t)length >= sizeof(line)) return APP_ERR_OVERFLOW;
    return bsp_usb_cdc_send((const uint8_t*)line, (uint32_t)length);
}
