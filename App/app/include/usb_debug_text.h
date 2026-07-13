/* Compact, line-oriented USB CDC diagnostics. */
#ifndef APP_USB_DEBUG_TEXT_H_
#define APP_USB_DEBUG_TEXT_H_

#include "err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_USB_TEXT_LINE_MAX 72U

app_err_t app_usb_text_send(const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_USB_DEBUG_TEXT_H_ */
