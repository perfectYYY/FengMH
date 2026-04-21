/*
 * err.h — 统一错误码
 */
#ifndef APP_COMMON_ERR_H_
#define APP_COMMON_ERR_H_

typedef enum {
    APP_OK              = 0,
    APP_ERR_GENERIC     = -1,
    APP_ERR_NULL_PTR    = -2,
    APP_ERR_INVALID_ARG = -3,
    APP_ERR_TIMEOUT     = -4,
    APP_ERR_NOT_FOUND   = -5,
    APP_ERR_BUSY        = -6,
    APP_ERR_NO_MEM      = -7,
    APP_ERR_CHECKSUM    = -8,
    APP_ERR_PROTO       = -9,
    APP_ERR_OFFLINE     = -10,
    APP_ERR_OVERFLOW    = -11,
    APP_ERR_UNSUPPORTED = -12,
    APP_ERR_UNINIT      = -13
} app_err_t;

#endif /* APP_COMMON_ERR_H_ */
