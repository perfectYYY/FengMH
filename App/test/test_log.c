/*
 * test_log.c — log 模块行为测试
 */
#include "test_util.h"
#include "log.h"

#include <string.h>

static int s_be_calls;
static log_lvl_t s_be_last_lvl;
static char      s_be_last_tag[32];
static char      s_be_last_msg[256];

static void be(log_lvl_t lvl, const char* tag, const char* msg) {
    s_be_calls++;
    s_be_last_lvl = lvl;
    strncpy(s_be_last_tag, tag ? tag : "", sizeof(s_be_last_tag) - 1);
    strncpy(s_be_last_msg, msg ? msg : "", sizeof(s_be_last_msg) - 1);
}

static const char* TAG = "TEST";

static void t_global_level_filter(void) {
    log_init();
    log_register_backend(be);
    log_set_global_level(LOG_LVL_WARN);

    s_be_calls = 0;
    LOGI("info_should_drop");
    TEST_ASSERT_EQUAL_INT(0, s_be_calls);

    LOGW("warn_should_pass");
    TEST_ASSERT_EQUAL_INT(1, s_be_calls);
    TEST_ASSERT_EQUAL_INT(LOG_LVL_WARN, s_be_last_lvl);
    TEST_ASSERT(strstr(s_be_last_msg, "warn_should_pass") != NULL);
}

static void t_per_tag_level(void) {
    log_init();
    log_register_backend(be);
    log_set_global_level(LOG_LVL_ERR);

    log_set_level("TEST", LOG_LVL_DBG);

    s_be_calls = 0;
    LOGD("dbg_pass_via_tag");
    TEST_ASSERT_EQUAL_INT(1, s_be_calls);
    TEST_ASSERT(strstr(log_last_line(), "dbg_pass_via_tag") != NULL);
}

static void t_format(void) {
    log_init();
    log_register_backend(be);
    log_set_global_level(LOG_LVL_TRACE);
    LOGI("a=%d b=%s", 42, "hello");
    TEST_ASSERT(strstr(s_be_last_msg, "a=42 b=hello") != NULL);
}

int main(void) {
    TU_RUN(t_global_level_filter);
    TU_RUN(t_per_tag_level);
    TU_RUN(t_format);
    TU_MAIN_EPILOGUE();
}
