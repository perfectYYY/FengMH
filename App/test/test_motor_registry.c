/*
 * test_motor_registry.c
 */
#include "test_util.h"
#include "motor_registry.h"

static motor_dev_t s_stub;

static void t_init_clears_devs(void) {
    motor_registry_init();
    TEST_ASSERT_NULL(motor_get(MOTOR_ID_FL_HIP));
    TEST_ASSERT_NULL(motor_get(MOTOR_ID_RR_WHEEL));
    TEST_ASSERT_EQUAL_UINT((unsigned)MOTOR_ID_MAX, motor_registry_count());
}

static void t_cfg_lookup(void) {
    const motor_cfg_t* c = motor_get_cfg(MOTOR_ID_FL_WHEEL);
    TEST_ASSERT_NOT_NULL(c);
    TEST_ASSERT_EQUAL_INT(MOTOR_M3508, c->type);
    TEST_ASSERT_EQUAL_UINT(1, c->can_bus);
    TEST_ASSERT_EQUAL_UINT(0x201, c->can_id);

    const motor_cfg_t* arm = motor_get_cfg(MOTOR_ID_ARM_J1);
    TEST_ASSERT_EQUAL_INT(MOTOR_DAMIAO, arm->type);
}

static void t_bind_and_get(void) {
    motor_registry_init();
    s_stub.ops = NULL; s_stub.drv_ctx = NULL;
    TEST_ASSERT_EQUAL_INT(APP_OK, motor_registry_bind(MOTOR_ID_FR_KNEE, &s_stub));
    motor_dev_t* d = motor_get(MOTOR_ID_FR_KNEE);
    TEST_ASSERT(d == &s_stub);
    TEST_ASSERT_EQUAL_INT(MOTOR_GO, d->state.type);
    TEST_ASSERT_EQUAL_UINT(MOTOR_ID_FR_KNEE, d->state.id);
}

static void t_oob_returns_null(void) {
    TEST_ASSERT_NULL(motor_get((motor_logical_id_t)9999));
    TEST_ASSERT_NULL(motor_get_cfg((motor_logical_id_t)9999));
}

int main(void) {
    TU_RUN(t_init_clears_devs);
    TU_RUN(t_cfg_lookup);
    TU_RUN(t_bind_and_get);
    TU_RUN(t_oob_returns_null);
    TU_MAIN_EPILOGUE();
}
