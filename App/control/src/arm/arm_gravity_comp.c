/*
 * arm_gravity_comp.c - Arm gravity compensation model.
 */
#include "arm_gravity_comp.h"

#include <math.h>

volatile float g_arm_gc_mass_l2_kg = ARM_GRAVITY_MASS_L2;
volatile float g_arm_gc_mass_l3_kg = ARM_GRAVITY_MASS_L3;
volatile float g_arm_gc_mass_ee_kg = ARM_GRAVITY_MASS_EE_EMPTY;
volatile float g_arm_gc_mass_cargo_kg = ARM_GRAVITY_MASS_CARGO_BOX;
volatile float g_arm_gc_com_r2_x_m = ARM_GRAVITY_COM_R2_X_M;
volatile float g_arm_gc_com_r2_y_m = ARM_GRAVITY_COM_R2_Y_M;
volatile float g_arm_gc_com_r3_x_m = ARM_GRAVITY_COM_R3_X_M;
volatile float g_arm_gc_com_r3_y_m = ARM_GRAVITY_COM_R3_Y_M;
volatile float g_arm_gc_com_ee_x_m = ARM_GRAVITY_COM_EE_X_M;

static arm_gravity_payload_state_t s_payload_state = ARM_GRAVITY_PAYLOAD_EMPTY;
static float s_active_ee_mass_kg = ARM_GRAVITY_MASS_EE_EMPTY;
static float s_target_ee_mass_kg = ARM_GRAVITY_MASS_EE_EMPTY;

static void update_mass_transition(void) {
    s_target_ee_mass_kg =
        (s_payload_state == ARM_GRAVITY_PAYLOAD_LOADED) ?
        (g_arm_gc_mass_ee_kg + g_arm_gc_mass_cargo_kg) :
        g_arm_gc_mass_ee_kg;

    const float diff = s_target_ee_mass_kg - s_active_ee_mass_kg;
    if (diff > ARM_GRAVITY_MASS_TRANSITION_STEP_KG) {
        s_active_ee_mass_kg += ARM_GRAVITY_MASS_TRANSITION_STEP_KG;
    } else if (diff < -ARM_GRAVITY_MASS_TRANSITION_STEP_KG) {
        s_active_ee_mass_kg -= ARM_GRAVITY_MASS_TRANSITION_STEP_KG;
    } else {
        s_active_ee_mass_kg = s_target_ee_mass_kg;
    }
}

void arm_gravity_comp_init(void) {
    s_payload_state = ARM_GRAVITY_PAYLOAD_EMPTY;
    s_active_ee_mass_kg = g_arm_gc_mass_ee_kg;
    s_target_ee_mass_kg = g_arm_gc_mass_ee_kg;
}

void arm_gravity_comp_set_payload_state(arm_gravity_payload_state_t state) {
    s_payload_state = state;
    s_target_ee_mass_kg =
        (s_payload_state == ARM_GRAVITY_PAYLOAD_LOADED) ?
        (g_arm_gc_mass_ee_kg + g_arm_gc_mass_cargo_kg) :
        g_arm_gc_mass_ee_kg;
}

arm_gravity_payload_state_t arm_gravity_comp_get_payload_state(void) {
    return s_payload_state;
}

float arm_gravity_comp_get_active_end_mass(void) {
    return s_active_ee_mass_kg;
}

float arm_gravity_comp_get_target_end_mass(void) {
    return s_target_ee_mass_kg;
}

void arm_gravity_comp_calculate(float theta2_rad,
                                float theta3_rad,
                                float theta4_rad,
                                float* tau2_nm,
                                float* tau3_nm,
                                float* tau4_nm) {
    if (!tau2_nm || !tau3_nm || !tau4_nm) return;

    update_mass_transition();

    const float theta_ee_abs = theta3_rad - theta4_rad;
    const float ee_common = s_active_ee_mass_kg * ARM_GRAVITY_G;

    const float ee_tau2 =
        ee_common * ARM_GRAVITY_LEN_L2_M * cosf(theta2_rad);
    const float ee_tau3 =
        ee_common * (ARM_GRAVITY_LEN_L3_M * cosf(theta3_rad) +
                     g_arm_gc_com_ee_x_m * cosf(theta_ee_abs));
    const float ee_tau4 =
        -ee_common * g_arm_gc_com_ee_x_m * cosf(theta_ee_abs);

    const float link3_tau3 =
        g_arm_gc_mass_l3_kg * ARM_GRAVITY_G *
        (g_arm_gc_com_r3_x_m * cosf(theta3_rad) -
         g_arm_gc_com_r3_y_m * sinf(theta3_rad));
    const float link3_tau2 =
        g_arm_gc_mass_l3_kg * ARM_GRAVITY_G *
        ARM_GRAVITY_LEN_L2_M * cosf(theta2_rad);

    const float link2_tau2 =
        g_arm_gc_mass_l2_kg * ARM_GRAVITY_G *
        (g_arm_gc_com_r2_x_m * cosf(theta2_rad) -
         g_arm_gc_com_r2_y_m * sinf(theta2_rad));

    *tau2_nm = ARM_GRAVITY_TAU2_SIGN * (link2_tau2 + link3_tau2 + ee_tau2);
    *tau3_nm = ARM_GRAVITY_TAU3_SIGN * (link3_tau3 + ee_tau3);
    *tau4_nm = ARM_GRAVITY_TAU4_SIGN * ee_tau4;
}
