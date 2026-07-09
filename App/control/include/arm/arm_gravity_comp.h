/*
 * arm_gravity_comp.h - Arm gravity compensation model.
 *
 * Ported from damiao_new1/Core/Src/gravity_comp.c.
 */
#ifndef APP_CONTROL_ARM_GRAVITY_COMP_H_
#define APP_CONTROL_ARM_GRAVITY_COMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_GRAVITY_G 9.81f

#define ARM_GRAVITY_MASS_L2        1.083f
#define ARM_GRAVITY_MASS_L3        0.134f
#define ARM_GRAVITY_MASS_EE_EMPTY  0.613f
#define ARM_GRAVITY_MASS_CARGO_BOX 0.500f

#define ARM_GRAVITY_MASS_TRANSITION_STEP_KG 0.0020f

#define ARM_GRAVITY_LEN_L2_M 0.35f
#define ARM_GRAVITY_LEN_L3_M 0.30f

#define ARM_GRAVITY_COM_R2_X_M 0.2778f
#define ARM_GRAVITY_COM_R2_Y_M (-0.0235f)
#define ARM_GRAVITY_COM_R3_X_M (0.4447f - ARM_GRAVITY_LEN_L2_M)
#define ARM_GRAVITY_COM_R3_Y_M (-0.2311f)
#define ARM_GRAVITY_COM_EE_X_M 0.0368f

#define ARM_GRAVITY_TAU2_SIGN 1.0f
#define ARM_GRAVITY_TAU3_SIGN 1.0f
#define ARM_GRAVITY_TAU4_SIGN 1.0f

typedef enum {
    ARM_GRAVITY_PAYLOAD_EMPTY = 0,
    ARM_GRAVITY_PAYLOAD_LOADED = 1,
} arm_gravity_payload_state_t;

extern volatile float g_arm_gc_mass_l2_kg;
extern volatile float g_arm_gc_mass_l3_kg;
extern volatile float g_arm_gc_mass_ee_kg;
extern volatile float g_arm_gc_mass_cargo_kg;
extern volatile float g_arm_gc_com_r2_x_m;
extern volatile float g_arm_gc_com_r2_y_m;
extern volatile float g_arm_gc_com_r3_x_m;
extern volatile float g_arm_gc_com_r3_y_m;
extern volatile float g_arm_gc_com_ee_x_m;

void arm_gravity_comp_init(void);
void arm_gravity_comp_set_payload_state(arm_gravity_payload_state_t state);
arm_gravity_payload_state_t arm_gravity_comp_get_payload_state(void);
float arm_gravity_comp_get_active_end_mass(void);
float arm_gravity_comp_get_target_end_mass(void);

void arm_gravity_comp_calculate(float theta2_rad,
                                float theta3_rad,
                                float theta4_rad,
                                float* tau2_nm,
                                float* tau3_nm,
                                float* tau4_nm);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_ARM_GRAVITY_COMP_H_ */
