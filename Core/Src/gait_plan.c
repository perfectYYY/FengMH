/*
 * gait_plan.c
 *
 *  Created on: Dec 29, 2025
 *      Author: FMI
 */

#include "gait_plan.h"
#include "arm_math.h"
#include "main.h"


// 机械腿尺寸参数初始化函数
void params_init(leg_size *leg_size, float thigh_length, float shin_length, float link_length,
                 float angle1_max, float angle1_min, float angle2_max, float angle2_min, float wheel_diameter,
                 float thigh_mass_1, float thigh_mass_2, float shin_mass, float link_mass, float wheel_mass,
                 float lc_t_m_1, float lc_t_m_2, float lc_s_m, float lc_l_m, float g)
{
    // 长度
    leg_size->thigh_length = thigh_length;
    leg_size->shin_length = shin_length;
    leg_size->link_length = link_length;
    // 轮子直径
    leg_size->wheel_diameter = wheel_diameter;
    // 角度范围
    leg_size->thigh_motor_angle_range.min = angle1_min;
    leg_size->thigh_motor_angle_range.max = angle1_max;
    leg_size->shin_motor_angle_range.min = angle2_min;
    leg_size->shin_motor_angle_range.max = angle2_max;
    // 质量
    leg_size->thigh_mass_1 = thigh_mass_1;
    leg_size->thigh_mass_2 = thigh_mass_2;
    leg_size->shin_mass = shin_mass;
    leg_size->link_mass = link_mass;
    leg_size->wheel_mass = wheel_mass;
    // 质心距离
    leg_size->lc_t_m_1 = lc_t_m_1;
    leg_size->lc_t_m_2 = lc_t_m_2;
    leg_size->lc_s_m = lc_s_m;
    leg_size->lc_l_m = lc_l_m;
    // 重力加速度
    leg_size->g = g;
}
// 步态参数初始化函数
void gait_action_init(gait_action *gait_action, float radius, float duty_ratio, float cycle_time, float point_number) 
{
    gait_action->radius = radius;
    gait_action->duty_ratio = duty_ratio;
    gait_action->cycle_time = cycle_time;
    gait_action->point_number = point_number;
};

//记录轨迹点信息函数
void gait_record(gait_action *gait_action, int indx, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az){
    // 初始化矩阵结构体
    gait_action->position[indx].vector.numRows = 3;
    gait_action->position[indx].vector.numCols = 1;
    gait_action->position[indx].vector.pData = gait_action->position[indx].data;

    gait_action->velocity[indx].vector.numRows = 3;
    gait_action->velocity[indx].vector.numCols = 1;
    gait_action->velocity[indx].vector.pData = gait_action->velocity[indx].data;

    gait_action->acceleration[indx].vector.numRows = 3;
    gait_action->acceleration[indx].vector.numCols = 1;
    gait_action->acceleration[indx].vector.pData = gait_action->acceleration[indx].data;
    // 记录数据
    gait_action->position[indx].data[0] = x;
    gait_action->position[indx].data[1] = y;
    gait_action->position[indx].data[2] = z;

    gait_action->velocity[indx].data[0] = vx;
    gait_action->velocity[indx].data[1] = vy;
    gait_action->velocity[indx].data[2] = vz;

    gait_action->acceleration[indx].data[0] = ax;
    gait_action->acceleration[indx].data[1] = ay;
    gait_action->acceleration[indx].data[2] = az;

    x = 0.0f, y = 0.0f, z = 0.0f;
    vx = 0.0f, vy = 0.0f, vz = 0.0f;
    ax = 0.0f, ay = 0.0f, az = 0.0f;
};

//计算出右前腿向前迈的轨迹点位置信息、速度信息和加速度信息,此处以时间为索引依据进行计算；摆线默认起点在对应腿坐标系下的原点
void compete_gait(gait_action *gait_action){
    int point_number = gait_action->point_number;   // 轨迹点个数
    float cycle_time = gait_action->cycle_time; // 周期时间
    float duty_ratio = gait_action->duty_ratio;  // 占空比
    float radius = gait_action->radius; // 摆线半径
    
    float distance = gait_action->radius * 2.0f * PI; // 每个周期前进的距离
    float support_speed = distance / (cycle_time * duty_ratio); // 支撑相直线速度
    float swing_speed = 2.0f * PI / (cycle_time * (1.0f - duty_ratio)); // 摆动相角速度
    float duty_time = cycle_time / point_number; // 每个轨迹点对应的时间间隔

    float front_time = cycle_time * duty_ratio / 2.0f; // 支撑相前半段时间,其实也是后半段的时间

    float x = 0.0f, y = 0.0f, z = 0.0f;
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;// 位置、速度、加速度变量初始都为0.0f
    for(float t = 0.0f; t < cycle_time; t += duty_time ){
        if(t < front_time){
            x = -t * support_speed;
            vx = -support_speed;

            gait_record(gait_action, (int)(t/duty_time), x, y, z, vx, vy, vz, ax, ay, az);
        }
        else if(t >= front_time && t < (cycle_time - front_time)){
            float t_use = t - front_time;
            x = radius * ((t_use * swing_speed) - sin(t_use * swing_speed)) - (gait_action->radius * PI);
            z = radius * (1.0f - cos(t_use * swing_speed));
            vx = radius * swing_speed * (1.0f - cos(t_use * swing_speed));
            vz = radius * swing_speed * sin(t_use * swing_speed);
            ax = radius * swing_speed * swing_speed * sin(t_use * swing_speed);
            az = radius * swing_speed * swing_speed * cos(t_use * swing_speed);

            gait_record(gait_action, (int)(t/duty_time), x, y, z, vx, vy, vz, ax, ay, az);
        }
        else if(t >= (cycle_time - front_time)){
             float t_use = t - (cycle_time - front_time);
            x = (distance / 2) - t_use * support_speed;
            vx = support_speed;
            
            gait_record(gait_action, (int)(t/duty_time), x, y, z, vx, vy, vz, ax, ay, az);
        }
    }

};


//运动学逆解1，计算给定位置对应的电机角度
void inverse_kinematics_position(leg_size *leg_size, vector *position, vector *motor_angle, float hight, quadruped_leg_type leg_type)
{
    float x = position->data[0];
    float z = position->data[2] + hight;

    float L1 = leg_size->thigh_length;
    float L2 = leg_size->shin_length;

    float D = sqrtf(x * x + z * z);
    float theta = acosf((L1 * L1 + D * D - L2 * L2) / (2.0f * L1 * D));

    if (leg_type == LEG_TYPE_MIRROR)
    {
        float a = atan2f(z, x) + theta;
        motor_angle->data[1] = a;
        motor_angle->data[2] = atan2f((z - (L1 * sinf(a))), (x - (L1 * cosf(a))));
    }
    else if (leg_type == LEG_TYPE_ORIGINAL)
    {
        float a = atan2f(z, x) - theta;
        motor_angle->data[1] = a;
        motor_angle->data[2] = atan2f((z - (L1 * sinf(a))), (x - (L1 * cosf(a))));
    }

}

//运动学正解
void forward_kinematics_position(leg_size *leg_size, vector *position, vector *motor_angle){
	float theta_1 = motor_angle->data[1];
	float theta_2 = motor_angle->data[2];

    float L1 = leg_size->thigh_length;
    float L2 = leg_size->shin_length;

    float x = L1 * cos(theta_1) + L2 * cos(theta_2);
    float z = L1 * sin(theta_1) + L2 * sin(theta_2);

    position->data[0] = x;
    position->data[2] = z;
}
