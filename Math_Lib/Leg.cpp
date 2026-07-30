#include <stdio.h>
#include <math.h>

#define LEG_MODE 1

#if LEG_MODE == 0
// 从左右角度计算力矩
float calculate_torque(float left_angle, float right_angle) {
    // 线性拟合参数
    const float l_intercept = -1.4771f;
    const float l_slope = -0.0260f;
    const float r_intercept = 0.1182f;
    const float r_slope = 0.0340f;
    
    // 分别计算
    float t_from_left = (left_angle - l_intercept) / l_slope;
    float t_from_right = (right_angle - r_intercept) / r_slope;
    
    // 返回平均值
    return (t_from_left + t_from_right) * 0.5f;
}

// 仅从左角度计算力矩
float torque_from_left_only(float left_angle) {
    const float l_intercept = -1.4771f;
    const float l_slope = -0.0260f;
    return (left_angle - l_intercept) / l_slope;
}

// 仅从右角度计算力矩
float torque_from_right_only(float right_angle) {
    const float r_intercept = 0.1182f;
    const float r_slope = 0.0340f;
    return (right_angle - r_intercept) / r_slope;
}
#endif

#if LEG_MODE == 1

#define L_A2 -0.0029f
#define L_A1 -0.1244f
#define L_A0 -0.7055f

#define R_A2 -0.0053f
#define R_A1 0.1648f
#define R_A0 -0.5575f

// 左腿力矩计算（返回正值）
float get_left_torque(float angle) {
    float a = L_A2, b = L_A1, c = L_A0 - angle;
    float disc = b*b - 4.0f*a*c;
    
    if (disc < 0) return 10.0f;  // 默认值
    
    float sqrt_disc = sqrtf(disc);
    float t1 = (-b + sqrt_disc) / (2.0f*a);
    float t2 = (-b - sqrt_disc) / (2.0f*a);
    
    // 范围限制
    float torque = (t1 >= 4.0f && t1 <= 15.0f) ? t1 :
                   (t2 >= 4.0f && t2 <= 15.0f) ? t2 : 10.0f;
    
    if (torque < 4.0f) torque = 4.0f;
    if (torque > 15.0f) torque = 15.0f;
    
    return torque;  // 正值
}

// 右腿力矩计算（返回负值）
float get_right_torque(float angle) {
    float a = R_A2, b = R_A1, c = R_A0 - angle;
    float disc = b*b - 4.0f*a*c;
    
    if (disc < 0) return -10.0f;  // 默认负值
    
    float sqrt_disc = sqrtf(disc);
    float t1 = (-b + sqrt_disc) / (2.0f*a);
    float t2 = (-b - sqrt_disc) / (2.0f*a);
    
    // 范围限制
    float torque = (t1 >= 4.0f && t1 <= 15.0f) ? t1 :
                   (t2 >= 4.0f && t2 <= 15.0f) ? t2 : 10.0f;
    
    if (torque < 4.0f) torque = 4.0f;
    if (torque > 15.0f) torque = 15.0f;
    
    return -torque;
}

// 同时获取双腿力矩
void get_both_torques(float left_angle, float right_angle, 
                      float* left_torque, float* right_torque) {
    *left_torque = get_left_torque(left_angle);   // 正值
    *right_torque = get_right_torque(right_angle); // 负值
}
#endif
