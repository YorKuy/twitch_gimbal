#ifndef __LEG_H__
#define __LEG_H__



float calculate_torque(float left_angle, float right_angle);
float torque_from_left_only(float left_angle);
float torque_from_right_only(float right_angle);
float get_left_torque(float angle);
float get_right_torque(float angle);
void get_both_torques(float left_angle, float right_angle, 
                      float* left_torque, float* right_torque);
#endif
