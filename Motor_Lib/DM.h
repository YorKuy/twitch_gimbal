#ifndef __DM_H__
#define __DM_H__

#include "RM_Lib.h"

////////***********************************  DA MIAO 电 机 ********************************//////////
class MOTOR_DM
{ // 达妙电机 ,在这里定义的东西需要使用this来提取
public:
    const uint16_t ID; // 电机反馈ID
    USER_CAN *can_rev;

    int16_t id;  // 由达秒的串口助手设置
    int16_t ERR; // 反馈回来的电机错误信息，8：超压 9：欠压 A：过电流 B：mos过温 C：线圈过温 D：通讯丢失 E：过载
    int p_int;
    int v_int;
    int t_int;
    float mang;    // 位置 16位
    float sp;      //   速度  12位
    float Torque;  // 扭矩 12位
    float T_Rotor; // 表示电机内部线圈的平均温度 单位：摄氏度
    float T_MOS;   // 表示驱动上 MOS 的平均温度

    float nsqd_8PI_Cnt_mang; // 圈速*编码值
    float mang_inf;          // 过圈编码值
    uint8_t first = 0;       // 初始标志
    float Last_mang;         // 上次的角度值，判断过圈用
    int16_t motor_number;    // 圈速

    uint32_t motor_send_error_cnt = 0;                // 达妙单电机发送错误计次
    HAL_StatusTypeDef motor_send_state = HAL_TIMEOUT; // 电机发送状态/是否有调用标志
    HAL_StatusTypeDef DM_Start(uint16_t id);
    HAL_StatusTypeDef DM_End(uint16_t id);
    HAL_StatusTypeDef DM_Savezero(uint16_t id);
    HAL_StatusTypeDef DM_MIT(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
    HAL_StatusTypeDef DM_POS(uint16_t id, float _pos, float _vel);
    HAL_StatusTypeDef DM_VEL(uint16_t id, float _vel);
    void update_4PI_mang_inf_basic_zeromang(void); // 不改变0点的过圈检测
    HAL_StatusTypeDef DM_update(void);             // 得到速度，位置等参数
    MOTOR_DM(const uint16_t id, class USER_CAN *CAN_rev) : ID(id), can_rev(CAN_rev) {}
        
        float P_MIN = -3.141593f,
          P_MAX = 3.141593f,
          V_MIN = -30.0f,
          V_MAX = 30.0f,
          KP_MIN = 0.0f,
          KP_MAX = 500.0f,
          KD_MIN = 0.0f,
          KD_MAX = 5.0f,
          T_MIN = -12.0f,
          T_MAX = 12.0f;

private:
    
    // 这里定义的东西是用给class类里面的函数参数定义
};

#endif
