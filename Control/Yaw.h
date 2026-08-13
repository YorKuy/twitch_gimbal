#ifndef __YAW_H__
#define __YAW_H__

#include "main.h"
#include "RM_Lib.h"

// Phase-dependent chassis spin feed-forward tuning.  Front means the gimbal
// is near Motor_Yaw_front (cos(relative yaw) = +1); rear means cos = -1.
#ifndef CHASSIS_SPIN_FF_FRONT_GAIN
#define CHASSIS_SPIN_FF_FRONT_GAIN 0.001f
#endif
#ifndef CHASSIS_SPIN_FF_REAR_GAIN
#define CHASSIS_SPIN_FF_REAR_GAIN 0.003f
#endif
#ifndef CHASSIS_SPIN_FF_SIGN
#define CHASSIS_SPIN_FF_SIGN (1.0f)
#endif
#ifndef CHASSIS_SPIN_FF_MAX_TORQUE
#define CHASSIS_SPIN_FF_MAX_TORQUE 2.0f
#endif
#ifndef CHASSIS_SPIN_FF_TIMEOUT_MS
#define CHASSIS_SPIN_FF_TIMEOUT_MS 100U
#endif
#ifndef CHASSIS_SPIN_PHASE_SCALE
#define CHASSIS_SPIN_PHASE_SCALE 10000.0f
#endif

extern u8 YAW_Mode;
extern RC YK;
extern u16 Communicate_Send_Flag_1;
extern BMI088	GIMBAL_088;
class YAW
{
    public:      
        f Yaw_Out;        //Yaw最后的输出
        f Target_Angle;   //目标角度
        f Chassis_Spin_FF_Torque;       //便于在线观察前馈力矩
        f Chassis_Spin_Rear_Weight;     //0=前方，1=后方
        void set_SMCref(f Target_Angle);//接口
        void set_Yaw_Angle(f Target_Angle);
    YAW():Yaw_Out(0),Target_Angle(0),Chassis_Spin_FF_Torque(0),Chassis_Spin_Rear_Weight(0),
        Yaw_real(0),Yaw_vel(0),Yaw_acc(0),Back_Flag(0),
        chassis_spin_speed_target(0),chassis_spin_speed_filtered(0),
        chassis_spin_phase_sin_target(0),chassis_spin_phase_sin_filtered(0),
        chassis_spin_phase_cos_target(1),chassis_spin_phase_cos_filtered(1),
        chassis_spin_enabled(false),chassis_spin_update_tick(0){}
    enum class State
    {
        NORMAL,
        BACKING,
        AUTO_ZM,
        ZERO,
        COUNT
    };
    enum class Mode
    {
        PROTECT,
        GYRO,
        AUTO
    };
    using StateFunc = f (YAW::*)(u8 jianshu_flag);
    void yaw_mode_deal(u8 GIMBAL_088_State,u8 YK_Mode,u8 zm_request); //输入接口
    f stateNORMAL(u8 jianshu_flag);
    f stateBACKING(u8 jianshu_flag);
    f stateAUTO_ZM(u8 jianshu_flag);
    f stateZERO(u8 jianshu_flag);
    Mode  currentMode = Mode::PROTECT;
    State currentState = State::ZERO;
    StateFunc currentStateFunc = &YAW::stateZERO;
    void switchState(State newState);
    f Yaw_Out_Interface(u8 jianshu_flag); //输出接口
    void set_ChassisSpinState(f speed, bool enabled, f phase_sin, f phase_cos);
    private:
        static StateFunc stateFuncOf(State state);
        f runCurrentState(u8 jianshu_flag);
        f Yaw_real,Yaw_vel,Yaw_acc;  //陀螺仪实际角度，角速度，角加速度
        u8 Back_Flag;
        f chassis_spin_speed_target;
        f chassis_spin_speed_filtered;
        f chassis_spin_phase_sin_target;
        f chassis_spin_phase_sin_filtered;
        f chassis_spin_phase_cos_target;
        f chassis_spin_phase_cos_filtered;
        bool chassis_spin_enabled;
        uint32_t chassis_spin_update_tick;

        f chassisSpinFeedforward(void);
        
};
#endif
