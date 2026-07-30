#ifndef __YAW_H__
#define __YAW_H__

#include "main.h"
#include "RM_Lib.h"

extern u8 YAW_Mode;
extern RC YK;
extern u16 Communicate_Send_Flag_1;
extern BMI088	GIMBAL_088;
class YAW
{
    public:      
        f Yaw_Out;        //Yaw最后的输出
        f Target_Angle;   //目标角度
        void set_SMCref(f Target_Angle);//接口
        void set_Yaw_Angle(f Target_Angle);
    YAW():Yaw_Out(0),Target_Angle(0),Yaw_real(0),Yaw_vel(0),Yaw_acc(0),Back_Flag(0){}
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
    private:
        static StateFunc stateFuncOf(State state);
        f runCurrentState(u8 jianshu_flag);
        f Yaw_real,Yaw_vel,Yaw_acc;  //陀螺仪实际角度，角速度，角加速度
        u8 Back_Flag;
        
};
#endif