#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "main.h"
#include "RM_Lib.h"
#define MCL_ON 1
#define MCL_OFF 0
#define P_YG   600
#define N_YG  -600
#define Speed_Clipping  6200 //6200
#define ONE_ON_MASK  (1 << 0)
#define CON_ON_MASK  (1 << 1)
#define OFF_MASK  0
#define Shot_SP_1			36.57								// Shot_SP_1：单发拨盘补偿系数（需实测）
#define BOPAN_ANGLE         (8192 * 36.0f /8.0f)									//����(8192 * 36.0 / 8.0f)
#define BP_TEST_FLAG 0
#define BP_ERROR_ONE   bp->One_Target_Angle += BOPAN_ANGLE  *1.5F
#define BP_ERROR_CON   bp->Continuous_Target_Angle += 1.3F*BOPAN_ANGLE
#define BP_RECOVERY_MIN_INTERVAL_MS 400
#define Charge_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)			//ӫ�����ģ��
#define Charge_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

extern RC YK;
class MCL
{
    private:
        f Target_R,Target_L;
        u8 Update_Flag;
        uint32_t Last_Ramp_Tick;
    public:
        f  PID_OUT[2];
        u8 Mode,State;
        f  MCL_Speed = 0,MCL_Change = 0;
        void MCL_while_layer(u8 YK_Mode);
        f    *MCL_deal(u8 YK_Mode);
        MCL():
            Target_R(0),
            Target_L(0),
            Update_Flag(0),
            Last_Ramp_Tick(0),
            PID_OUT{0, 0},
            Mode(MCL_OFF),
            State(0),
            MCL_Speed(-5900),
            MCL_Change(0)
        {}

};
class BP
{
    private:
        enum RecoveryState
        {
            RECOVERY_IDLE = 0,
            RECOVERY_ACTIVE = 1
        };
        u8 ONE_ON,CON_ON; //����������־
        u8 Prefabricate_Flag;
        f  BP_Error;
        u8 YZ_flag;
        u8 Error_flag;
        u8 BP_State_Buffer;
        struct Energy
        {
            f Energy_re; //�����ָ�
            f Energy_total;  //����������ģʽ�͵ȼ���
            f Energy_Clipping; //�����޷�
            int C_Shoot;     //�������
            u8 mode;
            f Energy_Shoot; //�������
            f Energy_real;  //������ʵֵ
            f last_inf,real_inf,total_real;
            Energy():Energy_re(14),Energy_total(230),Energy_Clipping(200),C_Shoot(0),mode(GYRO_MODE),Energy_Shoot(0),Energy_real(0),
                     last_inf(0),real_inf(0),total_real(0){}
        };
        Energy energy;
        u8 Continuous_shooting_flag;
        RecoveryState Recovery_State;
        u8 Recovery_Use_ContinuousPid;
        double Recovery_Target_Angle;
        double Recovery_Last_Error;
        uint32_t Last_Blockage_Recovery_Tick;
        void Start_Recovery(double current_angle, double recovery_angle, u8 use_continuous_pid);
        void Stop_Recovery(double current_angle);
      public:
        u8 Mode,State;
        int delay_2ms;
        double  Continuous_Target_Angle,One_Target_Angle;
        f  PID_OUT;
        u8 Blockage_Compensation,Blockage,Blockage_To_Daed,shoot_flag;
        void BP_while_layer();
        f BP_Out_Interface(u8 YK_Mode, u8 jianshu_flag);
        void BP_Energy();
        void BP_time_out(void);
        //INFO
        u8 test_num;

        // 状态机成员
        enum class FsmState
        {
            ZERO,
            ONE_SHOT,
            CONTINUOUS,
            COUNT
        };
        enum class FsmMode
        {
            PROTECT,
            GYRO,
            AUTO
        };
        using StateFunc = f (BP::*)(void);
        FsmMode  currentMode = FsmMode::PROTECT;
        FsmState currentState = FsmState::ZERO;
        StateFunc currentStateFunc = &BP::stateZERO;
        void switchState(FsmState newState);
        f stateOneShot(void);
        f stateContinuous(void);
        f stateZERO(void);
        f BP_deal(u8 YK_Mode, u8 jianshu_flag);
        BP():
            ONE_ON(0),
            CON_ON(0),
            Prefabricate_Flag(0),
            BP_Error(0),
            YZ_flag(0),
            Error_flag(0),
            BP_State_Buffer(0),
            energy(),
            Continuous_shooting_flag(0),
            Recovery_State(RECOVERY_IDLE),
            Recovery_Use_ContinuousPid(0),
            Recovery_Target_Angle(0),
            Recovery_Last_Error(0),
            Last_Blockage_Recovery_Tick(0),
            Mode(PROTECT_MODE),
            State(OFF_MASK),
            delay_2ms(0),
            Continuous_Target_Angle(0),
            One_Target_Angle(0),
            PID_OUT(0),
            Blockage_Compensation(0),
            Blockage(0),
            Blockage_To_Daed(0),
            shoot_flag(0),
            test_num(0),
            currentMode(FsmMode::PROTECT),
            currentState(FsmState::ZERO),
            currentStateFunc(&BP::stateZERO)
        {}

};
#endif
