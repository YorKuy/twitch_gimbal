#include "Pitch.h"
#include "DM.h"
#include "PID.h"
#include "SMC.h"

#define PITCH_HIGH   -0.6987F
#define PITCH_LOW    0.00608F
#define PITCH_UP_GRAVITY_ERR_DB_DEG        1.0f
#define PITCH_UP_GRAVITY_ERR_RANGE_DEG     9.0f
#define PITCH_UP_GRAVITY_LOW_ANGLE_DEG    -12.0f
#define PITCH_UP_GRAVITY_HIGH_ANGLE_DEG    30.0f
#define PITCH_UP_GRAVITY_COMP_BASE         0.3f
#define PITCH_UP_GRAVITY_COMP_LOW_K        2.0f
#define PITCH_UP_GRAVITY_COMP_MAX          2.0f
PID_class M6020_Pitch_Speed(0.018f, 0, 0, 5.0f, 0, 0, 5.0f), // 180,0.3,0,0,10000,0,30000,2,50
    M6020_Pitch_Angle(20.0f, 0, 10.0f, 350.0f, 0, 10.0f, 400.0f);   // 15,0,0,0,10000,0,30000
PID_class PID_Pitch_sp_zm(0.025, 0.1, 0, 5, 3, 0, 6,3),
    PID_Pitch_mang_zm(20, 1, 20, 400, 100, 30, 400,4.0f); //
SMC         Pitch(45,70,0,0.001,15000,0.9,1,1),
            Pitch_Zm(60, 130, 0, 0.1, 16000, 1, 1, 1);
SMC_PITCH SMC_Pitch(45,55, 1.0f, 0.01f, 10000, 0.8f, 1),
            SMC_Pitch_Zm(52,65, 21.0f, 0.01f, 12000, 0.8f, 1.0f);
static PITCH pitch_instance;
PITCH *pitch = &pitch_instance;
extern MOTOR_DM DM_PITCH;
extern float Zm_Pitch_Vel, Zm_Pitch_Acc;

float gm6020to_torq_pitch(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
static inline float GetPitchUpGravityComp(float target_angle_deg, float current_angle_deg)
{
    const float up_error_deg = target_angle_deg - current_angle_deg;
    if (up_error_deg <= PITCH_UP_GRAVITY_ERR_DB_DEG)
        return 0.0f;
    const float low_factor = LIMIT((PITCH_UP_GRAVITY_HIGH_ANGLE_DEG - current_angle_deg) /
                                    (PITCH_UP_GRAVITY_HIGH_ANGLE_DEG - PITCH_UP_GRAVITY_LOW_ANGLE_DEG),0.0f,1.0f);

    const float error_factor = LIMIT((up_error_deg - PITCH_UP_GRAVITY_ERR_DB_DEG)
                                    /PITCH_UP_GRAVITY_ERR_RANGE_DEG,0.0f,1.0f);

    float comp = PITCH_UP_GRAVITY_COMP_BASE + PITCH_UP_GRAVITY_COMP_LOW_K * low_factor;
    comp *= error_factor;
    return LIMIT(comp, 0.0f, PITCH_UP_GRAVITY_COMP_MAX);
}

// 状态函数表
static const PITCH::StateFunc kPitchStateFuncTable[static_cast<u8>(PITCH::State::COUNT)] = {
    &PITCH::stateNORMAL,
    &PITCH::stateAUTO_ZM,
    &PITCH::stateZERO,
};

PITCH::StateFunc PITCH::stateFuncOf(State state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kPitchStateFuncTable) / sizeof(kPitchStateFuncTable[0])))
    {
        return &PITCH::stateZERO;
    }
    return kPitchStateFuncTable[index];
}

void PITCH::switchState(State newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &PITCH::stateZERO && newState != State::ZERO)
    {
        currentState = State::ZERO;
    }
}

f PITCH::runCurrentState(u8 jianshu_flag)
{
    if (currentStateFunc == nullptr)
    {
        switchState(State::ZERO);
    }
    return (this->*currentStateFunc)(jianshu_flag);
}

// NORMAL: 遥控/键鼠手动控制
f PITCH::stateNORMAL(u8 jianshu_flag)
{
    Angle_buf = jianshu_flag ? (float)(LIMIT(YK.shubiao.y, -400, 400) / 800.0) : (float)(YK.yaogan.ch3 / 3300.0);
    Target_Angle -= Angle_buf;
    if (DM_PITCH.mang < PITCH_HIGH + 0.0001 && Angle_buf > 0)
        Target_Angle = Last_Angle;
    if (DM_PITCH.mang > PITCH_LOW - 0.0001  && Angle_buf < 0)
        Target_Angle = Last_Angle;
    Last_Angle = Target_Angle;
    Target_Angle = LIMIT(Target_Angle, -38.5f,1.2f);
    Pitch.ref = Target_Angle;
    SMC_Pitch.SMC_Tick(Target_Angle, Angle_buf, 0, GIMBAL_088.realAngle.roll, GIMBAL_088.Anglespeed.Deal_roll);
    Pitch_Out = gm6020to_torq_pitch(SMC_Pitch.u);
    Pitch_Out = LIMIT(Pitch_Out, -7.0f, 7.0f);
    return Pitch_Out;
}

// AUTO_ZM: 自瞄零力矩控制
f PITCH::stateAUTO_ZM(u8 jianshu_flag)
{
    (void)jianshu_flag;
    SMC_Pitch_Zm.SMC_Tick(Target_Angle, Zm_Pitch_Vel, Zm_Pitch_Acc, GIMBAL_088.realAngle.roll, GIMBAL_088.Anglespeed.Deal_roll);
    Pitch_Out = gm6020to_torq_pitch(SMC_Pitch_Zm.u);
    Pitch_Out += GetPitchUpGravityComp(Target_Angle, GIMBAL_088.realAngle.roll);
    Pitch_Out = LIMIT(Pitch_Out, -7.0f, 7.0f);
    return Pitch_Out;
}

// ZERO: 保护模式，输出置零
f PITCH::stateZERO(u8 jianshu_flag)
{
    (void)jianshu_flag;
    Target_Angle = GIMBAL_088.realAngle.roll;
    Pitch_Out = 0;
    return Pitch_Out;
}

// 主入口：根据全局 PITCH_Mode 同步 currentMode 并切换 State
f PITCH::Pitch_Out_Interface(u8 jianshu_flag)
{
    // 同步全局 Mode 到状态机
    if (PITCH_Mode == GYRO_MODE)
        currentMode = Mode::GYRO;
    else if (PITCH_Mode == AUTO_MODE)
        currentMode = Mode::AUTO;
    else
        currentMode = Mode::PROTECT;

    State nextState = State::ZERO;

    switch (currentMode)
    {
        case Mode::GYRO:
            nextState = State::NORMAL;
            break;
        case Mode::AUTO:
            nextState = State::AUTO_ZM;
            break;
        case Mode::PROTECT:
        default:
            nextState = State::ZERO;
            break;
    }
    if (currentState != nextState)
    {
        switchState(nextState);
    }
    return runCurrentState(jianshu_flag);
}

void PITCH::set_Pitch_Target(f Target_Angle)
{
    pitch->Target_Angle = Target_Angle;
}
void PITCH::set_SMCref(f Target_Angle)
{
    Pitch.ref = Target_Angle;
    Pitch_Zm.ref = Target_Angle;
}
