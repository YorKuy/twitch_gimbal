#include "Yaw.h"
#include "SMC.h"
#include "RM.h"
#include "PID.h"
#include "DM.h"


#define YAW_ST 6600
UpDown_check_class UD_Yaw_Back(0);
SMC Yaw(28,50, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Zm(30, 55, 0, 0.01, 30000, 0.9, 1, 1),
    Yaw_Back(15, 50, 0, 0.001, 16000, 0.9, 1, 1);
PID_class yaw_angle(20, 0, 0, 450, 0, 10, 500),
          yaw_speed(0.02, 0, 0, 10, 0, 0, 10),
          yaw_back_angle(30,0,10,450,0,10,500),
          yaw_back_speed(0.035,0,0,10,0,0,10),
          yaw_angle_zm(30,0,0,450,0,0,500),
          yaw_speed_zm(0.025,0,0,5,0,0,5);
static YAW yaw_instance;
YAW *yaw = &yaw_instance;
extern MOTOR_RM M6020_YAW;
extern MOTOR_DM DM_YAW;
extern float Zm_Yaw_Vel, Zm_Yaw_Acc;
static float error_yaw;
static const YAW::StateFunc kYawStateFuncTable[static_cast<u8>(YAW::State::COUNT)] = {
    &YAW::stateNORMAL,
    &YAW::stateBACKING,
    &YAW::stateAUTO_ZM,
    &YAW::stateZERO,
};
//转力矩
float gm6020to_torq(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
//Yaw轴模式切换
void YAW::yaw_mode_deal(u8 GIMBAL_088_State,u8 YK_Mode,u8 zm_request)
{
    if (GIMBAL_088_State == BMI088_OK &&
        (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE ||
         YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) &&
        !zm_request)
    {
        currentMode = Mode::GYRO;
        YAW_Mode = GYRO_MODE;
    }
    else if (zm_request)
    {
        currentMode = Mode::AUTO;
        YAW_Mode = AUTO_MODE;
    }
    else
    {
        currentMode = Mode::PROTECT;
        YAW_Mode = PROTECT_MODE;
    }
}

void YAW::set_SMCref(f Target_Angle)
{
    Yaw.ref = Target_Angle;
    Yaw_Zm.ref = Target_Angle;
    Yaw_Back.ref = Target_Angle;
}
void YAW::set_Yaw_Angle(f Target_Angle)
{
    this->Target_Angle = Target_Angle;
}

void YAW::set_ChassisSpinState(f speed, bool enabled, f phase_sin, f phase_cos)
{
    // The chassis uses joystick-speed units (currently roughly 0..520).  A
    // bounded value prevents a malformed CAN frame from producing a large
    // feed-forward torque while still allowing the full configured range.
    if (speed > 1000.0f)
        speed = 1000.0f;
    else if (speed < -1000.0f)
        speed = -1000.0f;

    if (phase_sin > 1.0f)
        phase_sin = 1.0f;
    else if (phase_sin < -1.0f)
        phase_sin = -1.0f;
    if (phase_cos > 1.0f)
        phase_cos = 1.0f;
    else if (phase_cos < -1.0f)
        phase_cos = -1.0f;

    chassis_spin_speed_target = enabled ? speed : 0.0f;
    chassis_spin_phase_sin_target = enabled ? phase_sin : 0.0f;
    chassis_spin_phase_cos_target = enabled ? phase_cos : 1.0f;
    chassis_spin_enabled = enabled;
    chassis_spin_update_tick = HAL_GetTick();

    if (!enabled)
    {
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
    }
}

f YAW::chassisSpinFeedforward(void)
{
    const uint32_t now = HAL_GetTick();
    const bool timed_out = (uint32_t)(now - chassis_spin_update_tick) > CHASSIS_SPIN_FF_TIMEOUT_MS;
    if (!chassis_spin_enabled || timed_out)
    {
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
        return 0.0f;
    }

    // Smooth the command so entering/leaving XTL does not create a torque
    // step that would disturb the vision loop.
    chassis_spin_speed_filtered += 0.05f * (chassis_spin_speed_target - chassis_spin_speed_filtered);
    chassis_spin_phase_sin_filtered += 0.25f * (chassis_spin_phase_sin_target - chassis_spin_phase_sin_filtered);
    chassis_spin_phase_cos_filtered += 0.25f * (chassis_spin_phase_cos_target - chassis_spin_phase_cos_filtered);

    const f phase_norm = sqrtf(chassis_spin_phase_sin_filtered * chassis_spin_phase_sin_filtered +
                               chassis_spin_phase_cos_filtered * chassis_spin_phase_cos_filtered);
    f normalized_cos = (phase_norm > 0.1f) ? chassis_spin_phase_cos_filtered / phase_norm : 1.0f;
    if (normalized_cos > 1.0f)
        normalized_cos = 1.0f;
    else if (normalized_cos < -1.0f)
        normalized_cos = -1.0f;

    // Smoothly interpolate the gain: front centre -> 0, rear centre -> 1.
    // This matches the observed once-per-revolution mechanical disturbance
    // without changing the normal auto-aim controller gains.
    Chassis_Spin_Rear_Weight = 0.5f * (1.0f - normalized_cos);
    const f phase_gain = CHASSIS_SPIN_FF_FRONT_GAIN +
        (CHASSIS_SPIN_FF_REAR_GAIN - CHASSIS_SPIN_FF_FRONT_GAIN) * Chassis_Spin_Rear_Weight;

    f torque = CHASSIS_SPIN_FF_SIGN * phase_gain * chassis_spin_speed_filtered;
    if (torque > CHASSIS_SPIN_FF_MAX_TORQUE)
        torque = CHASSIS_SPIN_FF_MAX_TORQUE;
    else if (torque < -CHASSIS_SPIN_FF_MAX_TORQUE)
        torque = -CHASSIS_SPIN_FF_MAX_TORQUE;
    Chassis_Spin_FF_Torque = torque;
    return Chassis_Spin_FF_Torque;
}

YAW::StateFunc YAW::stateFuncOf(State state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kYawStateFuncTable) / sizeof(kYawStateFuncTable[0])))
    {
        return &YAW::stateZERO;
    }
    return kYawStateFuncTable[index];
}

void YAW::switchState(State newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &YAW::stateZERO && newState != State::ZERO)
    {
        currentState = State::ZERO;
    }
}

f YAW::runCurrentState(u8 jianshu_flag)
{
    if (currentStateFunc == nullptr)
    {
        switchState(State::ZERO);
    }
    return (this->*currentStateFunc)(jianshu_flag);
}

float YAW::stateNORMAL(u8 jianshu_flag)
{
    const f yaw_delta = jianshu_flag ?
        (f)LIMIT(YK.shubiao.x, -400, 400) / 1200.0f :
        (f)YK.yaogan.ch2 / 6600.0f;
    Target_Angle -= yaw_delta;

    if (UD_Yaw_Back.updata(YK.Pressed_Check(KEY_PRESSED_R)) == UpDown_check_rising)
    {
        Target_Angle += 180.0f;
        set_SMCref(Target_Angle);
        Communicate_Send_Flag_1 |= (0x0001 << 1);
        Back_Flag = 1;
        switchState(State::BACKING);
        return stateBACKING(jianshu_flag);
    }

    Back_Flag = 0;
    set_SMCref(Target_Angle);
    Yaw.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw.u);
    return Yaw_Out;
}

float YAW::stateAUTO_ZM(u8 jianshu_flag)
{
    (void)jianshu_flag;
    set_SMCref(Target_Angle);
    Yaw_Zm.SMC_AngleSpeed(Target_Angle, Zm_Yaw_Vel, Zm_Yaw_Acc, GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
    Yaw_Out = gm6020to_torq(Yaw_Zm.u);
    return Yaw_Out;
}

float YAW::stateBACKING(u8 jianshu_flag)
{
    set_SMCref(Target_Angle);
    if (fabsf(Target_Angle - GIMBAL_088.realAngle.yaw) > 5.0f)
    {
        Yaw_Back.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
        Yaw_Out = gm6020to_torq(Yaw_Back.u);
        return Yaw_Out;
    }

    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Back_Flag = 0;
    switchState(State::NORMAL);
    return stateNORMAL(jianshu_flag);
}

float YAW::stateZERO(u8 jianshu_flag)
{
    (void)jianshu_flag;
    Target_Angle = GIMBAL_088.realAngle.yaw;
    set_SMCref(Target_Angle);
    Back_Flag = 0;
    Communicate_Send_Flag_1 &= ~(0x0001 << 1);
    Yaw_Out = 0.0f;
    return Yaw_Out;
}
float YAW::Yaw_Out_Interface(u8 jianshu_flag)
{
    // 同步全局 YAW_Mode 到状态机
    if (YAW_Mode == GYRO_MODE)
        currentMode = Mode::GYRO;
    else if (YAW_Mode == AUTO_MODE)
        currentMode = Mode::AUTO;
    else
        currentMode = Mode::PROTECT;

    State nextState = State::ZERO;

    switch (currentMode)
    {
        case Mode::GYRO:
            nextState = Back_Flag ? State::BACKING : State::NORMAL;
            break;
        case Mode::AUTO:
            Back_Flag = 0;
            Communicate_Send_Flag_1 &= ~(0x0001 << 1);
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
    const f state_output = runCurrentState(jianshu_flag);
    if (currentMode == Mode::PROTECT)
    {
        // Do not retain a stale compensation command while the gimbal is
        // disabled.  This also makes the first frame after re-enabling safe.
        chassis_spin_speed_filtered = 0.0f;
        Chassis_Spin_FF_Torque = 0.0f;
        Chassis_Spin_Rear_Weight = 0.0f;
        Yaw_Out = 0.0f;
    }
    else
    {
        Yaw_Out = state_output + chassisSpinFeedforward();
    }
    return Yaw_Out;
}
