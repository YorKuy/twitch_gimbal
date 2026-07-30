#include "Shoot.h"
#include "PID.h"
#include "RM.h"
#include "communication.h"
#include "RM_Lib.h"
extern MOTOR_RM M3508_MCL_Right,
    M3508_MCL_Left,
    M2006_BP;
extern USER_CAN  CAN_1;
extern u16 remain_heat, shooter_id1_17mm_cooling_limit, shooter_id1_17mm_cooling_heat;
PID_class M3508_MCL_Left_Speed(20, 0, 0, 15000, 0, 0, 15000),
    M3508_MCL_Right_Speed(20, 0, 0, 15000, 0, 0, 15000);
PID_class M2006_BP_Speed(13, 0, 0, 16000, 0, 0, 16000),
    M2006_BP_Angle(0.4, 0, 0, 16000, 0, 10000, 16000),
    M2006_BP_Con_Speed(13, 0, 0, 16000, 0, 0, 16000),
    M2006_BP_Con_Angle(0.45, 0, 0, 16000, 0, 0, 16000);
UpDown_check_class UD_BP_ON(0), UD_BP_ALIGN(0), UD_BP_PREFAB_RELEASE(0), UD_BP_DF(0);
UpDown_check_class UD_PR_BP(0);
MCL *mcl = new MCL();
BP *bp = new BP();
static u8 buff_flag = 1;
extern u8 buff_mode;
static uint16_t test_flag_1,test_flag_2;

static float ramp_towards(float current, float target, float max_delta)
{
    float delta = target - current;
    if (delta > max_delta)
        return current + max_delta;
    if (delta < -max_delta)
        return current - max_delta;
    return target;
}

void slope(float *rec, float target, float slow_Inc)
{
    if (abs(*rec) - abs(target) < 0)
    {
    }
    if (abs(*rec) - abs(target) > 0)
        slow_Inc *= 2.5f;

    if (abs(*rec - target) < slow_Inc)
        *rec = target;
    else
    {
        if ((*rec) > target)
            (*rec) = target;
        if ((*rec) < target)
            (*rec) += slow_Inc;
    }
}
void MCL::MCL_while_layer(u8 YK_Mode)
{
    if (YK_Mode == SHOOT_MODE)
    {
        bp->Mode = GYRO_MODE;
        bp->currentMode = BP::FsmMode::GYRO;
        if (YK.yaogan.ch1 > P_YG)
        {
            mcl->Mode = MCL_ON;
        }
        else
        {
            mcl->Mode = MCL_OFF;
        }
    }
    else if (YK_Mode == PLAYER_MODE)
    {
        bp->Mode = AUTO_MODE;
        bp->currentMode = BP::FsmMode::AUTO;
        mcl->Mode = MCL_ON;
    }
    else if (YK_Mode == PROTECT_MODE)
    {
        bp->Mode = PROTECT_MODE;
        bp->currentMode = BP::FsmMode::PROTECT;
        mcl->Mode = MCL_OFF;
        bp->One_Target_Angle = bp->Continuous_Target_Angle = M2006_BP.mang_inf;
    }
    else
    {
        bp->Mode = PROTECT_MODE;
        bp->currentMode = BP::FsmMode::PROTECT;
        mcl->Mode = MCL_OFF;
    }
}

f *MCL::MCL_deal(u8 YK_Mode)
{
    if (M3508_MCL_Right.update() == HAL_OK)
    {
        mcl->Update_Flag |= 0x1;
        test_flag_1++;
    }
    if (M3508_MCL_Left.update() == HAL_OK)
    {
        mcl->Update_Flag |= 0x2;
        test_flag_2++;
    }
    if ((mcl->Update_Flag & 0x3) == 0x3)
    {
        const float desired_speed = mcl->Mode ? (mcl->MCL_Speed + mcl->MCL_Change) : 0.0f;
        const uint32_t now = HAL_GetTick();
        uint32_t dt_ms = 0;

        if (mcl->Last_Ramp_Tick != 0)
        {
            dt_ms = now - mcl->Last_Ramp_Tick;
            if (dt_ms > 20)
                dt_ms = 20;
        }
        mcl->Last_Ramp_Tick = now;

        const float ramp_speed = fabsf(mcl->MCL_Speed + mcl->MCL_Change);
        const float accel_step = (ramp_speed / 500.0f) * dt_ms;
        const float decel_step = (ramp_speed / 100.0f) * dt_ms;
        const float right_step = (fabsf(desired_speed) > fabsf(mcl->Target_R)) ? accel_step : decel_step;
        const float left_target = -desired_speed;
        const float left_step = (fabsf(left_target) > fabsf(mcl->Target_L)) ? accel_step : decel_step;

        mcl->Target_R = ramp_towards(mcl->Target_R, desired_speed, right_step);
        mcl->Target_L = ramp_towards(mcl->Target_L, left_target, left_step);

        if (mcl->Mode)
        {
            Charge_ON;
        }
        else
        {
            Charge_OFF;
        }

        M3508_MCL_Right_Speed.PID_new_update(mcl->Target_R, M3508_MCL_Right.sp);
        mcl->PID_OUT[0] = M3508_MCL_Right_Speed.OUT_PID;
        M3508_MCL_Left_Speed.PID_new_update(mcl->Target_L, M3508_MCL_Left.sp);
        mcl->PID_OUT[1] = M3508_MCL_Left_Speed.OUT_PID;
        mcl->Update_Flag = 0;
    }
    return mcl->PID_OUT;
}

static const BP::StateFunc kBPStateFuncTable[static_cast<u8>(BP::FsmState::COUNT)] = {
    &BP::stateZERO,
    &BP::stateOneShot,
    &BP::stateContinuous,
};

static BP::StateFunc stateFuncOf(BP::FsmState state)
{
    const u8 index = static_cast<u8>(state);
    if (index >= (sizeof(kBPStateFuncTable) / sizeof(kBPStateFuncTable[0])))
    {
        return &BP::stateZERO;
    }
    return kBPStateFuncTable[index];
}

void BP::switchState(FsmState newState)
{
    currentState = newState;
    currentStateFunc = stateFuncOf(newState);
    if (currentStateFunc == &BP::stateZERO && newState != FsmState::ZERO)
    {
        currentState = FsmState::ZERO;
    }
}

static f runCurrentState(BP *self)
{
    if (self->currentStateFunc == nullptr)
    {
        self->switchState(BP::FsmState::ZERO);
    }
    return (self->*self->currentStateFunc)();
}

// ZERO: 无射击
f BP::stateZERO(void)
{
    PID_OUT = 0;
    return PID_OUT;
}

// ONE_SHOT: 单发
f BP::stateOneShot(void)
{
    M2006_BP_Angle.PID_new_update(One_Target_Angle, M2006_BP.mang_inf);
    M2006_BP_Speed.PID_new_update(M2006_BP_Angle.OUT_PID, M2006_BP.sp);
    PID_OUT = M2006_BP_Speed.OUT_PID;
    return PID_OUT;
}

// CONTINUOUS: 连发
f BP::stateContinuous(void)
{
    M2006_BP_Con_Angle.PID_new_update(Continuous_Target_Angle, M2006_BP.mang_inf);
    M2006_BP_Con_Speed.PID_new_update(M2006_BP_Con_Angle.OUT_PID, M2006_BP.sp);
    PID_OUT = M2006_BP_Con_Speed.OUT_PID;
    return PID_OUT;
}

// 状态机入口：处理目标角度 + 根据 Mode 决定 State + PID 输出
f BP::BP_Out_Interface(u8 YK_Mode, u8 jianshu_flag)
{
    // 目标角度更新（原 BP_deal 逻辑）
    if (YK_Mode == SHOOT_MODE)
    {
        if (UD_BP_ON.updata(bp->ONE_ON) == UpDown_check_rising && mcl->Mode == 1 && Error_flag == 0)
        {
            bp->One_Target_Angle = M2006_BP.mang_inf;
            if (abs((int32_t)M2006_BP.mang_inf % (int32_t)BOPAN_ANGLE) < 10000)
                bp->Blockage_Compensation = 0;
            else
                bp->Blockage_Compensation = 1;
            if (bp->Blockage || bp->Blockage_To_Daed)
            {
            }
            else
            {
                bp->One_Target_Angle -= BOPAN_ANGLE;
                bp->energy.C_Shoot++;
                bp->Continuous_shooting_flag = 0;
                bp->Continuous_Target_Angle = bp->One_Target_Angle;
                bp->Blockage = 0;
                bp->Blockage_To_Daed = 0;
            }
        }
        else if (((bp->CON_ON || (request.zimiao_status && SuperPower.mode == 2)) && mcl->Mode == 1) && Error_flag == 0 && buff_mode != 1)
        {
            bp->Continuous_shooting_flag = 1;
            bp->Continuous_Target_Angle -= Shot_SP_1 * 15;
            bp->energy.real_inf = M2006_BP.mang_inf;
            bp->energy.total_real += bp->energy.real_inf - bp->energy.last_inf;
            bp->energy.C_Shoot = bp->energy.total_real / BOPAN_ANGLE;
            bp->energy.last_inf = M2006_BP.mang_inf;
            bp->One_Target_Angle = bp->Continuous_Target_Angle;
        }
        else
        {
            if (bp->Continuous_shooting_flag)
                bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->Continuous_shooting_flag = 0;
            bp->Continuous_Target_Angle = M2006_BP.mang_inf;
        }
    }
    else if (YK_Mode == PLAYER_MODE)
    {
        remain_heat = shooter_id1_17mm_cooling_limit - shooter_id1_17mm_cooling_heat;
        if (UD_BP_DF.updata(YK.shubiao.press_l) == UpDown_check_rising && (remain_heat > 30 || YK.Pressed_Check(KEY_PRESSED_CTRL)))
        {
            bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->One_Target_Angle -= BOPAN_ANGLE;
            bp->Continuous_Target_Angle = bp->One_Target_Angle;
            bp->Continuous_shooting_flag = 0;
            bp->delay_2ms = 0;
        }
        else if ((((YK.shubiao.press_l) && bp->delay_2ms > 150) || (request.zimiao_status && YK.shubiao.press_l && SuperPower.mode == 2 && mcl->Mode)) && buff_mode == 0)
        {
            bp->Continuous_shooting_flag = 1;
#if BP_TEST_FLAG
            bp->Continuous_Target_Angle += Shot_SP_1 * 7;
#else
            if (remain_heat < 20)
            {
                bp->Continuous_Target_Angle = bp->One_Target_Angle = M2006_BP.mang_inf;
                bp->Prefabricate_Flag = 0;
            }
            else if (remain_heat >= 20 && remain_heat < 60)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 5;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 60 && remain_heat < 120)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 10;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 120 && remain_heat < 150)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 15;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 150 && remain_heat < 190)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 18;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 190 && remain_heat < 220)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 20;
                bp->Prefabricate_Flag = 1;
            }
            else if (remain_heat >= 220)
            {
                bp->Continuous_Target_Angle -= Shot_SP_1 * 20;
                bp->Prefabricate_Flag = 1;
            }
#endif
        }
        else
        {
            if (bp->Continuous_shooting_flag)
                bp->One_Target_Angle = M2006_BP.mang_inf;
            bp->Continuous_shooting_flag = 0;
            bp->Continuous_Target_Angle = M2006_BP.mang_inf;
        }
    }
    (void)jianshu_flag;

    // 状态机：根据 Mode 决定 State
    FsmState nextState = FsmState::ZERO;

    switch (currentMode)
    {
        case FsmMode::GYRO:
            nextState = Continuous_shooting_flag ? FsmState::CONTINUOUS : FsmState::ONE_SHOT;
            break;
        case FsmMode::AUTO:
            nextState = Continuous_shooting_flag ? FsmState::CONTINUOUS : FsmState::ONE_SHOT;
            break;
        case FsmMode::PROTECT:
        default:
            nextState = FsmState::ZERO;
            break;
    }

    if (currentState != nextState)
    {
        switchState(nextState);
    }
    return runCurrentState(this);
}

void BP::Start_Recovery(double current_angle, double recovery_angle, u8 use_continuous_pid)
{
    this->Recovery_State = RECOVERY_ACTIVE;
    this->Recovery_Use_ContinuousPid = use_continuous_pid;
    this->Recovery_Target_Angle = current_angle + recovery_angle;
    this->Recovery_Last_Error = this->Recovery_Target_Angle - current_angle;
    this->One_Target_Angle = current_angle;
    this->Continuous_Target_Angle = current_angle;
    this->Continuous_shooting_flag = 0;
    this->Error_flag = 1;
}

void BP::Stop_Recovery(double current_angle)
{
    this->Recovery_State = RECOVERY_IDLE;
    this->Recovery_Use_ContinuousPid = 0;
    this->Recovery_Target_Angle = current_angle;
    this->Recovery_Last_Error = 0;
    this->One_Target_Angle = current_angle;
    this->Continuous_Target_Angle = current_angle;
    this->Continuous_shooting_flag = 0;
    this->BP_Error = 0;
    this->Error_flag = 0;
}

void BP::BP_while_layer()
{
    if (bp->Mode == GYRO_MODE || bp->Mode == AUTO_MODE)
    {
        if (YK.yaogan.ch0 < N_YG)
        {
            bp->State = ONE_ON_MASK;
        }
        else if (YK.yaogan.ch0 > P_YG)
        {
            bp->State = CON_ON_MASK;
        }
        else
        {
            bp->State = OFF_MASK;
        }
    }
    else
    {
        bp->State = OFF_MASK;
    }
    bp->ONE_ON = (bp->State & ONE_ON_MASK) ? 1 : 0;
    bp->CON_ON = (bp->State & CON_ON_MASK) ? 1 : 0;
}

void BP::BP_Energy()
{
    bp->test_num++;
    bp->energy.Energy_Shoot = bp->energy.C_Shoot * 10.0f - bp->energy.Energy_re / 10.0f;
    bp->energy.Energy_real += bp->energy.Energy_Shoot;
    if (bp->energy.Energy_real < 0)
    {
        bp->energy.Energy_real = 0;
        bp->energy.mode = GYRO_MODE;
    }
    else if (bp->energy.Energy_real > bp->energy.Energy_Clipping)
    {
        bp->energy.mode = PROTECT_MODE;
        bp->energy.Energy_real = bp->energy.Energy_Clipping;
    }
    else
        bp->energy.mode = GYRO_MODE;
    bp->energy.C_Shoot = 0;
}

void BP::BP_time_out(void)
{
    const uint32_t now_tick = HAL_GetTick();
    const u8 recovery_ready = (bp->Last_Blockage_Recovery_Tick == 0) ||
                              ((now_tick - bp->Last_Blockage_Recovery_Tick) >= BP_RECOVERY_MIN_INTERVAL_MS);
    const u8 use_continuous_pid = bp->Continuous_shooting_flag;
    const u8 one_pid_overload = (M2006_BP_Speed.OUT_PID > 12000 || M2006_BP_Speed.OUT_PID < -12000);
    const u8 con_pid_overload = (M2006_BP_Con_Speed.OUT_PID > 12000 || M2006_BP_Con_Speed.OUT_PID < -12000);

    if (!use_continuous_pid && one_pid_overload)
    {
        if (++bp->BP_Error > 200)
        {
            if (recovery_ready)
            {
                Error_flag = 1;
                bp->One_Target_Angle = bp->Continuous_Target_Angle = M2006_BP.mang_inf;
                BP_ERROR_ONE;
                bp->Last_Blockage_Recovery_Tick = now_tick;
                Error_flag = 0;
            }
            bp->BP_Error = 0;
        }
    }
    else if (use_continuous_pid && con_pid_overload)
    {
        if (++bp->BP_Error > 200)
        {
            if (recovery_ready)
            {
                Error_flag = 1;
                bp->Continuous_shooting_flag = 1;
                bp->Continuous_Target_Angle = bp->One_Target_Angle = M2006_BP.mang_inf;
                BP_ERROR_CON;
                bp->Last_Blockage_Recovery_Tick = now_tick;
                Error_flag = 0;
            }
            bp->BP_Error = 0;
        }
    }
    else
        bp->BP_Error = 0;

    bp->delay_2ms++;
}

f BP::BP_deal(u8 YK_Mode, u8 jianshu_flag)
{
    return BP_Out_Interface(YK_Mode, jianshu_flag);
}
