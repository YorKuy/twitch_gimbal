#include "app_gimbal.h"

#include "CP_System.h"
#include "DM.h"
#include "PID.h"
#include "Pitch.h"
#include "RM.h"
#include "RM_Lib.h"
#include "SMC.h"
#include "Shoot.h"
#include "Yaw.h"
#include "communication.h"
#include "gpio.h"
#include "hipnuc_dec.h"
#include "spi.h"

#include <math.h>
#include <stdint.h>

#define ROBOT_ID_MASK (0x0001 << 0)
#define DOGHOLE_FLAG_MASK (0x0001 << 1)
#define SPEED_CUT_FLAG_MASK (0x0001 << 2)
#define XTL_FLAG_MASK (0x0001 << 3)
#define CHASSIS_SPIN_FF_CAN_ID 0x116
#define PI 3.1415926
#define JG_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define PRINTF(x) INFO(#x "=%.1f\n", (x))
#define PRINTF_INFO(x) INFO(#x "=%d\n", (x));

USER_CAN CAN_1(&hcan1, 0),
    CAN_2(&hcan2, 1);
MOTOR_RM M6020_YAW(0x208, &CAN_2),
    M3508_MCL_Right(0x201, &CAN_1),
    M3508_MCL_Left(0x202, &CAN_1),
    M2006_BP(0x204, &CAN_2);

MOTOR_DM DM_PITCH(0x11, &CAN_1),
    DM_YAW(0x12, &CAN_2);
BMI088 GIMBAL_088(&hspi1, &htim7, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_4, 3000, 1.7f, BMI088_GYRO_RANGE_2000, BMI088_ACC_RANGE_24);

extern uint16_t zm_test_flag;
extern YAW *yaw;
extern PITCH *pitch;
extern MCL *mcl;
extern BP *bp;
extern uint8_t test_flag_1, test_flag_2;

Vision_LPF V_Pitch(30, 0.001, PI);
Vision_LPF V_Yaw(30, 0.001, PI);
Vision_LPF Yaw_PID_OUT(20, 0.001, PI);
Vision_LPF Pitch_PID_OUT(20, 0.001, PI);
Vision_LPF LPF_Deal_pitch(30, 0.001, PI);
Vision_LPF Pitch_MCU(10, 0.001, PI);
RC YK(&huart6, &huart1);

uint8_t YK_Mode = PROTECT_MODE;
uint8_t GIMBAL_088_State = BMI088_ERROR;
uint8_t YAW_Mode = PROTECT_MODE, PITCH_Mode = PROTECT_MODE;
f Pitch_Pid_Out, Yaw_Pid_Out, BP_PID_OUT;
int remain_heat;
uint8_t jianshu_flag = 0, Prefabricate_Flag = 0;
uint16_t shooter_id1_17mm_barrel_cooling_value, shooter_id1_17mm_cooling_limit = 0, shooter_id1_17mm_cooling_rate = 0, shooter_id1_17mm_cooling_heat = 0;
uint16_t Communicate_Send_Flag_1, Communicate_Rx_Flag_1;
uint8_t Buff_Flag = 0;
float LPF_Pitch_out;
uint8_t DR16_Stop_Flag;
uint16_t barrel_cooling_value = 0, cooling_limit = 0, cooling_heat = 0;

float *MCL_PID_OUT = nullptr;
uint8_t Right_Flag = 0;
float pitch_PID_OUT, LPF_Deal_pitch_out;

uint64_t disable_irq_1, disable_irq_2, disable_irq_3, disable_irq_4, disable_irq_5, disable_irq_6, disable_irq_7, disable_irq_8, disable_irq_9, disable_irq_10;
uint16_t zm_test_flag;
float Zm_Yaw_Vel, Zm_Yaw_Acc;
float Zm_Pitch_Vel, Zm_Pitch_Acc;
//INFO information
static uint8_t free_FIFO;
static uint8_t test_flag;
static uint8_t Motor_test_flag = 0;
static f test_out = 0;
static f pitch_test;
uint16_t Arrmor_id, Last_Arrmor_id;
uint8_t Arrmor_event_seq, Last_Arrmor_event_seq;
static u8 start_flag;
static u8 buff_start_flag;
u8 buff_mode;
static uint8_t start_yaw = 0, start_pitch = 0, time_yaw = 0;

UpDown_check_class UD_E(0), UD_SpeedUp(0), UD_SpeedDown(0), UD_Buff(0), UD_YK_BoPan(0), UD_BoPan_lian(0), UD_GenSui(0), UD_l(0), UD_ch0_exceed_600(0), UD_Laser(0);
UpDown_check_class UD_GenSui_chance(0);
YKStateTransitionDetector YK_MODE_SW_C_N(0), YK_MODE_SW_C_S(0), YK_MODE_SW_N_C(0), YK_MODE_SW_S_C(0);

static void MODE_DEAL(void)
{
  if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = PROTECT_MODE;
  else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = ONLY_GIMBAL;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = ONLY_CHASSIC;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = CONTROL_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = XTL_MODE;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = SHOOT_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = PLAYER_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = FAST_CHASSIC;
  else
    YK_Mode = PROTECT_MODE;

  if (GIMBAL_088_State == BMI088_OK && (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || (YK_Mode == SHOOT_MODE && !request.zimiao_status) || YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) && !request.zimiao_status)
  {
    YAW_Mode = GYRO_MODE;
  }
  else if (request.zimiao_status)
  {
    YAW_Mode = AUTO_MODE;
  }
  else
  {
    YAW_Mode = PROTECT_MODE;
  }

  if (GIMBAL_088_State == BMI088_OK && (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE || YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) && !request.zimiao_status)
  {
    PITCH_Mode = GYRO_MODE;
  }
  else if (request.zimiao_status)
  {
    PITCH_Mode = AUTO_MODE;
  }
  else
  {
    PITCH_Mode = PROTECT_MODE;
  }
}

static void can2_communicate_deal(void)
{
  static uint8_t flag = 0;

  if (YK.VT13_Data.mode_sw)
  {
    if (YK.VT13_rx_buffer[0] == 0xA9 && YK.VT13_rx_buffer[1] == 0x53)
    {
      YK.VT13_rx_buffer[7] = (YK.VT13_rx_buffer[7] & 0x0F) | (0xA << 4);
      CAN_2.Send_RM(0x110,
                    YK.VT13_rx_buffer[3] | (YK.VT13_rx_buffer[2] << 8),
                    YK.VT13_rx_buffer[5] | (YK.VT13_rx_buffer[4] << 8),
                    YK.VT13_rx_buffer[7] | (YK.VT13_rx_buffer[6] << 8),
                    YK.VT13_rx_buffer[18] | (YK.VT13_rx_buffer[17] << 8));
    }
  }
  else
  {
    CAN_2.Send_RM(0x110,
                  YK.DR16_rx_buffer[1] | (YK.DR16_rx_buffer[0] << 8),
                  YK.DR16_rx_buffer[3] | (YK.DR16_rx_buffer[2] << 8),
                  YK.DR16_rx_buffer[5] | (YK.DR16_rx_buffer[4] << 8),
                  YK.DR16_rx_buffer[15] | (YK.DR16_rx_buffer[14] << 8));
  }

  if (flag)
  {
    CAN_2.Send_RM(0x113, YK.DR16_rx_buffer[17] | (YK.DR16_rx_buffer[16] << 8), Communicate_Send_Flag_1, 0, 0);
    flag = 0;
  }
  else
  {
    CAN_2.Send_RM(0x115, request.zimiao_status, 0, 0, 0);
    flag = 1;
  }

  if (CAN_2.RxHeader.StdId == 0x558)
  {
    Communicate_Rx_Flag_1 = CAN_2.rx_buf[1] | CAN_2.rx_buf[0] << 8;
    shooter_id1_17mm_barrel_cooling_value = CAN_2.rx_buf[3] | CAN_2.rx_buf[2] << 8;
    shooter_id1_17mm_cooling_limit = CAN_2.rx_buf[5] | CAN_2.rx_buf[4] << 8;
    shooter_id1_17mm_cooling_heat = CAN_2.rx_buf[7] | CAN_2.rx_buf[6] << 8;
  }

}

static void jianshu_deal(void)
{
  if (YK_Mode == PLAYER_MODE)
  {
    jianshu_flag = GYRO_MODE;
  }
  else
    jianshu_flag = PROTECT_MODE;

  if (YK.shubiao.press_r || YK.yaogan.v < -600)
  {
    request.zimiao_status = 1;
    Right_Flag = 1;
  }
  else
  {
    request.zimiao_status = 0;
    Right_Flag = 0;
  }
  if (jianshu_flag)
  {
    if (YK.Pressed_Check(KEY_PRESSED_Z) && YK.Pressed_Check(KEY_PRESSED_CTRL))
    {
      uint8_t re_time;
      HAL_Delay(5);
      CAN_1.Send_RM(0x2FF, 0, 0, 0, 0);
      HAL_Delay(10);

      for (re_time = 0; re_time < 25; re_time++)
      {
        Yaw_Pid_Out = 0;
        CAN_2.Send_RM(0x1ff, 0, 0, 0, 0);
        HAL_Delay(10);
      }
      CAN_1.Send_RM(0x200, 0, 0, 0, 0);
      HAL_Delay(20);
      __set_FAULTMASK(1);
      NVIC_SystemReset();
    }
  }
  if (Communicate_Rx_Flag_1 & ROBOT_ID_MASK)
    AS.colour.typeMum = 0;
  else
    AS.colour.typeMum = 1;
  if (GIMBAL_088_State != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 0);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 0);
  if (Buff_Flag != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 2);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 2);
  if (UD_SpeedUp.updata(YK.Pressed_Check(KEY_PRESSED_F)) == UpDown_check_rising && (YK.jianpan & KEY_PRESSED_CTRL))
  {
    mcl->MCL_Change += 50;
  }

  if (UD_SpeedDown.updata(YK.Pressed_Check(KEY_PRESSED_F)) == UpDown_check_rising && !(YK.jianpan & KEY_PRESSED_CTRL))
  {
    mcl->MCL_Change -= 50;
  }
  if (UD_Buff.updata(YK.Pressed_Check(KEY_PRESSED_V)) == UpDown_check_rising || (YK_Mode == SHOOT_MODE && YK.yaogan.v > 600))
  {
    buff_mode = (buff_mode + 1) % 3;
  }
}

static uint8_t start_deal(void)
{
  static u8 open = 0;
  CAN_1.Init(0, 0);
  HAL_Delay(5);
  CAN_2.Init(1, 1);
  HAL_Delay(5);
  YK.VT13_Init();
  YK.DT16_Init();
  HAL_Delay(5);
  Mini_PC_Init();
  HAL_Delay(5);
  DM_PITCH.DM_Start(0x01);
  HAL_Delay(10);
  DM_YAW.DM_Start(0x02);
  HAL_Delay(5);
  GIMBAL_088_State = GIMBAL_088.Init();
  HAL_Delay(10);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim2);
  V_Yaw.Vision_Low_Pass_Filter_Init();
  V_Pitch.Vision_Low_Pass_Filter_Init();
  Yaw_PID_OUT.Vision_Low_Pass_Filter_Init();
  Pitch_PID_OUT.Vision_Low_Pass_Filter_Init();
  LPF_Deal_pitch.Vision_Low_Pass_Filter_Init();
  Pitch_MCU.Vision_Low_Pass_Filter_Init();
  HAL_Delay(5);
  //DM_PITCH.DM_Savezero(0x01);
  open = 1;
  return open;
}

static float ZM_Angle_Deal(float ZM_Angle, float Now_Angle, float Now_Current_Angle)
{
  float diff = ZM_Angle - Now_Angle;

  if (diff > 180.0)
  {
    diff -= 360.0;
  }
  else if (diff < -180.0)
  {
    diff += 360.0;
  }

  return Now_Current_Angle + diff;
}

void App_Gimbal_Init(void)
{
  HAL_Delay(500);
  start_flag = start_deal();
}

void App_Gimbal_Loop(void)
{
  JG_ON;
  MODE_DEAL();
  mcl->MCL_while_layer(YK_Mode);
  bp->BP_while_layer();
  jianshu_deal();
  yaw->set_SMCref(yaw->Target_Angle);
  pitch->set_SMCref(pitch->Target_Angle);
}

void App_Gimbal_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    can2_communicate_deal();
  }
  if (htim == &htim6)
  {
    if (!DR16_Stop_Flag)
      YK.DT16_watchdog_run();
    YK.VT13_watchdog_run();
    static uint8_t re_start_flag = 0, motor_flag = 0;
    if (re_start_flag % 10 == 0)
    {
      if (motor_flag)
      {
        DM_YAW.DM_Start(0x02);
        test_flag++;
        re_start_flag = 0;
        motor_flag = 0;
      }
      else
      {
        DM_PITCH.DM_Start(0x01);
        motor_flag = 1;
        re_start_flag = 0;
      }
    }
    re_start_flag++;
  }
  if (htim == &htim7)
  {
    GIMBAL_088.analyse();
    bp->BP_time_out();
  }
  if (htim == &htim2)
  {
    static uint8_t dm_send_flag = 0;
    if (dm_send_flag % 2 == 0)
    {
      if (DM_PITCH.ERR == 1)
      {
        if (PITCH_Mode == PROTECT_MODE)
          DM_PITCH.DM_MIT(0x01, 0, 0, 0, 0, 0);
        else
          DM_PITCH.DM_MIT(0x01, 0, 0, 0, 0.05, Pitch_Pid_Out);//-Pitch_Pid_Out
      }
    }
    else
    {
      if (DM_YAW.ERR == 1)
      {
        if (YAW_Mode == PROTECT_MODE)
          DM_YAW.DM_MIT(0x02, 0, 0, 0, 0, 0);
        else
          DM_YAW.DM_MIT(0x02, 0, 0, 0, 0, Yaw_Pid_Out);
      }
    }
    dm_send_flag++;
  }

  if (htim == &htim8)
  {
    static uint8_t mcl_stop_flag = 0;
    AS.Q_info_0.f = GIMBAL_088.q0_t;
    AS.Q_info_1.f = GIMBAL_088.q1_t;
    AS.Q_info_2.f = GIMBAL_088.q2_t;
    AS.Q_info_3.f = GIMBAL_088.q3_t;
    AS.heat_speed.f = 24.4;
    Mini_PC_SendData();
    static uint8_t mcl_flag = 0;
    if (YK_Mode == PROTECT_MODE && ++ mcl_flag > 10)
    {
      CAN_1.Send_RM(0x200, 0, 0, 0, 0);
    }
    else
    {
     mcl_flag = 0;
    CAN_1.Send_RM(0x200, mcl->PID_OUT[0], mcl->PID_OUT[1], 0, 0);
    }
  }
}

void App_Gimbal_CAN1_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  if (CAN_1.Receive(&hcan1) == HAL_OK)
  {
    if (DM_PITCH.DM_update() == HAL_OK)
    {
      Pitch_Pid_Out = pitch->Pitch_Out_Interface(jianshu_flag);
    }
    MCL_PID_OUT = mcl->MCL_deal(YK_Mode);
  }
}

void App_Gimbal_CAN2_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
  if (CAN_2.Receive(&hcan2) == HAL_OK)
  {
    if (CAN_2.RxHeader.StdId == CHASSIS_SPIN_FF_CAN_ID)
    {
      const int16_t spin_speed = (int16_t)((CAN_2.rx_buf[0] << 8) | CAN_2.rx_buf[1]);
      const uint16_t spin_enabled = (uint16_t)((CAN_2.rx_buf[2] << 8) | CAN_2.rx_buf[3]);
      const int16_t spin_phase_sin = (int16_t)((CAN_2.rx_buf[4] << 8) | CAN_2.rx_buf[5]);
      const int16_t spin_phase_cos = (int16_t)((CAN_2.rx_buf[6] << 8) | CAN_2.rx_buf[7]);
      yaw->set_ChassisSpinState((float)spin_speed, spin_enabled != 0U,
                                (float)spin_phase_sin / CHASSIS_SPIN_PHASE_SCALE,
                                (float)spin_phase_cos / CHASSIS_SPIN_PHASE_SCALE);
    }

    if (DM_YAW.DM_update() == HAL_OK)
    {
      Yaw_Pid_Out = yaw->Yaw_Out_Interface(jianshu_flag);
      CAN_2.Send_RM(0x1FE, 0, 0, 0, -Yaw_Pid_Out);
    }
    if (M2006_BP.update() == HAL_OK)
    {
      BP_PID_OUT = bp->BP_deal(YK_Mode, jianshu_flag);
      CAN_2.Send_RM(0x200, 0, 0, 0, BP_PID_OUT);
    }
  }
}

void App_Gimbal_USART1_IRQHandler(void)
{
  YK.VT13_RxCplt_IRQHandler();
  if (YK_MODE_SW_C_N.update(YK.VT13_Data.mode_sw) == C_N || YK_MODE_SW_C_S.update(YK.VT13_Data.mode_sw) == C_S)
  {
    HAL_UART_DMAStop(&huart6);
    __HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE | UART_IT_TC | UART_IT_IDLE);
    disable_irq_1++;
    DR16_Stop_Flag = 1;
    YK.DT16_set_zero();
  }
  if (YK_MODE_SW_N_C.update(YK.VT13_Data.mode_sw) == N_C || YK_MODE_SW_S_C.update(YK.VT13_Data.mode_sw) == S_C)
  {
    __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    HAL_UART_Receive_DMA(&huart6, YK.DR16_rx_buffer, 25);
    disable_irq_2++;
    DR16_Stop_Flag = 0;
    YK.DT16_set_zero();
  }
}

void App_Gimbal_USART2_IRQHandler(void)
{
  uint32_t temp;
  if ((__HAL_UART_GET_FLAG(&MINI_PC_USART_HANDLE, UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&MINI_PC_USART_HANDLE);
    temp = MINI_PC_USART_HANDLE.Instance->SR;
    temp = MINI_PC_USART_HANDLE.Instance->DR;
    (void)temp;
    HAL_UART_DMAStop(&MINI_PC_USART_HANDLE);
    GetReceive_SP(Mini_PC_rx_buf);
    HAL_UART_Receive_DMA(&MINI_PC_USART_HANDLE, (uint8_t *)Mini_PC_rx_buf, 128);

    if (Mini_PC_rx_buf[0] == 0x66 && Mini_PC_rx_buf[28] == 0x11)
    {
      if (YK_Mode == PLAYER_MODE || YK_Mode == SHOOT_MODE)
      {
        if (request.zimiao_status)
        {
          if (request.buff_status)
          {
          }
          else
          {
            if (SuperPower.mode == 1 || SuperPower.mode == 2)
            {
              Zm_Yaw_Vel = (SuperPower.yaw_vel.f * 57.3);
              Zm_Yaw_Acc = (SuperPower.yaw_acc.f * 57.3);
              Zm_Pitch_Vel = (SuperPower.pitch_vel.f * 57.3);
              Zm_Pitch_Acc = (SuperPower.pitch_acc.f * 57.3);
              yaw->Target_Angle = ZM_Angle_Deal(SuperPower.yaw.f * 180.0 / PI, GIMBAL_088.nowAngle.yaw, GIMBAL_088.realAngle.yaw);
              pitch->Target_Angle = (SuperPower.pitch.f * 180.0 / PI);
            }
          }
        }
      }
    }
  }
}
