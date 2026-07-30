#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "tim.h"
#include "usart.h"

void App_Gimbal_Init(void);
void App_Gimbal_Loop(void);
void App_Gimbal_CAN1_RxFifo0Callback(CAN_HandleTypeDef *hcan);
void App_Gimbal_CAN2_RxFifo1Callback(CAN_HandleTypeDef *hcan);
void App_Gimbal_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void App_Gimbal_USART1_IRQHandler(void);
void App_Gimbal_USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
