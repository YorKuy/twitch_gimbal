/**
  ******************************************************************************
  * File Name				: RM_Lib.cpp
  * Description			: RM c++库，包含USART，CAN，麦轮底盘算法，全向轮算法，舵轮算法，PID，遥控器（包含拨轮），键鼠加卡尔曼滤波
											陀螺仪（BMI088优化），snail电调，裁判系统交互等库函数
  * Version					: 1.2
  * Creation Date		: 2023.5.17
  ******************************************************************************
  */
#include "RM_Lib.h"

/**************************************** USART **********************************************************/
uint8_t info_ubuf[USART_BUF_SIZE];

/**************************************** C A N **********************************************************/
void USER_CAN::Init(uint16_t t,uint16_t x)
{
  CAN_FilterTypeDef sFilterConfig={0};
    
	this->FreeTxNum=0;
	this->TxHeader.DLC=8;
		
  sFilterConfig.FilterBank = t;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;   
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	
	if(this->FIFO == 0) sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	else sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;

	sFilterConfig.FilterActivation = ENABLE;  	
	sFilterConfig.SlaveStartFilterBank = x;

  if (HAL_CAN_ConfigFilter(this->hcan, &sFilterConfig) != HAL_OK)Error_Handler();
  if (HAL_CAN_Start(this->hcan) != HAL_OK)Error_Handler();
	
	if(this->FIFO == 0)
	{
		if (HAL_CAN_ActivateNotification(this->hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK) 
			Error_Handler();
	}
	else
	{
		if (HAL_CAN_ActivateNotification(this->hcan,CAN_IT_RX_FIFO1_MSG_PENDING)!= HAL_OK) 
			Error_Handler();
	}
}
HAL_StatusTypeDef USER_CAN::Send(uint16_t Id,uint8_t* pData)
{
    this->TxHeader.StdId=Id;
//    do {
		this->FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(this->hcan);
//	}
//    while (this->FreeTxNum == 0);
    return HAL_CAN_AddTxMessage(this->hcan,&this->TxHeader, pData,&this->TxMailbox);
}
HAL_StatusTypeDef USER_CAN::Send_RM(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204)
{
	uint8_t pData[8];
	pData[0] = (int8_t) (M_201 >> 8);
	pData[1] = (int8_t) M_201;
	pData[2] = (int8_t) (M_202 >> 8);
	pData[3] = (int8_t) M_202;
	pData[4] = (int8_t) (M_203 >> 8);
	pData[5] = (int8_t) M_203;
	pData[6] = (int8_t) (M_204 >> 8);
	pData[7] = (int8_t) M_204;
	return this->Send(Id,pData);
}

HAL_StatusTypeDef USER_CAN::Send_DAMIAO(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204)
{
	uint8_t pData[8];
	
	pData[0] = (int8_t) M_201;
	pData[1] = (int8_t) (M_201 >> 8);
	pData[2] = (int8_t) M_202;
	pData[3] = (int8_t) (M_202 >> 8);
	pData[4] = (int8_t) M_203;
	pData[5] = (int8_t) (M_203 >> 8);
	pData[6] = (int8_t) M_204;
	pData[7] = (int8_t) (M_204 >> 8);
	return this->Send(Id,pData);
}
 
HAL_StatusTypeDef USER_CAN::Receive(CAN_HandleTypeDef *hcan)
{
		if(this->FIFO == 0) return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO0,&this->RxHeader,this->rx_buf);
		else return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO1,&this->RxHeader,this->rx_buf);
}
/*************************************************************************************************************/

MOTOR_DiPan::MOTOR_DiPan(void)
{
	this->ML.qz = 0;
	this->ML.qy = 0;
	this->ML.hz = 0;
	this->ML.hy = 0;
}

void MOTOR_DiPan::ML_Data_Deal(float lx,float ly,float lp,int MAX_rate) 
{
	float qy_in,qz_in,hy_in,hz_in;
	float sqy_in,sqz_in,shy_in,shz_in,sx,sp,sy; 
	float sqy_per,sqz_per,shz_per,shy_per;
	float Y_max,M_max,P_max;
	float qy_per,qz_per,hz_per,hy_per,Y_per,P_per;
	
	if(ly>0)sy=ly;else sy=-ly;
	if(lp>0)sp=lp;else sp=-lp;
	if(lx>0)sx=lx;else sx=-lx;
	
	Y_max=sy;
	
	if(Y_max<sx)Y_max=sx;
	if(Y_max<sp)Y_max=sp;
	
	Y_per=Y_max/660;
	
	qy_in=lx+ly-lp;
	qz_in=lx-ly+lp;
	hy_in=lx-ly-lp;
	hz_in=lx+ly+lp;
	
	if(qy_in>0)sqy_in=qy_in;else sqy_in=-qy_in;
	if(qz_in>0)sqz_in=qz_in;else sqz_in=-qz_in;
	if(hy_in>0)shy_in=hy_in;else shy_in=-hy_in;
	if(hz_in>0)shz_in=hz_in;else shz_in=-hz_in;
	
	M_max=sqy_in;
	if(M_max<sqz_in)M_max=sqz_in;
	if(M_max<shy_in)M_max=shy_in;
	if(M_max<shz_in)M_max=shz_in;
	
	if(M_max==0)
	{
		qy_per=0;
		qz_per=0;
		hy_per=0;
		hz_per=0;
	}
	else {
		qy_per=qy_in/M_max;
		qz_per=qz_in/M_max;
		hy_per=hy_in/M_max;
		hz_per=hz_in/M_max;
	}
	
	if(qy_per>0)sqy_per=qy_per;else sqy_per=-qy_per;
	if(qz_per>0)sqz_per=qz_per;else sqz_per=-qz_per;
	if(hy_per>0)shy_per=hy_per;else shy_per=-hy_per;
	if(hz_per>0)shz_per=hz_per;else shz_per=-hz_per;
	
	P_max=sqy_per;
	if(P_max<sqz_per)P_max=sqz_per;
	if(P_max<shy_per)P_max=shy_per;
	if(P_max<shz_per)P_max=shz_per;
	
	P_per=(P_max*4-(sqy_per+sqz_per+shy_per+shz_per))/3.1415926f+1;
//		P_per=1;
	
	this->ML.qy=-qy_per*Y_per*MAX_rate*P_per;
	this->ML.qz=qz_per*Y_per*MAX_rate*P_per;
	this->ML.hy=-hy_per*Y_per*MAX_rate*P_per;
	this->ML.hz=hz_per*Y_per*MAX_rate*P_per;
}

/*************************************************** Quan Xiang Lun  ******************************************************/

MOTOR_QXL_DiPan::MOTOR_QXL_DiPan(void)
{
	this->QXL.qz = 0;
	this->QXL.qy = 0;
	this->QXL.hz = 0;
	this->QXL.hy = 0;
}

float Q_MAX(float A,float B,float C,float D)
{
	A=A>B?A:B;
	A=A>C?A:C;
	A=A>D?A:D;
	return A;
}

void MOTOR_QXL_DiPan::QXL_Data_Deal(float lx,float ly,float lp,float angle,int MAX_rate) 
{
	  float sqy_in,sqz_in,shy_in,shz_in,sx,sp,sy; 
	  float QXL_angle,QXL_p,QXL_MAX,Y_MAX;
		float qz_in,qy_in,hz_in,hy_in;
	  float qy_per,qz_per,hz_per,hy_per,Y_per,P_per;
		if(lx==0&&ly==0)
		{
			QXL_angle=0;
			QXL_p=0;
		}
		else
		{
			QXL_angle=atan2(lx,ly);
			QXL_p=sqrtf(lx*lx+ly*ly);
		}
		
		//全向轮算法  angle是小陀螺移动
		 qz_in= QXL_p*sinf(QXL_angle+angle+QXL_PI/4)+lp;
		 qy_in=-QXL_p*cosf(QXL_angle+angle+QXL_PI/4)+lp;
		 hz_in= QXL_p*cosf(QXL_angle+angle+QXL_PI/4)+lp;
   	 hy_in=-QXL_p*sinf(QXL_angle+angle+QXL_PI/4)+lp;
		
		//遥感最大值
		if(lx>0)sx=lx;else sx=-lx;
		if(ly>0)sy=ly;else sy=-ly;
	  if(lp>0)sp=lp;else sp=-lp;
		Y_MAX=sy;
		if(Y_MAX<sx)Y_MAX=sx;
		if(Y_MAX<sp)Y_MAX=sp;
		Y_per=Y_MAX/660;
		
		//最终输出最大值
	 if(qz_in>0)sqz_in=qz_in;else sqz_in=-qz_in; 
	 if(qy_in>0)sqy_in=qy_in;else sqy_in=-qy_in;
	 if(hz_in>0)shz_in=hz_in;else shz_in=-hz_in;
	 if(hy_in>0)shy_in=hy_in;else shy_in=-hy_in;
	 QXL_MAX=Q_MAX(sqz_in,sqy_in,shz_in,shy_in);
		
		if(QXL_MAX==0)
		{
			qz_per=0;
			qy_per=0;	
			hz_per=0;
			hy_per=0;	
		}
		else
		{
			qz_per=qz_in/QXL_MAX;
		  qy_per=qy_in/QXL_MAX;
			hz_per=hz_in/QXL_MAX;
			hy_per=hy_in/QXL_MAX;
		}
		
		this->QXL.qz=qz_per*Y_per*MAX_rate;
		this->QXL.qy=qy_per*Y_per*MAX_rate;
		this->QXL.hz=hz_per*Y_per*MAX_rate;
		this->QXL.hy=hy_per*Y_per*MAX_rate;
}

/***************************************Duo Lun************************************************************************/
float RUDDER_DiPan:: Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1)
{
  if (CH0 == 0)
    return abs(CH1);
  if (CH1 == 0)
    return abs(CH0);
  return sqrtf(CH0 * CH0 + CH1* CH1);	
}

float RUDDER_DiPan:: Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2)
{
  float ch2_sqrt2 = 1.4142135623730950488016887242097f * abs(CH2);
  float ch01_max = Get_Ch0_Ch1_Vector_Speed(CH0,CH1);
  return ch2_sqrt2 > ch01_max ? ch2_sqrt2 :ch01_max;
}

float RUDDER_DiPan::absf(float d0)
{
  return d0 >= 0 ? d0 : -d0;
}

float RUDDER_DiPan:: Get_Max_float(float d0,float d1)
{
  d0 = absf(d0);
  return d0 > d1 ? d0 : d1;
}

int16_t RUDDER_DiPan:: Get_Max_int16(int16_t d0,int16_t d1)
{
  d0 = abs(d0);
  d1 = abs(d1);
  return d0 > d1 ? d0 : d1;
}

void RUDDER_DiPan:: Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2,int16_t SPEED_MAX)
{
	uint8_t flag;
	
	if(flag==0)
	{
		g_angle_6020[QZ] = ZERO_0[QZ];
		g_angle_6020[HZ] = ZERO_0[HZ];
		g_angle_6020[HY] = ZERO_0[HY];
		g_angle_6020[QY] = ZERO_0[QY];
		flag=1;
	}
	
  if(CH0 == 0 && CH1 == 0 && CH2 == 0)
	{
		g_wheel_3508[QZ].yaogan_speed = 0;
		g_wheel_3508[HZ].yaogan_speed = 0;
		g_wheel_3508[HY].yaogan_speed = 0;
		g_wheel_3508[QY].yaogan_speed = 0;
    return ;
	}

	float CH2_COS45 = COS_45 * CH2;
	
	if(this->follow_chassic == Yaw_mid_of_0)//4744
	{
		QZ_xy.x = CH0 + CH2_COS45,QZ_xy.y = CH1 - CH2_COS45;
		HZ_xy.x = -CH0 - CH2_COS45,HZ_xy.y = -CH1 - CH2_COS45;
		HY_xy.x = CH0 - CH2_COS45,HY_xy.y = CH1 + CH2_COS45;
		QY_xy.x = -CH0 + CH2_COS45,QY_xy.y = -CH1 + CH2_COS45;	
		
		g_wheel_3508[QZ].yaogan_speed = sqrtf(QZ_xy.x * QZ_xy.x + QZ_xy.y * QZ_xy.y);
		g_wheel_3508[HZ].yaogan_speed = sqrtf(HZ_xy.x * HZ_xy.x + HZ_xy.y * HZ_xy.y);
		g_wheel_3508[HY].yaogan_speed = sqrtf(HY_xy.x * HY_xy.x + HY_xy.y * HY_xy.y);
		g_wheel_3508[QY].yaogan_speed = sqrtf(QY_xy.x * QY_xy.x + QY_xy.y * QY_xy.y);
	}
	else if(this->follow_chassic == Yaw_mid_of_45)//5778
	{
		QZ_xy.x = CH0 + 0,QZ_xy.y = CH1 - CH2;
		HZ_xy.x = -CH0 - CH2,HZ_xy.y = -CH1 - 0;
		HY_xy.x = -CH0 - 0,HY_xy.y = -CH1 - CH2;
		QY_xy.x = -CH0 + CH2,QY_xy.y = -CH1 + 0;
		
		g_wheel_3508[QZ].yaogan_speed = sqrtf(QZ_xy.x * QZ_xy.x + QZ_xy.y * QZ_xy.y);
		g_wheel_3508[HZ].yaogan_speed = sqrtf(HZ_xy.x * HZ_xy.x + HZ_xy.y * HZ_xy.y);
		g_wheel_3508[HY].yaogan_speed = sqrtf(HY_xy.x * HY_xy.x + HY_xy.y * HY_xy.y);
		g_wheel_3508[QY].yaogan_speed = sqrtf(QY_xy.x * QY_xy.x + QY_xy.y * QY_xy.y);
	}
  float max_speed_nor = Get_Max_float(
                          Get_Max_float(
                            Get_Max_float(g_wheel_3508[QZ].yaogan_speed,g_wheel_3508[HZ].yaogan_speed)
                          ,g_wheel_3508[HY].yaogan_speed)
                        ,g_wheel_3508[QY].yaogan_speed);
  float max_speed_yaogan = Get_Max_int16(Get_Max_int16(Get_Max_int16(CH0,CH1),CH2),CH2);
  g_wheel_3508[QZ].yaogan_speed = g_wheel_3508[QZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[HZ].yaogan_speed = g_wheel_3508[HZ].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[HY].yaogan_speed = g_wheel_3508[HY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
  g_wheel_3508[QY].yaogan_speed = g_wheel_3508[QY].yaogan_speed / max_speed_nor * (max_speed_yaogan / 660) * SPEED_MAX;
	
	if(this->follow_chassic == Yaw_mid_of_0)
	{
		g_angle_6020[QZ] = atan2(QZ_xy.x,QZ_xy.y) * RAD2MANG+ ZERO_0[QZ];
		g_angle_6020[HZ] = atan2(HZ_xy.x,HZ_xy.y) * RAD2MANG+ ZERO_0[HZ];
		g_angle_6020[HY] = atan2(HY_xy.x,HY_xy.y) * RAD2MANG+ ZERO_0[HY];
		g_angle_6020[QY] = atan2(QY_xy.x,QY_xy.y) * RAD2MANG+ ZERO_0[QY];
	}
	else if(this->follow_chassic == Yaw_mid_of_45)
	{
		g_angle_6020[QZ] = atan2(QZ_xy.x,QZ_xy.y) * RAD2MANG+ ZERO_45[QZ];
		g_angle_6020[HZ] = atan2(HZ_xy.x,HZ_xy.y) * RAD2MANG+ ZERO_45[HZ];
		g_angle_6020[HY] = atan2(HY_xy.x,HY_xy.y) * RAD2MANG+ ZERO_45[HY];
		g_angle_6020[QY] = atan2(QY_xy.x,QY_xy.y) * RAD2MANG+ ZERO_45[QY];
	}
	
}

void RUDDER_DiPan::Xiaotuoluo_jie_Suan(uint16_t Mang_yaw,int16_t CH0,int16_t CH1,int16_t CH2,int16_t SPEED_MAX)
{
  int16_t mang_yaw_int16 = Mang_yaw << 3;
	mang_yaw_int16 /= 8;
	float yaw_angle =  -mang_yaw_int16 / RAD2MANG;
  float yaokong_angle = atan2(-CH0,-CH1);
  float max_speed_yaogan = Get_Max_int16(CH0,CH1);
  float vector_x = sinf(yaw_angle + yaokong_angle) * max_speed_yaogan,vector_y = cosf(yaw_angle + yaokong_angle) * max_speed_yaogan;
  Not_Xiaotuoluo_Jie_Suan(vector_x,vector_y,CH2,SPEED_MAX);
}
/****************************************** D B U S **********************************************************/
void RC::DT16_watchdog_run(void)
{
	this->DT16_time_100ms++;
	if(this->DT16_time_100ms > 5)
	{
		this->DT16_set_zero();
		this->DT16_feed_watchdog();
		HAL_UART_Abort_IT(this->F_huart);
		this->DT16_Init();
	}
}
void RC::VT13_watchdog_run(void)
{
	this->VT13_time_100ms++;
	if(this->VT13_time_100ms > 5)
	{
//		this->VT13_set_zero();
		this->VT13_feed_watchdog();
		HAL_UART_Abort_IT(this->S_huart);
		this->VT13_Init();
	}
}
void RC::DT16_Init(void)
{
	this->DT16_set_zero();
	while(this->F_huart->Init.BaudRate != 100000)
	{
		INFO("DEBUS Init Error\n");
		HAL_Delay(10);
	}
	
	__HAL_UART_ENABLE_IT(this->F_huart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(this->F_huart, DR16_rx_buffer,25);
}
void RC::VT13_Init(void)
{
	this->VT13_set_zero();
	while(this->S_huart->Init.BaudRate != 921600)
	{
		INFO("DEBUS Init Error\n");
		HAL_Delay(10);
	}
	
	__HAL_UART_ENABLE_IT(this->S_huart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(this->S_huart, VT13_rx_buffer,21);
}
void RC::DT16_RxCplt_IRQHandler(void)
{
	uint32_t temp;
	if(__HAL_UART_GET_FLAG(this->F_huart,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(this->F_huart);
		temp = this->F_huart->Instance->SR;  
		temp = this->F_huart->Instance->DR; 
		HAL_UART_DMAStop(this->F_huart); 
		if(DT16_check_and_deal() == HAL_OK)
		{
			this->DT16_data_deal();
			this->DT16_feed_watchdog();
			this->fill_data();
		}
		HAL_UART_Receive_DMA(this->F_huart,DR16_rx_buffer,25);
	}
}
void RC::VT13_RxCplt_IRQHandler(void)
{
	uint32_t temp;
	if(__HAL_UART_GET_FLAG(this->S_huart,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(this->S_huart);
		temp = this->S_huart->Instance->SR;  
		temp = this->S_huart->Instance->DR; 
		HAL_UART_DMAStop(this->S_huart); 
		if(VT13_check_and_deal() == HAL_OK)
		{
			this->VT13_data_deal();
			this->VT13_feed_watchdog();
			this->fill_data();
		}
		HAL_UART_Receive_DMA(this->S_huart,VT13_rx_buffer,21);
	}
}

HAL_StatusTypeDef RC::DT16_check_and_deal(void)
{
		if(this->shubiao.z != 0 || 
			 DR16_rx_buffer[18]	!= 0 || DR16_rx_buffer[19] != 0 || 
			 this->shubiao.press_r >= 2 || this->shubiao.press_l >= 2)
		{
			this->DT16_set_zero();
			return HAL_ERROR;
		}
		else 
			return HAL_OK;
}
HAL_StatusTypeDef RC::VT13_check_and_deal(void)
{
		if(VT13_rx_buffer[0] == 0xA9 && VT13_rx_buffer[1] == 0x53)
		{
			return HAL_OK;
		}
		else 
		{
			if(!(VT13_rx_buffer[0] == 0xA5 && VT13_rx_buffer[1] == 0x0C))
			{
				this->VT13_set_zero();
				return HAL_ERROR;
			}
		}
		return HAL_ERROR;
}
void RC::DT16_set_zero(void)
{
	this->DT16_yaogan.ch0 			= 0;
	this->DT16_yaogan.ch1 			= 0;
	this->DT16_yaogan.ch2 			= 0;
	this->DT16_yaogan.ch3 			= 0;
	this->DT16_yaogan.s1 			= YK_SW_UP;
	this->DT16_yaogan.s2 			= YK_SW_UP;
	this->DT16_shubiao.press_l		= 0;
	this->DT16_shubiao.press_r 		= 0;
	this->DT16_shubiao.x 			= 0;
	this->DT16_shubiao.y 			= 0;
	this->DT16_shubiao.z 			= 0;
	this->DT16_yaogan.v 			= 0;
	this->DT16_jianpan 				= 0;
	
	for(int i = 1; i <= 25; i++)
	{
		DR16_rx_buffer[i-1] = 0;
		if(i == 6)
		DR16_rx_buffer[i-1] = (DR16_rx_buffer[i-1] & 0x0F) | (0x5 << 4);//清零时双上
	}
	if(!this->VT13_Data.mode_sw)
	this->fill_data();
}
void RC::VT13_set_zero(void)
{
	this->VT13_Data.ch_0 			= 0;
	this->VT13_Data.ch_1 			= 0;
	this->VT13_Data.ch_2 			= 0;
	this->VT13_Data.ch_3 			= 0;
	this->VT13_Data.mode_sw			= 0;
	this->VT13_Data.pause			= 0;
	this->VT13_Data.fn_1			= 0;
	this->VT13_Data.fn_2			= 0;
	this->VT13_Data.wheel			= 0;
	this->VT13_Data.trigger			= 0;
	this->VT13_Data.mouse_x			= 0;
	this->VT13_Data.mouse_y			= 0;
	this->VT13_Data.mouse_z			= 0;
	this->VT13_Data.mouse_left		= 0;
	this->VT13_Data.mouse_right		= 0;
	this->VT13_Data.mouse_middle	= 0;
	this->VT13_Data.key				= 0;
	this->VT13_Data.crc16			= 0;
	
	for(int j = 1; j <= 25; j++)
	{
		VT13_rx_buffer[j-1] = 0;
	}
}
void RC::DT16_data_deal(void)
{
	this->DT16_yaogan.ch0 = ((DR16_rx_buffer[0]| (DR16_rx_buffer[1] << 8)) & 0x07ff)-1024;        	//!< Channel 0
	this->DT16_yaogan.ch1 = (((DR16_rx_buffer[1] >> 3) | (DR16_rx_buffer[2] << 5)) & 0x07ff)-1024;	//!< Channel 1
	this->DT16_yaogan.ch2 = (((DR16_rx_buffer[2] >> 6) | (DR16_rx_buffer[3] << 2) |             		//!< Channel 2
										 (DR16_rx_buffer[4] << 10)) & 0x07ff)-1024;
	this->DT16_yaogan.ch3 = (((DR16_rx_buffer[4] >> 1) | (DR16_rx_buffer[5] << 7)) & 0x07ff)-1024; //!< Channel 3
	this->DT16_yaogan.s1  = ((DR16_rx_buffer[5] >> 4)& 0x000C) >> 2;                           		//!< Switch left 
	this->DT16_yaogan.s2  = ((DR16_rx_buffer[5] >> 4)& 0x0003);                               		 	//!< Switch right 
	this->DT16_shubiao.x = (int16_t)(DR16_rx_buffer[6] | (DR16_rx_buffer[7]<< 8));             		//!< Mouse X axis 
	this->DT16_shubiao.y = (int16_t)(DR16_rx_buffer[8] | (DR16_rx_buffer[9]<< 8));             		//!< Mouse Y axis 
	this->DT16_shubiao.z = (int16_t)(DR16_rx_buffer[10] | (DR16_rx_buffer[11] << 8));         			//!< Mouse Z axis 
	this->DT16_shubiao.press_l = DR16_rx_buffer[12];                                        	 			//!< Mouse Left Is Press ? 
	this->DT16_shubiao.press_r = DR16_rx_buffer[13];                                       	 			//!< Mouse Right Is Press ? 
	this->DT16_jianpan = DR16_rx_buffer[14] | (DR16_rx_buffer[15]<< 8);
	this->DT16_yaogan.v =((DR16_rx_buffer[16]| (DR16_rx_buffer[17] << 8)) & 0x07ff)-1024; 
}

void RC::VT13_data_deal(void)
{
	this->VT13_Data.ch_0 = ((VT13_rx_buffer[2]| (VT13_rx_buffer[3] << 8)) & 0x07ff);
	this->VT13_Data.ch_1 = (((VT13_rx_buffer[3] >> 3) | (VT13_rx_buffer[4] << 5)) & 0x07ff);
	this->VT13_Data.ch_2 = (((VT13_rx_buffer[4] >> 6) | (VT13_rx_buffer[5] << 2) |
										 (VT13_rx_buffer[6] << 10)) & 0x07ff);
	this->VT13_Data.ch_3 = (((VT13_rx_buffer[6] >> 1) | (VT13_rx_buffer[7] << 7)) & 0x07ff);
	this->VT13_Data.mode_sw  = ((VT13_rx_buffer[7] >> 4) & 0x0003); 
	this->VT13_Data.pause  = ((VT13_rx_buffer[7] >> 6) & 0x01);
	this->VT13_Data.fn_1  = ((VT13_rx_buffer[7] >> 7) & 0x01);
	this->VT13_Data.fn_2  = ((VT13_rx_buffer[8] >> 0) & 0x01);
	this->VT13_Data.wheel  = ((VT13_rx_buffer[8] >> 1) | (VT13_rx_buffer[9] << 7)) & 0x07FF;
	this->VT13_Data.trigger  = (VT13_rx_buffer[9] >> 4) & 0x01;
	
	this->VT13_Data.mouse_x = (VT13_rx_buffer[10] | (VT13_rx_buffer[11] << 8));
	this->VT13_Data.mouse_y = (VT13_rx_buffer[12] | (VT13_rx_buffer[13] << 8));
	this->VT13_Data.mouse_z = (VT13_rx_buffer[14] | (VT13_rx_buffer[15] << 8));
	this->VT13_Data.mouse_left = (VT13_rx_buffer[16] >> 0) & 0x03;
	this->VT13_Data.mouse_right = (VT13_rx_buffer[16] >> 2) & 0x03;
	this->VT13_Data.mouse_middle = (VT13_rx_buffer[16] >> 4) & 0x03;
	this->VT13_Data.key =(VT13_rx_buffer[17] | (VT13_rx_buffer[18] << 8));
	this->VT13_Data.crc16 = (VT13_rx_buffer[19] | (VT13_rx_buffer[20] << 8));//不建议使用
	
	this->VT13_Data.ch_0 -= RC_CH_VALUE_OFFSET;
	this->VT13_Data.ch_1 -= RC_CH_VALUE_OFFSET;
	this->VT13_Data.ch_2 -= RC_CH_VALUE_OFFSET;
	this->VT13_Data.ch_3 -= RC_CH_VALUE_OFFSET;
	this->VT13_Data.wheel -= RC_CH_VALUE_OFFSET;//-1024
}

void RC::can_receive_data_deal(uint8_t *buf)
{
	this->yaogan.ch0 = ((buf[0]| (buf[1] << 8)) & 0x07ff)-1024;            //!< Channel 0
	this->yaogan.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff)-1024;    //!< Channel 1
	this->yaogan.ch2 = (((buf[2] >> 6) | (buf[3] << 2) |     //!< Channel 2
						(buf[4] << 10)) & 0x07ff)-1024;
	this->yaogan.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff)-1024;    //!< Channel 3
	this->yaogan.s1  = ((buf[5] >> 4)& 0x000C) >> 2;         //!< Switch left
	this->yaogan.s2  = ((buf[5] >> 4)& 0x0003);              //!< Switch right
	this->jianpan = buf[6] | (buf[7]<< 8);                   //!< KeyBoard value
	
	if(this->yaogan.ch0==-1024 || this->yaogan.ch1==-1024 ||
	   this->yaogan.ch2==-1024 || this->yaogan.ch3==-1024 ||
	   this->yaogan.s1 > 3	   || this->yaogan.s2 > 3	  ||
		 this->yaogan.s1 == 0  || this->yaogan.s2 == 0)
	{
		this->DT16_set_zero();
	}
}

uint8_t RC::Pressed_Check(uint16_t keyvalue)
{
	if(this->jianpan & keyvalue)  return 1;
	else  return 0;
}

void RC::fill_data(void)
{
	if(this->VT13_Data.mode_sw)
	{
		this->yaogan.ch0 = this->VT13_Data.ch_0;
		this->yaogan.ch1 = this->VT13_Data.ch_1;
		this->yaogan.ch2 = this->VT13_Data.ch_2;
		this->yaogan.ch3 = this->VT13_Data.ch_3;
		
		this->jianpan = this->VT13_Data.key;
		
		this->yaogan.s1 = YK_SW_DOWN;
		this->yaogan.s2 = YK_SW_DOWN;//this->VT13_Data.mouse_x
		
		this->shubiao.x = this->VT13_Data.mouse_x;
		this->shubiao.y = this->VT13_Data.mouse_y;
		this->shubiao.z = this->VT13_Data.mouse_z;
		this->shubiao.press_l = this->VT13_Data.mouse_left;
		this->shubiao.press_r = this->VT13_Data.mouse_right;
	}
	else
	{
		this->yaogan.ch0 = this->DT16_yaogan.ch0;
		this->yaogan.ch1 = this->DT16_yaogan.ch1;
		this->yaogan.ch2 = this->DT16_yaogan.ch2;
		this->yaogan.ch3 = this->DT16_yaogan.ch3;
		this->yaogan.s1 = DT16_yaogan.s1;
		this->yaogan.s2 = DT16_yaogan.s2;
		this->shubiao.x = this->DT16_shubiao.x;
		this->shubiao.y = this->DT16_shubiao.y;
		this->shubiao.z = this->DT16_shubiao.z;
		this->shubiao.press_l = this->DT16_shubiao.press_l;
		this->shubiao.press_r = this->DT16_shubiao.press_r;
		this->jianpan = this->DT16_jianpan;
		this->yaogan.v = this->DT16_yaogan.v;
	}
}

#if mouse_KF == 1

float RC::SF(float t,float *slopeFilter,float res)
{
	for(int i = SF_LENGTH-1;i>0;i--)
	{
		slopeFilter[i] = slopeFilter[i-1];
	}slopeFilter[0] = t;
	for(int i = 0;i<SF_LENGTH;i++)
	{
		res += slopeFilter[i];
	}
	return (res/SF_LENGTH);
}

//float RC::Mouse_X_Speed(float Xmax)
//{
//	int16_t res;
//	if(fabs(this->shubiao.x) > Xmax)res = 0;
//	else res = this->SF(KalmanFilter(&KF_Mouse_X_Speed,(float)this->shubiao.x),
//										this->shubiao.SFX,0);
//	return (float)res;
//}

//float RC::Mouse_Y_Speed(float Ymax)
//{
//	int16_t res;
//	if(fabs(this->shubiao.y) > Ymax)res = 0;
//	else res = this->SF(KalmanFilter(&KF_Mouse_Y_Speed,(float)this->shubiao.y),
//										this->shubiao.SFY,0);
//	return (float)res;
//}

bool RC::verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
	uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = this->get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

uint16_t RC::get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}


#endif
/*******************************************************ADXRS290*******************************************************************/
#ifdef __SPI_H__
ADXRS290_StatusTypeDef ADXRS290::adxrs290_writeByte(uint8_t subAddress, uint8_t data)
{
	uint8_t t_buf[] = { subAddress ,data };
	this->ADXRS290_SPI_ON();
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 2, 999);
	this->ADXRS290_SPI_OFF();
	HAL_Delay(1);
	return ADXRS290_OK;
}

void ADXRS290::adxrs290_readBytes(uint8_t subAddress, uint8_t count, uint8_t* spi_rev_buf)
{
	uint8_t t_buf[] = { (uint8_t)(subAddress | 0x80) };
	this->ADXRS290_SPI_ON();
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
	HAL_SPI_Receive(hspi, (uint8_t*)spi_rev_buf, count, 999);
	this->ADXRS290_SPI_OFF();
}

uint8_t ADXRS290::adxrs290_readByte(uint8_t subAddress)
{
	uint8_t data,t_buf[] = { (uint8_t)(subAddress | 0x80) };
	this->ADXRS290_SPI_ON();
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
	HAL_SPI_Receive(hspi, (uint8_t*)&data, 1, 999);
	this->ADXRS290_SPI_OFF();
	return data;
}

ADXRS290_StatusTypeDef	ADXRS290::Init(uint8_t hpf_corner,uint8_t odr_lpf)
{
	uint8_t ADXRS290_check[32];
	
	while(strcmp("put the Update function in EXTI Rising interrupt",string_check_290) != 0);
	
	HAL_Delay(10);
	adxrs290_writeByte(ADXRS290_POWER_CTL,0X00);
	adxrs290_writeByte(ADXRS290_DATA_READY,0X01);
	adxrs290_writeByte(ADXRS290_Filter,hpf_corner<<4|odr_lpf);
	adxrs290_writeByte(ADXRS290_POWER_CTL,0x02);
	
	adxrs290_readBytes(ADXRS290_ADI_ID,4,ADXRS290_check);
	if(!((ADXRS290_check[0]==0XAD)&(ADXRS290_check[1]==0X1D)&(ADXRS290_check[2]==0x92)&(ADXRS290_check[3]==0x09)))
		return ADXRS290_ID_ERROR;
	
	adxrs290_readBytes(ADXRS290_POWER_CTL,3,ADXRS290_check);
	if(!((ADXRS290_check[0]==0X02)&(ADXRS290_check[1]==((hpf_corner<<4)|(odr_lpf<<0)))&(ADXRS290_check[2]==0X01)))
		return ADXRS290_SET_ERROR;
	
	HAL_Delay(1);
	adxrs290_readBytes(ADXRS290_DATAX0,4,ADXRS290_check);
	return ADXRS290_OK;
}

void	ADXRS290::adxrs290_update(void)
{
	uint8_t ADXRS290_rev_buf[32];
	this->ADXRS290_SPI_ON();
	adxrs290_readBytes(ADXRS290_DATAX0,4,ADXRS290_rev_buf);
	this->ADXRS290_SPI_OFF();
	int16_t temp_x = (int16_t)(ADXRS290_rev_buf[1]<<8|ADXRS290_rev_buf[0]<<0);
	int16_t temp_y = (int16_t)(ADXRS290_rev_buf[3]<<8|ADXRS290_rev_buf[2]<<0);

	if(this->sensor_data_X.dev_count<=SELF_TEST_NUM_290)
	{
		this->sensor_data_X.bias = (this->sensor_data_X.dev_count * this->sensor_data_X.bias + temp_x) / (this->sensor_data_X.dev_count + 1);
		this->sensor_data_X.v = ((float)temp_x - this->sensor_data_X.bias)/(((uint16_t)0x7fff)/200);
		this->sensor_data_X.v_nonoise = this->sensor_data_X.v;
		this->sensor_data_X.dev_count++;
	}
	else
	{
		if(this->sensor_data_X.bias>10||this->sensor_data_X.bias<-10)this->sensor_data_X.bias=0;
		this->sensor_data_X.v = ((float)temp_x - this->sensor_data_X.bias)/(((uint16_t)0x7fff)/200);
		if(this->sensor_data_X.v<-DEAD_ZONE_290||this->sensor_data_X.v>DEAD_ZONE_290)this->sensor_data_X.v_nonoise=this->sensor_data_X.v;
		else this->sensor_data_X.v_nonoise=0;
		this->sensor_data_X.theta_euler += this->sensor_data_X.v_nonoise * 0.0020202f;
	}

	if(this->sensor_data_Y.dev_count<=SELF_TEST_NUM_290)
	{
		this->sensor_data_Y.bias = (this->sensor_data_Y.dev_count * this->sensor_data_Y.bias + temp_y) / (this->sensor_data_Y.dev_count + 1);
		this->sensor_data_Y.v = ((float)temp_y - this->sensor_data_Y.bias)/(((uint16_t)0x7fff)/200);
		this->sensor_data_Y.v_nonoise = this->sensor_data_Y.v;
		this->sensor_data_Y.dev_count++;
	}
	else
	{
		if(this->sensor_data_Y.bias>10||this->sensor_data_Y.bias<-10)this->sensor_data_Y.bias=0;
		this->sensor_data_Y.v = ((float)temp_y - this->sensor_data_Y.bias)/(((uint16_t)0x7fff)/200);
		if(this->sensor_data_Y.v<-DEAD_ZONE_290||this->sensor_data_Y.v>DEAD_ZONE_290)this->sensor_data_Y.v_nonoise=this->sensor_data_Y.v;
		else this->sensor_data_Y.v_nonoise=0;
		this->sensor_data_Y.theta_euler += this->sensor_data_Y.v_nonoise * 0.0020202f;
	}
}		
#endif


/*******************************************************ADXRS453*******************************************************************/
#ifdef __SPI_H__

ADXRS453_StatusTypeDef	ADXRS453::Init()
{
	int16_t ADXRS453_RATE;
	ADXRS453_StatusTypeDef error;
	
	while(strcmp("put the Update function in 485hz interrupt",string_check) != 0);

	HAL_Delay(100);
	error=this->sensor(1,(int16_t*)&ADXRS453_RATE);
//	if(error!=ADXRS453_OK)return error;
	HAL_Delay(60);
	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
//	if(error!=ADXRS453_OK)return error;
	HAL_Delay(60);
	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
//	if(error!=ADXRS453_OK)return error;
	HAL_Delay(60);
	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
//	if(error!=ADXRS453_OK)return error;
	HAL_Delay(60);
	error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
	if(error!=ADXRS453_OK)return error;
	HAL_Delay(60);
	HAL_TIM_Base_Start_IT(this->htim);
	return error;
}

ADXRS453_StatusTypeDef	ADXRS453::adxrs453_update(void)
{
		int16_t ADXRS453_RATE;
		ADXRS453_StatusTypeDef error;

		if(this->sensor_data.dev_count<this->SELF_TEST_NUM)
		{
			error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
			if(error==ADXRS453_OK)
			{
				if (this->sensor_data.bias > this->sensor_data.offset_max)
					this->sensor_data.offset_max = this->sensor_data.bias;
				if (this->sensor_data.bias < this->sensor_data.offset_min)
					this->sensor_data.offset_min = this->sensor_data.bias;
				
				if(this->sensor_data.offset_max - this->sensor_data.offset_min > 75)
				{
					this->sensor_data.dev_count=0;
				}
				
				if(this->sensor_data.dev_count==0)
				{
					this->sensor_data.bias = 0;
					this->sensor_data.offset_max = 0;
					this->sensor_data.offset_min = 0;
				}
				this->sensor_data.bias = (this->sensor_data.dev_count * this->sensor_data.bias + ADXRS453_RATE) / (this->sensor_data.dev_count + 1);
				this->sensor_data.v = ((float)ADXRS453_RATE - this->sensor_data.bias) * 0.0125f;
				this->sensor_data.v_nonoise = this->sensor_data.v;
				this->sensor_data.dev_count++;
			}
		}
		else
		{
			if(this->sensor_data.bias>300||this->sensor_data.bias<-300)this->sensor_data.bias=0;
			error=this->sensor(0,(int16_t*)&ADXRS453_RATE);
			if(error==ADXRS453_OK)
			{		
				this->sensor_data.v = ((float)ADXRS453_RATE - this->sensor_data.bias) * 0.0125f;
				if(this->sensor_data.v<-this->DEAD_ZONE||this->sensor_data.v>this->DEAD_ZONE)this->sensor_data.v_nonoise=this->sensor_data.v;
				else this->sensor_data.v_nonoise=0;
				this->sensor_data.theta_euler += this->sensor_data.v_nonoise * 0.0020202f;
				this->sensor_data.calibration=1;
			}
		}
		return error;
}

ADXRS453_StatusTypeDef ADXRS453::sensor(bool CHK,int16_t* date)
{
	uint32_t temp=0x20000000|(CHK<<1);
	
	temp=this->TransmitReceive(temp);

	if((temp&(0x01<<28))!=(this->odd_check(temp&0xEFFF0000)<<28))
		return ADXRS453_P0_ERROR;
	else if((temp&0x00000001)!=(this->odd_check(temp&0xFFFFFFFE)<<0))
		return ADXRS453_P1_ERROR;
	else if((temp&(0x01<<1))!=0)return ADXRS453_CHK_ERROR;
	else if((temp&(0x01<<7))!=0)return ADXRS453_PLL_ERROR;
	else if((temp&(0x01<<6))!=0)return ADXRS453_Q_ERROR;
	else if((temp&(0x01<<5))!=0)return ADXRS453_NVM_ERROR;
	else if((temp&(0x01<<4))!=0)return ADXRS453_POR_ERROR;
	else if((temp&(0x01<<3))!=0)return ADXRS453_PWR_ERROR;
	else if((temp&(0x01<<2))!=0)return ADXRS453_CST_ERROR;
	
	else if((temp&0xEC000000)!=0x4000000)
	{
		if((temp&0xEFF80000)==0x0E000000)return ADXRS453_RW_ERROR;
		else if((temp&(0x01<<18))!=0)return ADXRS453_SPI_ERROR;
		else if((temp&(0x01<<17))!=0)return ADXRS453_RE_ERROR;
		else if((temp&(0x01<<16))!=0)return ADXRS453_DU_ERROR;
		else return ADXRS453_RW_ERROR;
	}
	
	*date=temp>>10;
	return ADXRS453_OK;
}
ADXRS453_StatusTypeDef ADXRS453::addread(uint8_t address,int16_t* date)
{
	uint32_t temp=0x80000000|(address<<17);
	temp=this->TransmitReceive(temp);
	
	if((temp&(0x01<<28))!=(this->odd_check(temp&0xEFFF0000)<<28))
		return ADXRS453_P0_ERROR;
	if((temp&0x00000001)!=(this->odd_check(temp&0xFFFFFFFE)<<0))
		return ADXRS453_P1_ERROR;
	
	if((temp&0xEF000000)!=0x4E000000)
	{
		if((temp&0xEF000000)==0x0E000000)return ADXRS453_RW_ERROR;
		else if((temp&(0x01<<18))!=0)return ADXRS453_SPI_ERROR;
		else if((temp&(0x01<<17))!=0)return ADXRS453_RE_ERROR;
		else if((temp&(0x01<<16))!=0)return ADXRS453_DU_ERROR;
		else return ADXRS453_RW_ERROR;
	}
	
	*date=temp>>5;
	return ADXRS453_OK;
}
uint32_t ADXRS453::TransmitReceive(uint32_t address)
{
	uint32_t temp=(address>>16)|(address<<16),result;
	temp |= this->odd_check(temp)<<16;
	
	this->SPI_ON();
	HAL_SPI_TransmitReceive(hspi, (uint8_t *)&temp, (uint8_t *)&result, 2,999);
	this->SPI_OFF();
	result= (result>>16)|(result<<16);
	return result;
}
bool ADXRS453::odd_check(uint32_t date)
{
	static const bool ParityTable256[256] = 
	{
	#   define P2(n) n, n^1, n^1, n
	#   define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
	#   define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
			P6(0), P6(1), P6(1), P6(0)
	};

	uint32_t check=date;
	check ^= check >> 16;
	check ^= check >> 8;
	return !ParityTable256[check & 0xff];
}

#endif


/*******************************************************BMI088*******************************************************************/
#ifdef __SPI_H__

void BMI088::BMI088_writeByte(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t data)
{
	uint8_t t_buf[] = { (uint8_t)(subAddress & 0x7F)};
	this->BMI088_SPI_ON(GPIOx,GPIO_Pin);
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
	
//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
//	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
	
	HAL_SPI_Transmit(hspi, &data, 1, 999);
	this->BMI088_SPI_OFF(GPIOx,GPIO_Pin);
}

void BMI088::BMI088_readBytes(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	uint8_t t_buf[] = { (uint8_t)(subAddress | 0x80) };
	this->BMI088_SPI_ON(GPIOx,GPIO_Pin);
	HAL_SPI_Transmit(hspi, (uint8_t*)t_buf, 1, 999);
	HAL_SPI_Receive(hspi, (uint8_t*)spi_rev_buf, len, 999);
	this->BMI088_SPI_OFF(GPIOx,GPIO_Pin);
}

void BMI088::BMI088_write_Acc(uint8_t subAddress, uint8_t data)
{
	BMI088_writeByte(this->CSB1_GPIOx,this->CSB1_GPIO_Pin,subAddress,data);
}
void BMI088::BMI088_write_Gyro(uint8_t subAddress, uint8_t data)
{
	BMI088_writeByte(this->CSB2_GPIOx,this->CSB2_GPIO_Pin,subAddress,data);
}

void BMI088::BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	uint8_t t_buf[64],temp;
	this->BMI088_readBytes(this->CSB1_GPIOx,this->CSB1_GPIO_Pin,subAddress,len+1,t_buf);
	for(temp=0;temp<len;temp++)
		spi_rev_buf[temp] = t_buf[temp+1];
}

void BMI088::BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf)
{
	this->BMI088_readBytes(this->CSB2_GPIOx,this->CSB2_GPIO_Pin,subAddress,len,spi_rev_buf);
}

void BMI088::set_zero(void)
{
	this->sensor_data={0};
}

float inVSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

BMI088_StatusTypeDef	BMI088::Init(void)
{
	uint8_t BMI088_rev_buf[32],loop_break=1,loop_count=0;
		BMI088_write_Acc(ACC_SOFTRESET,0xB6);   // 复位加速度计
		HAL_Delay(1);

		BMI088_read_Acc(ACC_CHIP_ID,1,BMI088_rev_buf);   // 初始化为SPI模式
		HAL_Delay(5);
		BMI088_read_Acc(ACC_CHIP_ID,1,BMI088_rev_buf);   // 确定ID无误
		if(BMI088_rev_buf[0]!=0x1E)  return BMI088_ACC_ID_ERROR;
		
		BMI088_write_Acc(ACC_PWR_CONF,0x00);		// 设置加速度计为正常模式
		HAL_Delay(5);
		BMI088_write_Acc(ACC_PWR_CTRL,0x04);		// 使能加速度计和温度计
		HAL_Delay(10);				// 加速度计启动延时
		
		BMI088_write_Acc(ACC_CONF,0x0A);				// 设置加速度计输出速率为 400Hz
		
		BMI088_write_Acc(ACC_RANGE,AccRange);				// 设置量程为 ±6g		
		
		HAL_Delay(2);
	
	BMI088_write_Gyro(GYRO_SOFTRESET,0xB6);  // 复位陀螺仪
	HAL_Delay(30);        // 陀螺仪启动延时
	BMI088_read_Gyro(GYRO_CHIP_ID,1,BMI088_rev_buf);  // 确定ID无误
	if(BMI088_rev_buf[0]!=0x0F) return BMI088_GYRO_ID_ERROR;

	BMI088_write_Gyro(GYRO_BANDWIDTH,0x02); // 设置陀螺仪输出速率为 1000Hz，过滤器带宽为???Hz
	BMI088_write_Gyro(GYRO_RANGE,GyroRange);

	BMI088_write_Gyro(GYRO_SELF_TEST,0x01);  // 陀螺仪自检
	
	while(loop_break)
	{
		BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);		// 自检
		if((BMI088_rev_buf[0]&0x02)==0x02)
		{
			loop_break=0;
		}
		loop_count++;
		if(loop_count>=40)			    // 自检超时，自检失败
		{ 
				this->set_zero();
			  return BMI088_SELFTEXT_ERROR;
		}
		HAL_Delay(10);
	}
	
	HAL_Delay(1);
	this->BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);
	if((BMI088_rev_buf[0]&0x04)!=0)  return BMI088_SELFTEXT_ERROR;
	
	if(this->AccRange==BMI088_ACC_RANGE_3) 		   this->AccRangsetting=0.0008974358974f;
	else if(this->AccRange==BMI088_ACC_RANGE_6)  this->AccRangsetting=0.00179443359375f;
	else if(this->AccRange==BMI088_ACC_RANGE_12) this->AccRangsetting=0.0035888671875f;
	else if(this->AccRange==BMI088_ACC_RANGE_24) this->AccRangsetting=0.007177734375f;	
	
	if(this->GyroRange==BMI088_GYRO_RANGE_2000) this->GyroResolution=16.384f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_1000) this->GyroResolution=32.768f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_500) this->GyroResolution=65.536f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_250) this->GyroResolution=131.072f;
	else if(this->GyroRange==BMI088_GYRO_RANGE_125) this->GyroResolution=262.144f;
	
	this->set_zero();
	HAL_TIM_Base_Start_IT(this->htim);
	this->low_pass_filter_init();
	return BMI088_OK;
	
}

void	BMI088::BMI088_update(void)
{
	uint8_t BMI088_rev_buf[32];
	int16_t temp=0;

	if(this->enable_acc)
	{
		BMI088_read_Acc(TEMP_MSB,2,BMI088_rev_buf);
		temp = (int16_t)((BMI088_rev_buf[1]&0xE0)|BMI088_rev_buf[0]<<8);
		temp >>= 5;
		this->sensor_data.temperature = temp*0.125+23;
			
		if(abs(this->sensor_data.temperature-this->last_temperature)>2 && this->filter_count_temperature<10) 
		{
			this->sensor_data.temperature = this->last_temperature;
			this->filter_count_temperature++;
		}
		else
		{
			this->Acc_Temperature_Offset = (this->sensor_data.temperature-25)*0.2;
			this->Gyro_Temperature_Offset = (this->sensor_data.temperature-25)*0.015;
			this->last_temperature = this->sensor_data.temperature;
			this->filter_count_temperature=0;
		}
		BMI088_read_Acc(ACC_X_LSB,6,BMI088_rev_buf);   // 单位 mg
    this->sensor_data.acc.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0])) * this->AccRangsetting;
		this->sensor_data.acc.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2])) * this->AccRangsetting;
		this->sensor_data.acc.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4])) * this->AccRangsetting;
	}

	BMI088_read_Gyro(GYRO_SELF_TEST,1,BMI088_rev_buf);
	if((BMI088_rev_buf[0]&0x10)==0x10)
	{		
		BMI088_read_Gyro(RATE_X_LSB,6,BMI088_rev_buf);  // 单位 °/s
		this->sensor_data.gyro.origin.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0]));  
		this->sensor_data.gyro.origin.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2]));
		this->sensor_data.gyro.origin.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4]));
		
//		this->sensor_data.gyro.LPF.x = this->low_pass_filter(this->sensor_data.gyro.origin.x);
//		this->sensor_data.gyro.LPF.y = this->low_pass_filter(this->sensor_data.gyro.origin.y);
//		this->sensor_data.gyro.LPF.z = this->low_pass_filter(this->sensor_data.gyro.origin.z);			
		
		if(this->sensor_data.calibration)
		{
			this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x - this->sensor_data.gyro.offset.x + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y - this->sensor_data.gyro.offset.y + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z - this->sensor_data.gyro.offset.z + this->Acc_Temperature_Offset;		
		}
		else
		{
			this->sensor_data.gyro.calibration.x = this->sensor_data.gyro.LPF.x + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.y = this->sensor_data.gyro.LPF.y + this->Acc_Temperature_Offset;
			this->sensor_data.gyro.calibration.z = this->sensor_data.gyro.LPF.z + this->Acc_Temperature_Offset;	
		}	
				 
		if(this->sensor_data.runningTimes < this->SELF_TEST_NUM)
	  {
			if (this->sensor_data.gyro.offset_max.x - this->sensor_data.gyro.offset_min.x > 150 ||
					this->sensor_data.gyro.offset_max.y - this->sensor_data.gyro.offset_min.y > 150 ||
					this->sensor_data.gyro.offset_max.z - this->sensor_data.gyro.offset_min.z > 150)
			{
				this->sensor_data.runningTimes = 0;
			}	
			if(this->sensor_data.runningTimes == 0)
			{
				this->sensor_data.gyro.origin.x = 0;
				this->sensor_data.gyro.origin.y = 0;
				this->sensor_data.gyro.origin.z = 0;
				
				 this->sensor_data.gyro.dynamicSum.x = 0;
				 this->sensor_data.gyro.dynamicSum.y = 0;
				 this->sensor_data.gyro.dynamicSum.z = 0;				
				
				 this->sensor_data.gyro.offset_max.x = -32768;
				 this->sensor_data.gyro.offset_max.y = -32768;
				 this->sensor_data.gyro.offset_max.z = -32768;
				 this->sensor_data.gyro.offset_min.x = 32768;
				 this->sensor_data.gyro.offset_min.y = 32768;
				 this->sensor_data.gyro.offset_min.z = 32768;
			}		
			
			if (this->sensor_data.gyro.origin.x > this->sensor_data.gyro.offset_max.x)
				this->sensor_data.gyro.offset_max.x = this->sensor_data.gyro.origin.x;
			if (this->sensor_data.gyro.origin.y > this->sensor_data.gyro.offset_max.y)
				this->sensor_data.gyro.offset_max.y = this->sensor_data.gyro.origin.y;
			if (this->sensor_data.gyro.origin.z > this->sensor_data.gyro.offset_max.z)
				this->sensor_data.gyro.offset_max.z = this->sensor_data.gyro.origin.z;

			if (this->sensor_data.gyro.origin.x < this->sensor_data.gyro.offset_min.x)
				this->sensor_data.gyro.offset_min.x = this->sensor_data.gyro.origin.x;
			if (this->sensor_data.gyro.origin.y < this->sensor_data.gyro.offset_min.y)
				this->sensor_data.gyro.offset_min.y = this->sensor_data.gyro.origin.y;
			if (this->sensor_data.gyro.origin.z < this->sensor_data.gyro.offset_min.z)
				this->sensor_data.gyro.offset_min.z = this->sensor_data.gyro.origin.z;
				
			this->sensor_data.gyro.dynamicSum.x += this->sensor_data.gyro.origin.x;
			this->sensor_data.gyro.dynamicSum.y += this->sensor_data.gyro.origin.y;
			this->sensor_data.gyro.dynamicSum.z += this->sensor_data.gyro.origin.z;

			this->sensor_data.runningTimes++;	
   	}	
		else
		{				
			this->sensor_data.calibration=1;
	  }
		
		this->sensor_data.gyro.offset.x = (float)(this->sensor_data.gyro.dynamicSum.x) / this->sensor_data.runningTimes;
		this->sensor_data.gyro.offset.y = (float)(this->sensor_data.gyro.dynamicSum.y) / this->sensor_data.runningTimes;
		this->sensor_data.gyro.offset.z = (float)(this->sensor_data.gyro.dynamicSum.z) / this->sensor_data.runningTimes;	
		
		this->sensor_data.gyro.dps.x = this->sensor_data.gyro.calibration.x / this->GyroResolution;
		this->sensor_data.gyro.dps.y = this->sensor_data.gyro.calibration.y / this->GyroResolution;
		this->sensor_data.gyro.dps.z = this->sensor_data.gyro.calibration.z / this->GyroResolution;
		
		if(abs(this->sensor_data.gyro.dps.x-this->last_gyro_x)>7 && filter_count_x<2) this->filter_count_x++;
		else 
		{
			if(abs(this->sensor_data.gyro.dps.x) < abs(this->dead_zoom)) this->sensor_data.gyro.x_nonoise = 0;
			else this->sensor_data.gyro.x_nonoise = this->sensor_data.gyro.dps.x;
			this->last_gyro_x = this->sensor_data.gyro.dps.x;
			this->filter_count_x=0;
		}
		
		if(abs(this->sensor_data.gyro.dps.y-this->last_gyro_y)>7 && filter_count_y<2) this->filter_count_y++;
		else 
		{
			if(abs(this->sensor_data.gyro.dps.y) < abs(this->dead_zoom)) this->sensor_data.gyro.y_nonoise = 0;
			else this->sensor_data.gyro.y_nonoise = this->sensor_data.gyro.dps.y;
			this->last_gyro_y = this->sensor_data.gyro.dps.y;
			this->filter_count_y=0;
		}
		
		if(abs(this->sensor_data.gyro.dps.z-this->last_gyro_z)>7 && filter_count_z<2) this->filter_count_z++;
		else 
		{
			if(abs(this->sensor_data.gyro.dps.z) < abs(this->dead_zoom)) this->sensor_data.gyro.z_nonoise = 0;
			else this->sensor_data.gyro.z_nonoise = this->sensor_data.gyro.dps.z;
			this->last_gyro_z = this->sensor_data.gyro.dps.z;
			this->filter_count_z=0;
		}
		
		this->sensor_data.mang.x += this->sensor_data.gyro.x_nonoise*0.001f;
		this->sensor_data.mang.y += this->sensor_data.gyro.y_nonoise*0.001f;
		this->sensor_data.mang.z += this->sensor_data.gyro.z_nonoise*0.001f;
	}
}

void	BMI088::BMI088_New_update(void)
{
		uint8_t BMI088_rev_buf[32];
		int16_t temp=0;
		
		BMI088_read_Acc(ACC_X_LSB,6,BMI088_rev_buf);
		this->sensor_data.acc.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0]));
		this->sensor_data.acc.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2]));
		this->sensor_data.acc.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4]));
//		this->sensor_data.acc.LPF_x = this->low_pass_filter(this->sensor_data.acc.x);
//		this->sensor_data.acc.LPF_y = this->low_pass_filter(this->sensor_data.acc.y);
//		this->sensor_data.acc.LPF_z = this->low_pass_filter(this->sensor_data.acc.z);	
		BMI088_read_Gyro(RATE_X_LSB,6,BMI088_rev_buf);
		this->sensor_data.gyro.origin.x = ((int16_t)(BMI088_rev_buf[1]<<8|BMI088_rev_buf[0]));  
		this->sensor_data.gyro.origin.y = ((int16_t)(BMI088_rev_buf[3]<<8|BMI088_rev_buf[2]));
		this->sensor_data.gyro.origin.z = ((int16_t)(BMI088_rev_buf[5]<<8|BMI088_rev_buf[4]));
//		this->sensor_data.gyro.LPF.x = this->low_pass_filter(this->sensor_data.gyro.origin.x);
//		this->sensor_data.gyro.LPF.y = this->low_pass_filter(this->sensor_data.gyro.origin.y);
//		this->sensor_data.gyro.LPF.z = this->low_pass_filter(this->sensor_data.gyro.origin.z);	
			
}

/*四元数↓*/


/*全局变量 初始数据*/




/*姿态解算常量*/

/*自检采样次数*/

#define CALIBRATE_TIMES 3000 		//校准的次数

/*姿态解算宏定义及变量*/

#define delta_T  		0.001f  		//5ms计算一次
#define PI 				3.1415926535			//圆周率
#define new_weight 	1.0f 			//新数据权重
#define old_weight 	0.0f 			//旧数据权重
#define ACC_CONVER 	2048.f
#define GYRO_CONVER 16.4f

float I_ex, I_ey, I_ez;					//误差积分

quaterInfo_t Q_info = {1,0,0,0}; 	//全局四元数





float param_Kp = 0.17f;					//加速度计（磁力计）的收敛速率比例增益0.17
float param_Ki = 0.00f;					//陀螺仪收敛速率的积分增益 0.01

float icm_values[10];

uint8_t calibrationState;



/*算法部分*/

/*快速平方根（快速计算浮点数的倒数平方根）*/


/*对加速度计数据一阶低通滤波，对陀螺仪数据转成弧度每秒(2000dps)*/

void BMI088::getValues(void) 
{	
	//如果校准成功，则使原始值减去零漂值得到校准值
	if(calibrationState)
	{
		
		this->acc.calibration.data[0] = this->sensor_data.acc.x - this->acc.offset.data[0];
		this->acc.calibration.data[1] = this->sensor_data.acc.y - this->acc.offset.data[1];
		this->acc.calibration.data[2] = this->sensor_data.acc.z - this->acc.offset.data[2];
		this->gyro.calibration.data[0] = this->sensor_data.gyro.origin.x - this->gyro.offset.data[0];
		this->gyro.calibration.data[1] = this->sensor_data.gyro.origin.y - this->gyro.offset.data[1];
		this->gyro.calibration.data[2] = this->sensor_data.gyro.origin.z - this->gyro.offset.data[2];

	}
	//否则先暂时让校准值等于原始值
	else
	{
		this->acc.calibration.data[0] = this->sensor_data.acc.x;
		this->acc.calibration.data[1] = this->sensor_data.acc.y;
		this->acc.calibration.data[2] = this->sensor_data.acc.z;
		this->gyro.calibration.data[0] = this->sensor_data.gyro.origin.x;
		this->gyro.calibration.data[1] = this->sensor_data.gyro.origin.y;
		this->gyro.calibration.data[2] = this->sensor_data.gyro.origin.z;
	}
	
	//如果运行次数小于宏定义的校准次数则
	if(this->acc.runningTimes < CALIBRATE_TIMES)
	{	
		//如果运行次数等于0则初始化零漂值的最大和最小值
		//并且初始化原始值和校准时的求和累加值
		if(this->acc.runningTimes == 0)
		{
			this->sensor_data.acc.x = 0;
			this->sensor_data.acc.y = 0;
			this->sensor_data.acc.z = 0;
			this->sensor_data.gyro.origin.x = 0;
			this->sensor_data.gyro.origin.y = 0;
			this->sensor_data.gyro.origin.z = 0;
			
			this->acc.dynamicSum.data[0] = 0;
			this->acc.dynamicSum.data[1] = 0;
			this->acc.dynamicSum.data[2] = 0;
			this->gyro.dynamicSum.data[0] = 0;
			this->gyro.dynamicSum.data[1] = 0;
			this->gyro.dynamicSum.data[2] = 0;
			
			this->acc.offset_max.data[0] = -32768;
			this->acc.offset_max.data[1] = -32768;
			this->acc.offset_max.data[2] = -32768;
			this->acc.offset_min.data[0] = 32767;
			this->acc.offset_min.data[1] = 32767;
			this->acc.offset_min.data[2] = 32767;
			this->gyro.offset_max.data[0] = -32768;
			this->gyro.offset_max.data[1] = -32768;
			this->gyro.offset_max.data[2] = -32768;
			this->gyro.offset_min.data[0] = 32767;
			this->gyro.offset_min.data[1] = 32767;
			this->gyro.offset_min.data[2] = 32767;
		}
		
		//如果有一个原始值是范围内的正常的值（-32767~32767）
		//则直接替换这个最大/最小值
		if (this->sensor_data.acc.x > this->acc.offset_max.data[0])
			this->acc.offset_max.data[0] = this->sensor_data.acc.x;
		if (this->sensor_data.acc.y > this->acc.offset_max.data[1])
			this->acc.offset_max.data[1] = this->sensor_data.acc.y;
		if (this->sensor_data.acc.z > this->acc.offset_max.data[2])
			this->acc.offset_max.data[2] = this->sensor_data.acc.z;
		if (this->sensor_data.acc.x < this->acc.offset_min.data[0])
			this->acc.offset_min.data[0] = this->sensor_data.acc.x;
		if (this->sensor_data.acc.y < this->acc.offset_min.data[1])
			this->acc.offset_min.data[1] = this->sensor_data.acc.y;
		if (this->sensor_data.acc.z < this->acc.offset_min.data[2])
			this->acc.offset_min.data[2] = this->sensor_data.acc.z;
		
		if (this->sensor_data.gyro.origin.x > this->gyro.offset_max.data[0])
			this->gyro.offset_max.data[0] = this->sensor_data.gyro.origin.x;
		if (this->sensor_data.gyro.origin.y > this->gyro.offset_max.data[1])
			this->gyro.offset_max.data[1] = this->sensor_data.gyro.origin.y;
		if (this->sensor_data.gyro.origin.z > this->gyro.offset_max.data[2])
			this->gyro.offset_max.data[2] = this->sensor_data.gyro.origin.z;
		if (this->sensor_data.gyro.origin.x < this->gyro.offset_min.data[0])
			this->gyro.offset_min.data[0] = this->sensor_data.gyro.origin.x;
		if (this->sensor_data.gyro.origin.y < this->gyro.offset_min.data[1])
			this->gyro.offset_min.data[1] = this->sensor_data.gyro.origin.y;
		if (this->sensor_data.gyro.origin.z < this->gyro.offset_min.data[2])
			this->gyro.offset_min.data[2] = this->sensor_data.gyro.origin.z;
	
		this->acc.dynamicSum.data[0] += this->sensor_data.acc.x;
		this->acc.dynamicSum.data[1] += this->sensor_data.acc.y;
		this->acc.dynamicSum.data[2] += this->sensor_data.acc.z;
		
		this->gyro.dynamicSum.data[0] += this->sensor_data.gyro.origin.x;
		this->gyro.dynamicSum.data[1] += this->sensor_data.gyro.origin.y;
		this->gyro.dynamicSum.data[2] += this->sensor_data.gyro.origin.z;
		
		this->acc.runningTimes++;
		
		//如果误差值过大则重新开始计算运行次数
		if(this->gyro.offset_max.data[0] - this->gyro.offset_min.data[0] > 75 ||
		   this->gyro.offset_max.data[1] - this->gyro.offset_min.data[1] > 75 ||
		   this->gyro.offset_max.data[2] - this->gyro.offset_min.data[2] > 75)
		{			
			this->acc.runningTimes = 0;			
		}
	}
	
	else
	{
		
		calibrationState = 1;
		this->acc.offset.data[0] = (float)this->acc.dynamicSum.data[0] / this->acc.runningTimes;
		this->acc.offset.data[1] = (float)this->acc.dynamicSum.data[1] / this->acc.runningTimes;
		this->acc.offset.data[2] = (float)this->acc.dynamicSum.data[2] / this->acc.runningTimes;
		this->gyro.offset.data[0] = (float)(this->gyro.dynamicSum.data[0]) / this->acc.runningTimes;
		this->gyro.offset.data[1] = (float)(this->gyro.dynamicSum.data[1]) / this->acc.runningTimes;
		this->gyro.offset.data[2] = (float)(this->gyro.dynamicSum.data[2]) / this->acc.runningTimes;
		

	}
	
	int count;
    static double lastaccel[3]= {0,0,0};
	
	//获得校准之后给到values数组准备姿态融合和滤波
	this->Deal_acc.x = (((float)this->sensor_data.acc.x) * new_weight + lastaccel[0] * old_weight) / ACC_CONVER;
    this->Deal_acc.y = (((float)this->sensor_data.acc.y) * new_weight + lastaccel[1] * old_weight) / ACC_CONVER;
    this->Deal_acc.z = (((float)this->sensor_data.acc.z) * new_weight + lastaccel[2] * old_weight) / ACC_CONVER;
	
	//陀螺仪不需要考虑权重，只需要上面三行加速度需要依据权重处理数据
	this->Deal_gyro.x	= ((float)this->gyro.calibration.data[0]) * M_PI / 180 / GYRO_CONVER;
    this->Deal_gyro.y = ((float)this->gyro.calibration.data[1]) * M_PI / 180 / GYRO_CONVER;
    this->Deal_gyro.z = ((float)this->gyro.calibration.data[2]) * M_PI / 180 / GYRO_CONVER;
	
	//当前值赋值给下一次运算的上一个值
//	lastaccel[0] = this->Deal_acc.x;
//	lastaccel[1] = this->Deal_acc.y;
//	lastaccel[2] = this->Deal_acc.z;
	//没事别瞎加滤波
}

/*姿态解算融合，互补滤波算法*/

void BMI088::BMI088_AHRS(float gx, float gy, float gz, float ax, float ay, float az)
{
	float halfT = 0.5 * delta_T;	//半个周期
	float vx, vy, vz;    			//当前的机体坐标系上的重力单位向量
	float ex, ey, ez;    			//四元数计算值与加速度计测量值的误差
	
	//四元数
	float q0 = Q_info.q0;
	float q1 = Q_info.q1;
	float q2 = Q_info.q2;
	float q3 = Q_info.q3;
	
	//四元数乘积
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	
	//对加速度数据进行归一化 得到单位加速度
	float norm = inVSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	ex = ay * vz - az * vy;
	ey = az * vx - ax * vz;
	ez = ax * vy - ay * vx;
	
/*
	用叉乘误差来做PI修正陀螺零偏
	通过调节 param_Kp，param_Ki 两个参数
	可以控制加速度计修正陀螺仪积分姿态的速度
*/
	
	//积分误差缩放
	I_ex += delta_T * ex;  
	I_ey += delta_T * ey;
	I_ez += delta_T * ez;

	gx = gx+ param_Kp*ex + param_Ki*I_ex;
	gy = gy+ param_Kp*ey + param_Ki*I_ey;
	gz = gz+ param_Kp*ez + param_Ki*I_ez;
	
	//数据修正完毕
	
/*
	四元数微分方程，其中halfT为测量周期的1/2
	gx gy gz为陀螺仪角速度，以下都是已知量
	这里使用了一阶龙哥库塔求解四元数微分方程
	
	在线ChatGPT科普：
	一阶龙格-库塔方法是一种显式的单步法，
	常用于求解一阶常微分方程的初值问题。
	它通过迭代计算逐步逼近连续解。
	一阶龙格-库塔方法的基本思想是
	将微分方程的导数变化率在一个步长内进行估计，
	并使用这个估计值来更新解的近似值。
*/

	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;

	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	Q_info.q0 = q0 * norm;
	Q_info.q1 = q1 * norm;
	Q_info.q2 = q2 * norm;
	Q_info.q3 = q3 * norm;
}

/*把四元数转换成欧拉角*/

void BMI088::QuatToEulerAngles(void)
{
	this->getValues();
	this->BMI088_AHRS(this->Deal_gyro.x, this->Deal_gyro.y, this->Deal_gyro.z, this->Deal_acc.x, this->Deal_acc.y, this->Deal_acc.z);
	this->q0_t = Q_info.q0;
	this->q1_t = Q_info.q1;
	this->q2_t = Q_info.q2;
	this->q3_t = Q_info.q3;
	
    this->eulerAngle.pitch = asin(-2*this->q1_t*this->q3_t + 2*this->q0_t*this->q2_t) * 180/M_PI; 						// pitch
    this->eulerAngle.roll  = atan2(2*this->q2_t*this->q3_t + 2*this->q0_t*this->q1_t, -2*this->q1_t*this->q1_t - 2*this->q2_t*this->q2_t + 1) * 180/M_PI; // roll
    this->eulerAngle.yaw   = atan2(2*this->q1_t*this->q2_t + 2*this->q0_t*this->q3_t, -2*this->q2_t*this->q2_t - 2*this->q3_t*this->q3_t + 1) * 180/M_PI;	// yaw
	
	//更新当前的欧拉角
	this->nowAngle.pitch = this->eulerAngle.pitch;
	this->nowAngle.yaw = this->eulerAngle.yaw;
	this->nowAngle.roll = this->eulerAngle.roll;
}

/*过圈检测*/

void BMI088::BMI_CrossRound_err(void)
{
	//Yaw轴过圈
	if (this->nowAngle.yaw - this->lastAngle.yaw > 180) this->Round.roundYaw--;
	else if (this->nowAngle.yaw - this->lastAngle.yaw < -180) this->Round.roundYaw++;
	this->realAngle.yaw = this->Round.roundYaw * 360 + this->nowAngle.yaw;
	
	//Pitch轴过圈
	if (this->nowAngle.pitch - this->lastAngle.pitch > 180) this->Round.roundPitch--;
	else if (this->nowAngle.pitch - this->lastAngle.pitch < -180) this->Round.roundPitch++;
	this->realAngle.pitch = this->Round.roundPitch * 360 + this->nowAngle.pitch;
	
	//Roll轴过圈
	if (this->nowAngle.roll - this->lastAngle.roll > 180) this->Round.roundRoll--;
	else if (this->nowAngle.roll - this->lastAngle.roll < -180) this->Round.roundRoll++;
	this->realAngle.roll = this->Round.roundRoll * 360 + this->nowAngle.roll;
}

void BMI088::Analyse_speed(void)
{
	this->Anglespeed.pitch = (this->eulerAngle.pitch - this->lastAngle.pitch )/0.001f;
	this->Anglespeed.yaw 	= (this->eulerAngle.yaw   - this->lastAngle.yaw 	)/0.001f;
	this->Anglespeed.roll 	= (this->eulerAngle.roll  - this->lastAngle.roll 	)/0.001f;

		if(abs(this->Anglespeed.pitch-this->last_gyro_x)>7 && this->filter_count_x<2) this->filter_count_x++;
		else 
		{
			if(abs(this->Anglespeed.pitch) < abs(this->dead_zoom)) this->Anglespeed.Deal_pitch = 0;
			else this->Anglespeed.Deal_pitch = this->Anglespeed.pitch;
			this->last_gyro_x = this->Anglespeed.pitch;
			this->filter_count_x=0;
		}
		
		if(abs(this->Anglespeed.yaw-this->last_gyro_y)>7 && this->filter_count_y<2) this->filter_count_y++;
		else 
		{
			if(abs(this->Anglespeed.yaw) < abs(this->dead_zoom)) this->Anglespeed.Deal_yaw = 0;
			else this->Anglespeed.Deal_yaw = this->Anglespeed.yaw;
			this->last_gyro_y = this->Anglespeed.yaw;
			this->filter_count_y=0;
		}
		
		if(abs(this->Anglespeed.roll-this->last_gyro_z)>7 && this->filter_count_z<2) this->filter_count_z++;
		else 
		{
			if(abs(this->Anglespeed.roll) < abs(this->dead_zoom)) this->Anglespeed.Deal_roll = 0;
			else this->Anglespeed.Deal_roll = this->Anglespeed.roll;
			this->last_gyro_z = this->Anglespeed.roll;
			this->filter_count_z=0;
		}
	
}

/*ICM20602欧拉角解算*/

void BMI088::analyse(void)
{
	//更新上次的欧拉角
	this->lastAngle.pitch = this->eulerAngle.pitch;
	this->lastAngle.yaw 	= this->eulerAngle.yaw;
	this->lastAngle.roll 	= this->eulerAngle.roll;
	this->BMI088_New_update();
	this->QuatToEulerAngles();//四元数转换为欧拉角
	this->BMI_CrossRound_err();		//过圈检测
	this->Analyse_speed();
}
/*四元数↑*/

void  BMI088::selftext_error_reset(void)
{
	switch(this->selftext_reset_step)
	{
		case 0: 
			BMI088_write_Gyro(GYRO_SOFTRESET,0xB6);
			this->timer_1ms=0;
			this->selftext_reset_step=1;
		break;
		
		case 1:
			this->timer_1ms++;
			if(timer_1ms>30) this->selftext_reset_step=2;
		break;
		
		case 2:
			BMI088_write_Gyro(GYRO_BANDWIDTH,0x03);
			BMI088_write_Gyro(GYRO_RANGE,0x01);	
			this->selftext_reset_step=3;
		break;
		
		case 3:
			BMI088_write_Gyro(GYRO_SELF_TEST,0x01);
			this->timer_1ms=0;
			this->selftext_reset_step=4;
		break;
		
		case 4:
			this->timer_1ms++;
			if(timer_1ms>10) 
			{
				this->selftext_error_flag = 0;
				this->selftext_reset_step=0;
			}
		break;
		
		default: this->selftext_reset_step=0; break;
	}
}

/************************ 滤波器初始化 alpha *****************************/
void BMI088::low_pass_filter_init(void)
{
  float b = 2.0 * LPF_factor.pi * LPF_factor.CUTOFF_FREQ  * LPF_factor.SAMPLE_RATE;
  LPF_factor.alpha = b / (b + 1);
}

float BMI088::low_pass_filter(float value)
{
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    this->out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  this->out = this->out_last + LPF_factor.alpha * (value - this->out_last);
  this->out_last = this->out;

  return this->out;
}
#endif


/**************************************Vision_LPF***********************************************/
void Vision_LPF::Vision_Low_Pass_Filter_Init(void)
{
  this->b = 2.0 * this->pi * this->CUTOFF_FREQ  * this->SAMPLE_RATE;
  this->alpha = this->b / (this->b + 1);
}

float Vision_LPF::Vision_Low_Pass_Filter(float value)
{

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    this->out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  this->out = this->out_last + this->alpha * (value - this->out_last);
  this->out_last = this->out;

  return this->out;
}
/*一阶低通滤波*/



/****************************************PWM_MCL*********************************************/
#ifdef __TIM_H__

void MCL_snail::Init(void)
{
	while(strcmp("put the state_tick function in 50hz interrupt",this->string_check) != 0);
	HAL_TIM_PWM_Start(this->htim_z,this->Channel_z);
	HAL_TIM_PWM_Start(this->htim_y,this->Channel_y);
	set_speed(0,0);
	first_state=0;
}
void MCL_snail::Init_XC_Calibration(uint8_t speed_z_max,uint8_t speed_y_max)	// 行程校准
{
	while(strcmp("put the state_tick function in 50hz interrupt",this->string_check) != 0);
	HAL_TIM_PWM_Start(this->htim_z,this->Channel_z);
	HAL_TIM_PWM_Start(this->htim_y,this->Channel_y);

	set_speed(speed_z_max,speed_y_max);
	HAL_Delay(1000);
	set_speed(0,0);
	HAL_Delay(4000);
}
void MCL_snail::Init_Change_Steer(uint8_t dir,uint8_t speed_max)	// 切换转向
{
	while(strcmp("put the state_tick function in 50hz interrupt",this->string_check) != 0);
	
	if(dir) HAL_TIM_PWM_Start(this->htim_z,this->Channel_z);
	else HAL_TIM_PWM_Start(this->htim_y,this->Channel_y);
	
	set_speed(speed_max,speed_max);
	HAL_Delay(6000);
	set_speed(0,0);
	HAL_Delay(5000);
}
void MCL_snail::stop(void)
{
	if(first_state)
	{
		set_speed(0,0);
		this->time_20ms = 0,this->shoot_state_byte=0,this->run_stete = 0;
	}
}
void MCL_snail::run(uint8_t grade)
{
	if(first_state)
	{
		this->run_stete = 1;
		if(grade == 1)			this->set_speed(this->grade_1,this->grade_1-this->grade_1_error);
		else if(grade == 2)	this->set_speed(this->grade_2,this->grade_2-this->grade_2_error);
		else if(grade == 3)	this->set_speed(this->grade_3,this->grade_3-this->grade_3_error);
	}
}
HAL_StatusTypeDef MCL_snail::shoot_state(void)
{
	if(this->shoot_state_byte){this->time_20ms = 200/20;this->shoot_state_byte=0;return HAL_OK;}
	else return HAL_ERROR;
}
void MCL_snail::state_tick(TIM_HandleTypeDef *p)
{
	if(this->htim == p)
	{
		if(first_state)
		{
			if(this->run_stete)
			{
				this->time_20ms++;
				if(this->time_20ms >= 500/20)this->shoot_state_byte = 1;
			}
		}
		else 
		{
			this->time_20ms++;
			if(this->time_20ms >= 4000/20)first_state = 1,this->time_20ms = 0;
		}
	}
}
void	MCL_snail::set_speed(uint8_t speed_z,uint8_t speed_y)
{
	__HAL_TIM_SET_COMPARE(this->htim_z,this->Channel_z,speed_z+100);
	__HAL_TIM_SET_COMPARE(this->htim_y,this->Channel_y,speed_y+100);
}
#endif

/*************************************RGB**********************************************/
#ifdef __TIM_H__

//启动DMA-PWM传输。
void RGB_UI::WS_Load(void)
{
	HAL_TIM_PWM_Start_DMA(this->htim, this->Channel, (uint32_t *)send_Buf, NUM);
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)send_Buf, NUM);
}

//关闭所有灯：
void RGB_UI::WS_CloseAll(void)
{
	uint16_t i;
	for (i = 0; i < PIXEL_NUM * 24; i++)
		send_Buf[i] = 31; 											// 写入逻辑0的占空比
	for (i = PIXEL_NUM * 24; i < NUM; i++)
		send_Buf[i] = 0; 												// 占空比比为0，全为低电平
	WS_Load();
}

void RGB_UI::RGB_UI_Init(void)
{
	while(strcmp("put the Update function in 800kHz interrupt",string_check_rgb_ui) != 0);
	WS_CloseAll();
	HAL_Delay(100);
}


//设置所有灯为同一颜色：例如WS_WriteAll_RGB(0xFF,0,0)将所有灯设置为红色。
void RGB_UI::WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B)
{
	uint16_t i, j;
	uint8_t dat[24];
	// 将RGB数据进行转换
	for (i = 0; i < 8; i++)
	{
		dat[i] = ((n_G & 0x80) ? WS1 : WS0);
		n_G <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 8] = ((n_R & 0x80) ? WS1 : WS0);
		n_R <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 16] = ((n_B & 0x80) ? WS1 : WS0);
		n_B <<= 1;
	}
	
	for (i = 0; i < PIXEL_NUM; i++)
	{
		for (j = 0; j < 24; j++)
		{
			
			send_Buf[i * 24 + j] = dat[j];	
			
		}
	}
	for (i = PIXEL_NUM * 24; i < NUM; i++)
		{
			send_Buf[i] = 0;                       // 占空比比为0，全为低电平
		}
	
	  WS_Load();		
}
//设置单个灯的颜色（两个函数分别以不同格式设置颜色）：
uint32_t RGB_UI::WS281x_Color(uint8_t red, uint8_t green, uint8_t blue)
{
	return green << 16 | red << 8 | blue;
}

void RGB_UI::WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor)
{
	uint8_t i;
	if (n < PIXEL_NUM)
	{
		for (i = 0; i < 24; ++i)
		send_Buf[24 * n + i] = (((GRBColor << i) & 0X800000) ? WS1 : WS0);
	}
}

void RGB_UI::WS281x_SetPixelRGB(uint16_t n, uint8_t red, uint8_t green, uint8_t blue)
{

	uint8_t i;
	if (n < PIXEL_NUM)
	{
		for (i = 0; i < 24; ++i)
		send_Buf[24 * n + i] = (((WS281x_Color(red, green, blue) << i) & 0X800000) ? WS1 : WS0);
	}
	WS_Load();
}

#endif