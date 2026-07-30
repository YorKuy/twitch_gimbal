#ifndef __RM_LIB_H
#define __RM_LIB_H

#include "main.h"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
#include "math.h"
#include "can.h"			// 这行报错请屏蔽
#include "usart.h"		// 这行报错请屏蔽
#include "spi.h"			// 这行报错请屏蔽
#include "tim.h"			// 这行报错请屏蔽
#include "my_math.h"

/**************************************** USART **********************************************************/
#define USART_BUF_SIZE 		  128		// 数组大小，可修改
#define PRINTF_USART_HANDLE   huart3	// 串口号，可修改
extern uint8_t info_ubuf[USART_BUF_SIZE];
#define INFO(...) HAL_UART_Transmit (&PRINTF_USART_HANDLE,\
									(uint8_t *)info_ubuf,\
									sprintf((char *)info_ubuf,__VA_ARGS__),\
									0xffff)
									
/**************************************** C A N **********************************************************/
class USER_CAN
{
	public:
		CAN_HandleTypeDef* 	    hcan;
		CAN_TxHeaderTypeDef     TxHeader;
		CAN_RxHeaderTypeDef     RxHeader;
		uint8_t                 rx_buf[8];
		uint8_t                 tx_buf[8];
		bool					FIFO;
	
		void 					Init(uint16_t t,uint16_t x);
		HAL_StatusTypeDef		Send(uint16_t Id,uint8_t* pData);
		HAL_StatusTypeDef   Send_RM(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204);
		HAL_StatusTypeDef 	Receive(CAN_HandleTypeDef *hcan);
	
	  HAL_StatusTypeDef   RMD_Read_and_Write_Things	(uint16_t Id,uint16_t Order_Id);
	  HAL_StatusTypeDef   RMD_Write_PID(uint16_t Id,uint16_t Order_Id,uint16_t anglePidKp,uint16_t anglePidKi,
	                                    uint16_t speedPidKp,uint16_t speedPidKi,uint16_t iqPidKp,uint16_t iqPidKi);
	  HAL_StatusTypeDef   RMD_Write_ACCLE_to_RAM(uint16_t Id,int32_t Accel);
	  HAL_StatusTypeDef   RMD_Write_EncoderOffset_to_ROM(uint16_t Id,uint16_t EncoderOffset);//写入编码器值
  	HAL_StatusTypeDef   RMD_Iqcontrol_Motor(uint16_t Id,int16_t iqControl);
	  HAL_StatusTypeDef   RMD_Speedcontrol_Motor(uint16_t Id,int32_t angleControl);
	  HAL_StatusTypeDef   RMD_Speedcontrol_Motor(uint16_t Id,uint16_t Order_Id,uint16_t maxSpeed,int32_t angleControl);
		HAL_StatusTypeDef   RMD_Anglecontrol_Motor(uint16_t Id,uint16_t Order_Id,uint8_t spinDirection,uint16_t maxSpeed,uint16_t angleControl);	
	  HAL_StatusTypeDef 	RMD_Receive(CAN_HandleTypeDef *hcan);
	  HAL_StatusTypeDef Send_DAMIAO(uint16_t Id,int16_t M_201, int16_t M_202, int16_t M_203,int16_t M_204);
		
		USER_CAN(CAN_HandleTypeDef* p,bool fifo):hcan(p),FIFO(fifo){}
	private:
		uint32_t                TxMailbox;
		uint8_t 			    FreeTxNum;

};
/*************************************************** Mai Lun  ******************************************************/

typedef struct{
	float qy;
	float hy;
	float qz;
	float hz;
}ML_typedef;

class MOTOR_DiPan{
	public:
		ML_typedef 	ML;
	
		MOTOR_DiPan(void);
		void ML_Data_Deal(float lx,float ly,float lp,int MAX_rate);

};

/*************************************************** Quan Xiang Lun  ******************************************************/
#define QXL_PI       3.1415926535897932384626433832795f
typedef struct{
	float qy;
	float hy;
	float qz;
	float hz;
}	QXL_typedef;

class MOTOR_QXL_DiPan{
	public:
		ML_typedef 	QXL;
	
		MOTOR_QXL_DiPan(void);
		void QXL_Data_Deal(float lx,float ly,float lp,float angle,int MAX_rate);

};

/*************************************************** Duo Lun  ******************************************************/

#define M_PI       3.1415926535897932384626433832795f
#define COS_45     0.70710678118654752440084436210485f
#define RAD2MANG   1303.7972938088065906186957895476f

struct wheel_dir_and_weight
{
  bool dir;
  float speed;
  int yaogan_speed;
};
struct lun_xy{
  float x,y;
};

enum {
  QZ = 0,
  HZ,
  HY,
  QY
};

class RUDDER_DiPan
{
	public:
		int16_t g_angle_6020[4] = {0, 0,0,0};
    wheel_dir_and_weight g_wheel_3508[4] = {0,0,0,0};
		lun_xy QZ_xy,HZ_xy,HY_xy,QY_xy;
		uint16_t ZERO_0[4]  = {2682,4067,2803,4069};//qz,hz,hy,qy  //对角为头 每个舵轮都不一样，自己修改 //向前的方向
		uint16_t ZERO_45[4] = {1700,3100,5831,3055};//qz,hz,hy,qy  //灯条为头 每个舵轮都不一样，自己修改 //向前的方向
		#define Yaw_mid_of_0   4744//向前的方向
		#define Yaw_mid_of_45  5778//向前的方向
		uint16_t follow_chassic=Yaw_mid_of_0;//跟随角度
	  void  Not_Xiaotuoluo_Jie_Suan(float CH0, float CH1, int16_t CH2,int16_t SPEED_MAX);
		void Xiaotuoluo_jie_Suan(uint16_t Mang_yaw,int16_t CH0,int16_t CH1,int16_t CH2,int16_t SPEED_MAX);
		
	private:	
		
		float Get_Ch0_Ch1_Vector_Speed(int16_t CH0, int16_t CH1);
	  float  Get_Max_Speed(int16_t CH0, int16_t CH1, int16_t CH2);
	  float absf(float d0);
  	float Get_Max_float(float d0,float d1);
	  int16_t Get_Max_int16(int16_t d0,int16_t d1);
	
};
/****************************************** D B U S **********************************************************/
#define YK_SW_UP             	((uint16_t)1)
#define YK_SW_MID             ((uint16_t)3)
#define YK_SW_DOWN            ((uint16_t)2)

#define	YK_MODE_SW_C			((uint16_t)0)
#define	YK_MODE_SW_N			((uint16_t)1)
#define	YK_MODE_SW_S			((uint16_t)2)

#define KEY_PRESSED_W 			((uint16_t)0x01<<0)
#define KEY_PRESSED_S 			((uint16_t)0x01<<1)
#define KEY_PRESSED_A 			((uint16_t)0x01<<2)
#define KEY_PRESSED_D 			((uint16_t)0x01<<3)
#define KEY_PRESSED_SHIFT 	((uint16_t)0x01<<4)
#define KEY_PRESSED_CTRL 		((uint16_t)0x01<<5)
#define KEY_PRESSED_Q 			((uint16_t)0x01<<6)
#define KEY_PRESSED_E 			((uint16_t)0x01<<7)
#define KEY_PRESSED_R 			((uint16_t)0x01<<8)
#define KEY_PRESSED_F 			((uint16_t)0x01<<9)
#define KEY_PRESSED_G 			((uint16_t)0x01<<10)
#define KEY_PRESSED_Z 			((uint16_t)0x01<<11)
#define KEY_PRESSED_X 			((uint16_t)0x01<<12)
#define KEY_PRESSED_C 			((uint16_t)0x01<<13)
#define KEY_PRESSED_V 			((uint16_t)0x01<<14)
#define KEY_PRESSED_B 			((uint16_t)0x01<<15)
#define SF_LENGTH   10

#define mouse_KF  1

#define RC_CH_VALUE_OFFSET 1024

typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t v;
	uint8_t s1;
	uint8_t s2;
}yaogan_typedef;
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	float SFX[SF_LENGTH];
	float SFY[SF_LENGTH];	
	uint8_t press_l;
	uint8_t press_r;
}shubiao_typedef;

typedef struct __attribute__((packed))
{
    uint8_t sof_1;
    uint8_t sof_2;
    int64_t ch_0:11;
    int64_t ch_1:11;
    int64_t ch_2:11;
    int64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;

static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

class	RC
{
	public:
		UART_HandleTypeDef *	F_huart;
		UART_HandleTypeDef *	S_huart;
		uint8_t 				DR16_rx_buffer[25];
		
		yaogan_typedef 	yaogan;
		shubiao_typedef 	shubiao;
		uint16_t 				jianpan;
		
		yaogan_typedef 	DT16_yaogan;
		shubiao_typedef 	DT16_shubiao;
		uint16_t 				DT16_jianpan;
		
		remote_data_t			VT13_Data;
		uint16_t crc16_init = 0xffff;
		uint16_t	DT16_time_100ms;
		uint16_t	VT13_time_100ms;
	
	
		uint8_t 				VT13_rx_buffer[21];
		
		
		void DT16_feed_watchdog(void)
		{
			DT16_time_100ms=0;
		}
		void VT13_feed_watchdog(void)
		{
			VT13_time_100ms=0;
		}
		HAL_StatusTypeDef receive_run(void)
		{
			return HAL_UART_Receive_IT(this->F_huart,this->DR16_rx_buffer,25);
		}
		HAL_StatusTypeDef receive_refresh(void)
		{
			return HAL_UART_AbortReceive_IT(this->F_huart);
		}
		
		void 					DT16_Init(void);
		void 					VT13_Init(void);
		void					DT16_RxCplt_IRQHandler(void);
		void					VT13_RxCplt_IRQHandler(void);
		void 					DT16_set_zero(void);
		void 					VT13_set_zero(void);
		void 					DT16_data_deal(void);
		void 					VT13_data_deal(void);
		void 					DT16_watchdog_run(void);
		void 					VT13_watchdog_run(void);
		void					fill_data(void);
		void					can_receive_data_deal(uint8_t *buf);
		extKalman_t 		KF_Mouse_Y_Speed,KF_Mouse_X_Speed;
		uint8_t 			Pressed_Check(uint16_t key_value);	//按下状态 返回1
		float 				SF(float t,float *slopeFilter,float res);
		float 				Mouse_X_Speed(float Xmax);
		float 				Mouse_Y_Speed(float Ymax);
		bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
		uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
		
		RC(UART_HandleTypeDef *f,UART_HandleTypeDef *s):F_huart(f),S_huart(s){}
	
	private:
		uint8_t 	first;
		uint8_t		index;
		uint16_t 	delaycount[16];
		uint16_t 	last_jianpan;
	
		HAL_StatusTypeDef 		DT16_check_and_deal(void);
		HAL_StatusTypeDef 		VT13_check_and_deal(void);
};

// 状态定义
//#define	YK_MODE_SW_C			((uint16_t)0)
//#define	YK_MODE_SW_N			((uint16_t)1)
//#define	YK_MODE_SW_S			((uint16_t)2)

// 状态变化类型枚举
typedef enum {
    TRANS_NONE,     // 无变化
    C_N,         	// C -> N
    C_S,        	// C -> S
    N_C,         	// N -> C
    N_S,       		// N -> S
    S_C,        	// S -> C
    S_N        		// S -> N
} TransitionState;

// 状态检测类
class YKStateTransitionDetector {
public:
    // 构造函数：需要初始状态
    YKStateTransitionDetector(uint16_t initial_state) 
        : prev_state(initial_state) {}

    // 状态更新函数：返回状态变化类型
    TransitionState update(uint16_t current_state) {
        if (current_state == prev_state) {
            return TRANS_NONE; // 状态未变化
        }

        // 通过状态组合判断变化类型
        TransitionState transition = getTransitionType(prev_state, current_state);
        prev_state = current_state; // 更新历史状态
        return transition;
    }

private:
    uint16_t prev_state; // 保存上一次状态

    // 状态变化映射表
    TransitionState getTransitionType(uint16_t prev, uint16_t curr) {
        // UP 状态分支
        if (prev == YK_MODE_SW_C) {
            if (curr == YK_MODE_SW_N) return C_N;
            if (curr == YK_MODE_SW_S) return C_S;
        }
        // MID 状态分支
        else if (prev == YK_MODE_SW_N) {
            if (curr == YK_MODE_SW_C) return N_C;
            if (curr == YK_MODE_SW_S) return N_S;
        }
        // DOWN 状态分支
        else if (prev == YK_MODE_SW_S) {
            if (curr == YK_MODE_SW_C) return S_C;
            if (curr == YK_MODE_SW_N) return S_N;
        }
        return TRANS_NONE;
    }
};

/**************************************** ADXRS290 **********************************************************/
#ifdef __SPI_H__

#define ADXRS290_ADI_ID 		0x00
#define ADXRS290_MEMS_ID  		0x01
#define ADXRS290_DEV_ID 		0x02
#define ADXRS290_REV_ID  		0x03
#define ADXRS290_SN0  			0x04
#define ADXRS290_SN1 			0x05
#define ADXRS290_SN2 			0x06
#define ADXRS290_SN3 			0x07
#define ADXRS290_DATAX0 		0x08
#define ADXRS290_DATAX1 		0x09
#define ADXRS290_DATAY0 		0x0A
#define ADXRS290_DATAY1 		0x0B
#define ADXRS290_TEMP0 			0x0C
#define ADXRS290_TEMP1 			0x0D
#define ADXRS290_POWER_CTL 		0x10
#define ADXRS290_Filter 		0x11
#define ADXRS290_DATA_READY 	0x12

typedef struct gyro{
	float v;
	float v_nonoise;
	float theta_euler;
	float bias;
	uint32_t dev_count;
}ADXRS290_TYPEDEF;

typedef enum 
{
	ADXRS290_OK       		= 0x00U,
	ADXRS290_SET_ERROR    	= 0x01U,
	ADXRS290_ID_ERROR    	= 0x02U,
	ADXRS290_ERROR    		= 0x03U,
} ADXRS290_StatusTypeDef;


class ADXRS290
{
	public:
		SPI_HandleTypeDef *		hspi;
		GPIO_TypeDef *				GPIOx;
		uint16_t 							GPIO_Pin;

		ADXRS290_TYPEDEF		sensor_data_X;
		ADXRS290_TYPEDEF		sensor_data_Y;
	
		ADXRS290_StatusTypeDef		Init(uint8_t hpf_corner,uint8_t odr_lpf);
		void											adxrs290_update(void);
		ADXRS290_StatusTypeDef		adxrs290_writeByte(uint8_t subAddress, uint8_t data);
		uint8_t										adxrs290_readByte(uint8_t subAddress);
		void 											adxrs290_readBytes(uint8_t subAddress, uint8_t count, uint8_t* spi_rev_buf);
		
		ADXRS290(SPI_HandleTypeDef *q,GPIO_TypeDef *w,uint16_t e,uint16_t t,float y,const char *u):
		hspi(q),GPIOx(w),GPIO_Pin(e),SELF_TEST_NUM_290(t),DEAD_ZONE_290(y),string_check_290(u){}
		private:
		uint16_t 		SELF_TEST_NUM_290;
		float 			DEAD_ZONE_290;
		const char*		string_check_290;
		void ADXRS290_SPI_ON()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void ADXRS290_SPI_OFF()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}

};
#endif


/*****************************************ADXRS453*******************************************/
#ifdef __SPI_H__


typedef struct {
	float v;
	float v_nonoise;
	float theta_euler;
	float bias;
	float offset_v;
	float offset_max;
	float offset_min;
	uint32_t dev_count;
	uint8_t calibration;
}ADXRS453_TYPEDEF;
typedef enum 
{
	ADXRS453_OK       		= 0x00U,
	ADXRS453_RW_ERROR   	= 0x01U,
	ADXRS453_P0_ERROR   	= 0x02U,
	ADXRS453_P1_ERROR  		= 0x03U,
	ADXRS453_SPI_ERROR		= 0x04U,
	ADXRS453_RE_ERROR		= 0x05U,
	ADXRS453_DU_ERROR		= 0x06U,
	ADXRS453_PLL_ERROR		= 0x07U,
	ADXRS453_Q_ERROR		= 0x08U,
	ADXRS453_NVM_ERROR		= 0x09U,
	ADXRS453_POR_ERROR		= 0x0AU,
	ADXRS453_PWR_ERROR		= 0x0BU,
	ADXRS453_CST_ERROR		= 0x0CU,
	ADXRS453_CHK_ERROR		= 0x0DU,
	ADXRS453_ERROR
} ADXRS453_StatusTypeDef;

class ADXRS453
{
	public:
		SPI_HandleTypeDef *		hspi;
		GPIO_TypeDef *			GPIOx;
		uint16_t 				GPIO_Pin;
		TIM_HandleTypeDef *		htim;
		
		ADXRS453_TYPEDEF 		sensor_data;
		
		ADXRS453_StatusTypeDef	Init();
		ADXRS453_StatusTypeDef  adxrs453_update(void);
		ADXRS453_StatusTypeDef	sensor(bool CHK,int16_t* date);
		uint32_t 								TransmitReceive(uint32_t address);
		ADXRS453_StatusTypeDef 	addread(uint8_t address,int16_t* date);
	
		ADXRS453(SPI_HandleTypeDef *q,GPIO_TypeDef *w,uint16_t e,TIM_HandleTypeDef *r,uint16_t t,float y,const char *u):
		hspi(q),GPIOx(w),GPIO_Pin(e),htim(r),SELF_TEST_NUM(t),DEAD_ZONE(y),string_check(u){}
	
	private:
		uint16_t 		SELF_TEST_NUM;
		float 			DEAD_ZONE;
		const char	*	string_check;
		void SPI_ON()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void SPI_OFF()
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}
		bool 			odd_check(uint32_t date);
};
#endif


/**************************************BMI088************************************************/
#ifdef __SPI_H__

#define  ACC_CHIP_ID  		0x00
#define  ACC_ERR_REG			0X02
#define  ACC_STATUS				0X03
#define  ACC_X_LSB				0X12
#define  ACC_X_MSB				0X13
#define  ACC_Y_LSB				0X14
#define  ACC_Y_MSB				0X15
#define  ACC_Z_LSB				0X16
#define  ACC_Z_MSB				0X17
#define  SENSORTIME_0			0X18
#define  SENSORTIME_1			0X19
#define  SENSORTIME_2			0X1A
#define  ACC_INT_STAT_1		0X1D
#define  TEMP_MSB					0X22
#define  TEMP_LSB					0X23
#define  ACC_CONF					0X40
#define  ACC_RANGE				0X41
#define  INT1_IO_CTRL			0X53
#define  INT2_IO_CTRL			0X54
#define  INT_MAP_DATA			0X58
#define  ACC_SELF_TEST		0X6D
#define  ACC_PWR_CONF			0X7C
#define  ACC_PWR_CTRL			0X7D
#define  ACC_SOFTRESET		0X7E

#define  GYRO_CHIP_ID				0X00
#define  RATE_X_LSB					0X02
#define  RATE_X_MSB					0X03
#define  RATE_Y_LSB					0X04
#define  RATE_Y_MSB					0X05
#define  RATE_Z_LSB					0X06
#define  RATE_Z_MSB					0X07
#define  GYRO_INT_STAT_1		0X0A
#define  GYRO_RANGE					0X0F
#define  GYRO_BANDWIDTH			0X10
#define  GYRO_LPM1					0X11
#define  GYRO_SOFTRESET			0X14
#define  GYRO_INT_CTRL			0X15
#define  INT3_INT4_IO_CONF	0X16
#define  INT3_INT4_IO_MAP		0X18
#define  GYRO_SELF_TEST			0X3C
float inVSqrt(float x);
typedef struct {
	struct {
		float x;
		float y;
		float z;	
		float LPF_x;
		float LPF_y;
		float LPF_z;		
	} acc;
	
	struct {
		float x_nonoise;
		float y_nonoise;
		float z_nonoise;	
	struct {
			float x;
			float y;
			float z;
		} calibration;	
	struct {
			float x;
			float y;
			float z;
		} origin;			
	struct {
		float x;
		float y;
		float z;
	} dynamicSum;
	struct {
		float x;
		float y;
		float z;
	} offset;	
	struct {
		float x;
		float y;
		float z;
	} offset_max;	
	struct {
		float x;
		float y;
		float z;
	} offset_min;	
	struct {
		float x;
		float y;
		float z;
	} dps;		
	struct {
		float x;
		float y;
		float z;
	} LPF;			
	} gyro;
	
	struct {
		float x;
		float y;
		float z;
		float now_x;
		float now_y;
		float now_z;
		float last_x;
		float last_y;
		float last_z;	
		float Real_x;
		float Real_y;
		float Real_z;			
	} mang;
	
	uint16_t runningTimes;
	float temperature;
	uint8_t calibration;
	
}BMI088_TYPEDEF;

typedef enum 
{
	BMI088_OK       		  = 0x00U,
	BMI088_SET_ERROR      = 0x01U,
	BMI088_ACC_ID_ERROR	  = 0x02U,
	BMI088_GYRO_ID_ERROR  = 0x03U,
	BMI088_ERROR    		  = 0x04U,
	BMI088_SELFTEXT_ERROR = 0x05U,
} BMI088_StatusTypeDef;

typedef enum 
{
	BMI088_GYRO_RANGE_2000 = 0x00U,
	BMI088_GYRO_RANGE_1000 = 0x01U,
	BMI088_GYRO_RANGE_500  = 0x02U,
	BMI088_GYRO_RANGE_250  = 0x03U,
	BMI088_GYRO_RANGE_125  = 0x04U,
} BMI088_GyroRangeTypeDef;

typedef enum 
{
	BMI088_ACC_RANGE_3  =  0X00U,
	BMI088_ACC_RANGE_6  =  0X01U,
	BMI088_ACC_RANGE_12 =  0X02U,
	BMI088_ACC_RANGE_24 =  0X03U,
}BMI088_AccRangeTypeDef;

struct{
	float CUTOFF_FREQ = 500.0f;     //截止频率
	float SAMPLE_RATE = 0.001f;    //采样周期
	float pi = 3.1415926; //π
	float alpha;     //滤波系数
}LPF_factor;



typedef struct{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound_t;

/*四元数↓*/


typedef struct{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound;

/*二维float向量结构体*/

void BMI_CrossRound_err(void);

typedef struct vec2f
{
	float data[2];
} vec2f;

/*二维int16向量结构体*/

typedef struct vec2int16
{
	 short data[2];
} vec2int16;

/*三维float向量结构体*/

typedef struct vec3f
{
	float data[3];
} vec3f;

/*三维int16向量结构体*/

typedef struct vec3int16
{
	 short data[3];
} vec3int16;

/*四维float向量结构体*/

typedef struct vec4f
{
	float data[4];
}vec4f;

/*四维int16向量结构体*/

typedef struct vec4int16
{
	 short data[4];
}vec4int16;


/*结构体*/

typedef struct accdata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
	vec3f accValue;	   //加速度值，单位：m/s2
	vec3f dynamicSum;  //校准时求和计算
	uint16_t runningTimes;//运行次数
} accdata;

typedef struct gyrodata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
    vec3f dps;         //度每秒 
    vec3f radps;       //弧度每秒
    vec3f dynamicSum;  //校准时求和计算
} gyrodata;

typedef 	struct {
		float x;
		float y;
		float z;	
	} Deal_acc_t;
typedef 	struct {
		float x;
		float y;
		float z;	
	} Deal_gyro_t;
	
typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}eulerianAngles_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
	float Deal_pitch;
	float Deal_roll;
	float Deal_yaw;	
}Anglespeed_t;


/*四元数↑*/

class BMI088
{
	public:
		SPI_HandleTypeDef     *hspi;
		TIM_HandleTypeDef 		*htim;
		GPIO_TypeDef 					*CSB1_GPIOx,  *CSB2_GPIOx;
		uint16_t 							CSB1_GPIO_Pin,CSB2_GPIO_Pin;
		angleRound_t       	Round;
		BMI088_TYPEDEF		sensor_data;
		BMI088_StatusTypeDef	Init_High(void);		
		BMI088_StatusTypeDef	Init(void);
		Deal_acc_t Deal_acc;
		Deal_gyro_t Deal_gyro;
		accdata acc;
		gyrodata gyro;
	  eulerianAngles_t eulerAngle;  	//欧拉角
		eulerianAngles_t lastAngle;			//上一次的欧拉角
		eulerianAngles_t nowAngle;			//现在的欧拉角
		eulerianAngles_t realAngle;			//现在真实的欧拉角（已经叠加了圈数）	
		Anglespeed_t Anglespeed;
    float q0_t;
    float q1_t;
    float q2_t;
    float q3_t;
	float out_last;
	float out;
		quaterInfo_t Q_info = {1,0,0,0}; 	//全局四元数
		void									BMI088_write_Acc(uint8_t subAddress,uint8_t data);
		void									BMI088_write_Gyro(uint8_t subAddress,uint8_t data);
		void 									BMI088_read_Acc(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void 									BMI088_read_Gyro(uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void 									set_zero(void);
		void 									low_pass_filter_init(void);
		float 									low_pass_filter(float value);
		void    								BMI088_update(void);
		void									BMI088_New_update(void);
		void									BMI_Get_EulerAngle(void);
		void 									getValues(void);
		void 									QuatToEulerAngles(void);
		void 									analyse(void);
		void 									BMI_analyse(void);	
		void 									BMI_QuatToEulerAngles(void);
		void 									BMI_CrossRound(void);
		void 									BMI_CrossRound_err(void);
		void 									Analyse_speed(void);
		void 									BMI088_AHRS(float gx, float gy, float gz, float ax, float ay, float az);
		BMI088(SPI_HandleTypeDef *q,TIM_HandleTypeDef *t,GPIO_TypeDef *w1,uint16_t p1,GPIO_TypeDef *w2,uint16_t p2,uint16_t num,float dz,BMI088_GyroRangeTypeDef gyrorange,BMI088_AccRangeTypeDef accrange):
		hspi(q),htim(t),CSB1_GPIOx(w1),CSB1_GPIO_Pin(p1),CSB2_GPIOx(w2),CSB2_GPIO_Pin(p2),SELF_TEST_NUM(num),dead_zoom(dz),GyroRange(gyrorange),AccRange(accrange){}
	private:
		BMI088_GyroRangeTypeDef  GyroRange;  // 陀螺仪量程
  	BMI088_AccRangeTypeDef   AccRange;  // 加速度量程
		float 				GyroResolution;   // 陀螺仪分辨率
	  float 				AccRangsetting;   // 设置量程为
		float         Acc_Temperature_Offset=0,Gyro_Temperature_Offset=0;
		float         dead_zoom;
		uint8_t				enable_acc;
		uint16_t 			SELF_TEST_NUM;
		const char*		string_check_088;
		uint16_t      timer_1ms=0;
		uint8_t       selftext_error_flag=0,selftext_reset_step=0;
		float 				last_gyro_x,last_gyro_y,last_gyro_z,last_temperature,filter_count_x,filter_count_y,filter_count_z,filter_count_temperature;
	
		void BMI088_SPI_ON(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
		}
		void BMI088_SPI_OFF(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
		{
			HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
		}
		void	BMI088_writeByte(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t data);
		void 	BMI088_readBytes(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t subAddress, uint8_t len, uint8_t* spi_rev_buf);
		void  selftext_error_reset(void);
        void DWT_Init(void);
        float DWT_Get_time(void);
};


#endif

/**************************************Vision_LPF***********************************************/
class Vision_LPF
{
	public:
		void 									Vision_Low_Pass_Filter_Init(void);
		float 									Vision_Low_Pass_Filter(float value);
		Vision_LPF(float CF,float SR ,float PI):
		CUTOFF_FREQ(CF),SAMPLE_RATE(SR),pi(PI){}
		/*CF:截止频率 SR:采样频率*/
		float CUTOFF_FREQ;     //截止频率
		float SAMPLE_RATE;    //采样周期
		float pi; //π
		float alpha;     //滤波系数
		float b;
		float out_last = 0; //上一次滤波值
		float out;
};

/**************************************PWM_MCL***********************************************/
#ifdef __TIM_H__


class MCL_snail
{
	public:
		TIM_HandleTypeDef *		htim;
		TIM_HandleTypeDef *		htim_z;
		uint32_t 				Channel_z;
		TIM_HandleTypeDef *		htim_y;
		uint32_t 				Channel_y;
	
		void 					Init(void);
		void 					Init_XC_Calibration(uint8_t speed_z_max,uint8_t speed_y_max);
		void 					Init_Change_Steer(uint8_t dir,uint8_t speed_max);
		void 					stop(void);
		void 					run(uint8_t grade);
		HAL_StatusTypeDef 		shoot_state(void);
		void 					state_tick(TIM_HandleTypeDef *p);
		void					set_speed(uint8_t speed_z,uint8_t speed_y);
		
		MCL_snail(	TIM_HandleTypeDef *htim,
					TIM_HandleTypeDef *htim_z,uint32_t Channel_z,
					TIM_HandleTypeDef *htim_y,uint32_t Channel_y,
					uint8_t grade_1,uint8_t grade_2,uint8_t grade_3,
					uint8_t grade_1_error,uint8_t grade_2_error,uint8_t grade_3_error,
					const char *string_check
				 ):
					htim(htim),htim_z(htim_z),Channel_z(Channel_z),htim_y(htim_y),Channel_y(Channel_y),
					grade_1(grade_1),grade_2(grade_2),grade_3(grade_3),
					grade_1_error(grade_1_error),grade_2_error(grade_2_error),grade_3_error(grade_3_error),
					string_check(string_check){}
	
	private:
		uint8_t 		grade_1,grade_2,grade_3;
		uint8_t 		grade_1_error,grade_2_error,grade_3_error;
		const char 	*	string_check;
		uint8_t 		shoot_state_byte,run_stete;
		uint32_t 		time_20ms;
		uint8_t 		first_state;
};
#endif


/*************************************UD_check**********************************************/
typedef enum{
	UpDown_check_nothing,
	UpDown_check_falling,
	UpDown_check_rising
}UpDown_check_state;
class UpDown_check_class
{
	public:
		UpDown_check_class(bool initial_conditions):bit(initial_conditions){}
		UpDown_check_state updata(bool Condition)
		{
			if (((Condition) != 0) && ((bit & 1) == 0)) {
				bit |= 1;
				return UpDown_check_rising;
			}
			else if (!((Condition) != 0) && ((bit & 1) != 0)) {
				bit &= ~1;
				return UpDown_check_falling;
			}
			else return UpDown_check_nothing;
		}
	private:
		bool bit;
};		
		
/*************************************RGB**********************************************/
#ifdef __TIM_H__

#define PIXEL_NUM  5   										//灯珠数
#define NUM 			 (24*PIXEL_NUM + 300)    // Reset 280us（复位时间） / 1.25us = 224   NUM的值为单个灯珠的位宽（24）* 灯珠数量（PIXEL_NUM）+ 复位脉冲数    1/84M = 11.9ns
#define WS1  			 75    //重装值105
#define WS0  			 30

class RGB_UI
{
		public:
			TIM_HandleTypeDef *		htim;
			uint32_t 				Channel;
	  	void RGB_UI_Init(void);
			void WS_Load(void);
			void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);
			void WS_CloseAll(void);
			void WS281x_SetPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);

		RGB_UI(TIM_HandleTypeDef *htim,uint32_t Channel,const char *string_check_rgb_ui):
				htim(htim),Channel(Channel),string_check_rgb_ui(string_check_rgb_ui){}
		private:
			uint16_t send_Buf[NUM];
			const char*		string_check_rgb_ui;
			uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue);
			void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);
		
};
#endif
#endif
