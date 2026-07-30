#ifndef __RMD_H__
#define __RMD_H__

#include "RM_Lib.h"
typedef struct{

	uint16_t anglePidKp;
	uint16_t anglePidKi;
	uint16_t speedPidKp;
	uint16_t speedPidKi;
  uint16_t iqPidKp;
	uint16_t iqPidKi;	
	int32_t  Accel;
	uint16_t encoder;          //编码器位置     0~65535
	uint16_t encoderRaw;       //编码器原始位置 0~65535
	uint16_t encoderOffset;	   //编码器零偏     0~65535
	int64_t motorAngle;
  uint16_t circleAngle;		
	int8_t temperature;
	uint16_t voltage;
	uint8_t eerorState;
  int16_t iq;
	int16_t speed;
	uint16_t now_encoder;  //编码器位置值	
	int16_t iA;
	int16_t iB;
	int16_t iC;	
	int16_t iqControl;		
  int32_t speedControl;
	int32_t angleControl;
		
}RMD_typedef;

class MOTOR_RMD{
	public:
		const uint16_t ID;	// 电机反馈ID
		USER_CAN* 	can_rev;
	  RMD_typedef  RMD_X;
	
    HAL_StatusTypeDef   RMD_update(void);
		MOTOR_RMD(const uint16_t id,class USER_CAN* CAN_rev):ID(id),can_rev(CAN_rev){}
		private:
			
};

#endif
