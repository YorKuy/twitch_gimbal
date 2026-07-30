#ifndef __RM_H__
#define __RM_H__

#include "RM_Lib.h"
class MOTOR_RM{
	public:
		const 		uint16_t ID;	// µç»ú·´À¡ID
		USER_CAN* 	can_rev;
	
		int16_t  	mang;      		// (int16_t) ((Can1_Data[0] << 8) | Can1_Data[1]);
		int16_t  	sp;        		// (int16_t) ((Can1_Data[2] << 8) | Can1_Data[3]);
		int16_t  	AT_current;     // (int16_t) ((Can1_Data[4] << 8) | Can1_Data[5]);
		int8_t   	temp;      		//  Can1_Data[6];
		int8_t      Error_State;
		int 		mang_inf;
		int 		motor_number;
		HAL_StatusTypeDef 	DAMIAO_Update(void);
	
		HAL_StatusTypeDef 		update(void);
		void 					update_mang_inf(void);
		MOTOR_RM(const uint16_t id,class USER_CAN* CAN_rev):ID(id),can_rev(CAN_rev){}
	private:
		uint8_t 	first;
		int 		Last_mang;
};

#endif
