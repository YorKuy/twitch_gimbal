#ifndef __PID_H__
#define __PID_H__
#include "RM_Lib.h"
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
#define LIMIT(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3
class PID_class{
	public:
		float		error;
		float 		KP,KI,KD;
		float		LIMIT_P,LIMIT_I,LIMIT_D,LIMIT_PID,Integral;
		float 		LP_out_last ;
		float 		Deadzoom,Separate;
		float 		OUT_P,OUT_I,OUT_D,last_OUT_D,OUT_PID,delta_OUT_PID;
	  float 		g_speed_3508[4] = {0, 0, 0, 0};
		int16_t 	g_angle_6020[4] = {0, 0, 0, 0};
	
	  bool 		Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now);
	  void 		PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight);
		void 		PID_new_update(float goal,float now);
		void 		PID_update(float goal,float now);
		void    PID_Inc_update(float goal,float now);
		void 	PID_update_LP(float goal, float now, float k_value);
		float 	low_pass_filter(float value, float k_value);
		PID_class(float kp,float ki,float kd,float limit_p,float limit_i,float limit_d,float limit_pid,float dz = 0,float separate = 0):
		KP(kp),KI(ki),KD(kd),LIMIT_P(limit_p),LIMIT_I(limit_i),LIMIT_D(limit_d),LIMIT_PID(limit_pid),Deadzoom(dz),Separate(separate){}
	private:
		float alpha=0.5;
		float LAST_Error;
		float last_error;
		float previous_error;
};

class PID_Fuzzy_class{
	public:
		float LastError;		//前次误差
		float error;			//当前误差
		float SumError;			//积分误差
		float IMax;					//积分限制
		float POut,IOut,DOut;	//比例输出
	  float DOut_last;    //上一次微分输出
		float OutMax;       //限幅
	  float Out;          //总输出
		float Out_last;     //上一次输出
		
		float I_U;          //变速积分上限
		float I_L;          //变速积分下限
	
	  float Kp0,Ki0,Kd0;          //PID初值
	  float dKp,dKi,dKd;          //PID变化量
	
    float stair,Kp_stair,Ki_stair,Kd_stair;	      //动态调整梯度   //0.25f
		
		void  FuzzyPID_update(float goal ,float now);
		PID_Fuzzy_class(float kp,float ki,float kd,float limit_i,float limit_pid,float IL,float Stair,float KP_stair,float KI_stair,float KD_stair):
		Kp0(kp),Ki0(ki),Kd0(kd),IMax(limit_i),OutMax(limit_pid),I_L(IL),stair(Stair),Kp_stair(KP_stair),Ki_stair(KI_stair),Kd_stair(KD_stair){}		
		
	private:
		
	const float fuzzyRuleKp[7][7]={
		PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
		PL,	PL,	PM,	PS,	PS,	ZE,	NS,
		PM,	PM,	PM,	PS,	ZE,	NS,	NS,
		PM,	PM,	PS,	ZE,	NS,	NM,	NM,
		PS,	PS,	ZE,	NS,	NS,	NM,	NM,
		PS,	ZE,	NS,	NM,	NM,	NM,	NL,
		ZE,	ZE,	NM,	NM,	NM,	NL,	NL
	};
	 
	const float fuzzyRuleKi[7][7]={
		NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
		NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
		NL,	NM,	NS,	NS,	ZE,	PS,	PS,
		NM,	NM,	NS,	ZE,	PS,	PM,	PM,
		NS,	NS,	ZE,	PS,	PS,	PM,	PL,
		ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
		ZE,	ZE,	PS,	PM,	PM,	PL,	PL
	};
	 
	const float fuzzyRuleKd[7][7]={
		PS,	NS,	NL,	NL,	NL,	NM,	PS,
		PS,	NS,	NL,	NM,	NM,	NS,	ZE,
		ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
		ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
		ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
		PL,	NS,	PS,	PS,	PS,	PS,	PL,
		PL,	PM,	PM,	PM,	PS,	PS,	PL
	};
	
	void 	fuzzy(float goal,float now);
};

#endif
