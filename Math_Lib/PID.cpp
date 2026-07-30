#include "PID.h"

bool PID_class::Get_6020mang_need_turn_direction_is(uint16_t goal, uint16_t now)
{
	return (abs(goal - now) > 2048);
}
void PID_class::PID_update_for_6020mang(int16_t goal, uint16_t now, struct wheel_dir_and_weight *wheel_dir_and_weight)
{
	int16_t deal_3508_g = goal << 3,deal_3508_n = now << 3;
	int now_error;
	goal &= 0x1fff;
	bool ret = Get_6020mang_need_turn_direction_is(goal, now);
	int16_t res;
	int16_t error;

	if (ret)
	{
		goal = (goal + (8192 / 2)) & 0x1fff;
	}

	res = (goal - now);
	if (res > 8192 / 2)
	{
		error = (res - 8192);
	}
	else if (res < -8192 / 2)
	{
		error = (res + 8192);
	}
	else
	{
		error = res;
	}

	now_error = error;

	wheel_dir_and_weight->speed = (float)((2048 - abs(now_error)) / (float)(2048)) * wheel_dir_and_weight->yaogan_speed;
	
	if(((deal_3508_g - (2048 << 3)) < deal_3508_n) && ((deal_3508_g + (2048 << 3)) > deal_3508_n))
	{
		wheel_dir_and_weight->dir = 0;
	}
	else{
		wheel_dir_and_weight->dir = 1;
		wheel_dir_and_weight->speed = -wheel_dir_and_weight->speed;
	}

	this->OUT_P = this->KP * now_error;

	this->OUT_I += this->KI * (now_error);
	this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);


	this->OUT_D = this->KD * (now_error - this->LAST_Error);

	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

	this->LAST_Error = now_error;
}

/*******************************************************************************************************/
void PID_class::PID_new_update(float goal,float now)
{
	this->error = goal - now;
	
	if(fabs(this->error) < this->Deadzoom)//死区
	{
		this->error=0;
	}
	if(fabs(this->error) < this->Separate)//积分分离
	{
		this->Integral += this->error;
//		this->Integral += (this->error + this->last_error)/2;
		this->Integral = LIMIT(this->Integral,-this->LIMIT_I,this->LIMIT_I);
	}
	else	
	{
		this->OUT_I=0;
	}
	
	this->OUT_P = this->KP * this->error;
	
	this->OUT_I = this->KI*this->Integral;
	
//	this->OUT_D = this->KD * (this->error - this->last_error);
	
	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);

	this->last_error = this->error;
}

void PID_class::PID_update(float goal,float now)
{
	float now_error;

	now_error = goal - now;

	this->OUT_P = this->KP * now_error ;
	this->OUT_P=LIMIT(this->OUT_P,-this->LIMIT_P,this->LIMIT_P);

	this->OUT_I 	+=  this->KI*(now_error);
	this->OUT_I=LIMIT(this->OUT_I,-this->LIMIT_I,this->LIMIT_I);
	
	this->OUT_D = this->KD * (now_error - this->LAST_Error);
	this->OUT_D=LIMIT(this->OUT_D,-this->LIMIT_D,this->LIMIT_D);

	this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->OUT_PID = LIMIT(this->OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);

	this->LAST_Error=now_error;
}

void PID_class::PID_Inc_update(float goal,float now)
{
	float now_error;
	
	now_error = goal - now;

	this->OUT_P = this->KP * (now_error-this->last_error) ;
	this->OUT_P = LIMIT(this->OUT_P,-this->LIMIT_P,this->LIMIT_P);

	this->OUT_I = this->KI * now_error;
	this->OUT_I = LIMIT(this->OUT_I,-this->LIMIT_I,this->LIMIT_I);

	this->OUT_D = this->KD * (now_error - 2*this->last_error + this->previous_error);
	this->OUT_D = LIMIT(this->OUT_D,-this->LIMIT_D,this->LIMIT_D);

	this->delta_OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
	this->delta_OUT_PID = LIMIT(this->delta_OUT_PID,-this->LIMIT_PID,this->LIMIT_PID);
	
	this->OUT_PID+=this->delta_OUT_PID;

	this->previous_error = this->last_error;
	this->last_error = now_error;
}

void PID_class::PID_update_LP(float goal, float now, float k_value)
{
    this->error = goal - now;

    this->OUT_P = this->KP * this->error;
    this->OUT_P = LIMIT(this->OUT_P, -this->LIMIT_P, this->LIMIT_P);
    if(fabs(this->error) < this->Deadzoom)//死区
	{
		this->error=0;
	}
        if (fabs(this->error) < this->Separate)
        {
            this->OUT_I += this->KI * this->error;
            this->OUT_I = LIMIT(this->OUT_I, -this->LIMIT_I, this->LIMIT_I);
        }
        else if (this->error * this->OUT_I < 0)
        {
            this->OUT_I = 0;
        }

    this->OUT_D = this->KD * low_pass_filter(this->error - this->LAST_Error, k_value);
    this->OUT_D = LIMIT(this->OUT_D, -this->LIMIT_D, this->LIMIT_D);

    this->OUT_PID = this->OUT_P + this->OUT_I + this->OUT_D;
    this->OUT_PID = LIMIT(this->OUT_PID, -this->LIMIT_PID, this->LIMIT_PID);

    this->LAST_Error = this->error;
}
//低通
float PID_class::low_pass_filter(float value, float k_value)
{
    float out;
	float real_value = 2*k_value*3.1415926*0.001;
    static int fisrt_lp_flag = 1;
    if (fisrt_lp_flag == 1)
    {
        fisrt_lp_flag = 0;
        LP_out_last = value;
    }
    out = LP_out_last + k_value * (value - LP_out_last);
    LP_out_last = out;

    return out;
}

//模糊PID算法
void PID_Fuzzy_class::fuzzy(float goal,float now)
{
	this->error=goal - now;
	 float e = this->error/ this->stair;
	 float ec = (this->Out - this->Out_last) / this->stair;
	 short etemp,ectemp;
	 float eLefttemp,ecLefttemp;    //隶属度
	 float eRighttemp ,ecRighttemp; 

	 short eLeftIndex,ecLeftIndex;  //标签
	 short eRightIndex,ecRightIndex;

	//模糊化
	 if(e>=PL)
		 etemp=PL;//超出范围
	 else if(e>=PM)
		 etemp=PM;
	 else if(e>=PS)
		 etemp=PS;
	 else if(e>=ZE)
		 etemp=ZE;
	 else if(e>=NS)
		 etemp=NS;
	 else if(e>=NM)
		 etemp=NM;
	 else if(e>=NL)
		 etemp=NL;
	 else 
		 etemp=2*NL;

	 if( etemp == PL)
	{
	 //计算E隶属度
			eRighttemp= 0 ;    //右溢出
			eLefttemp= 1 ;
		
	 //计算标签
	 eLeftIndex = 6 ;      
	 eRightIndex= 6 ;
		
	}else if( etemp == 2*NL )
	{
		//计算E隶属度
			eRighttemp = 1;    //左溢出
			eLefttemp = 0;

	 //计算标签
	 eLeftIndex = 0 ;       
	 eRightIndex = 0 ;
	}	else 
	{
		//计算E隶属度
			eRighttemp=(e-etemp);  //线性函数作为隶属函数
			eLefttemp=(1- eRighttemp);
	
	 //计算标签
	 eLeftIndex =(short) (etemp-NL);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
	 eRightIndex=(short) (eLeftIndex+1);
	}		

	 if(ec>=PL)
		 ectemp=PL;
	 else if(ec>=PM)
		 ectemp=PM;
	 else if(ec>=PS)
		 ectemp=PS;
	 else if(ec>=ZE)
		 ectemp=ZE;
	 else if(ec>=NS)
		 ectemp=NS;
	 else if(ec>=NM)
		 ectemp=NM;
	 else if(ec>=NL)
		 ectemp=NL;
	 else 
		 ectemp=2*NL;
  
   if( ectemp == PL )
	 {
    //计算EC隶属度		 
		 ecRighttemp= 0 ;      //右溢出
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //计算EC隶属度		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //计算EC隶属度		 
		 ecRighttemp=(ec-ectemp);
		 ecLefttemp=(1- ecRighttemp);
			
		 ecLeftIndex =(short) (ectemp-NL);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }
	this->dKp = this->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	this->dKi = this->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	this->dKd = this->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
}
//没用上
void PID_Fuzzy_class::FuzzyPID_update(float goal ,float now)
{
	  this->LastError = this->error;
		this->error = goal - now;
		
		fuzzy(goal,now);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值
	
    float Kp = this->Kp0 + this->dKp , Ki = this->Ki0 + this->dKi , Kd = this->Kd0 + this->dKd ;   //PID均模糊
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //仅PD均模糊
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //仅P均模糊

		
    if(fabs(this->error) < this->I_L )			
		{
			this->SumError += this->error/2;    
			this->SumError = LIMIT(this->SumError,-this->IMax,this->IMax);
		}
			
		this->POut = Kp * this->error;
		this->IOut = Ki * this->SumError;
		this->DOut = Kd * (this->error - this->LastError);  
		
		this->Out_last  = this->Out;
		this->Out = LIMIT(this->POut+this->IOut+this->DOut,this->OutMax,-this->OutMax);
		                         
}