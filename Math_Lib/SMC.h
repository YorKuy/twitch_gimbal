#ifndef _SMC_H__
#define _SMC_H__

#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
#include "math.h"
#include "stm32f4xx_hal.h"
#include "communication.h"
typedef double db;

#define SMC_MODE 1     //(0:MySMC,1:复旦的SMC，2:川大的SMC)

#if   SMC_MODE == 0
namespace SMC{
    //符号函数
    inline db sign(db x)
    {
        if(x > 0.0) return 1.0;
        if(x < 0.0) return -1.0;
        return 0.0;
    }
    //饱和函数(线性)
    inline db saturate(db x,db limit)
    {
        if(x > limit) return 1.0;
        if(x < -limit) return -1.0;
        return x / limit;
    }
    
    //边界层饱和函数
    inline db boundaryLayerSat(db s,db Phi)
    {
        if(Phi <= 0.0) return sign(s);
        return saturate(s,Phi);
    }
    
    class SMC_Base{
        protected:
            struct Parameters{
                db lambda;  //滑模面参数
                db K;       //切换增益
                db Phi;     //边界层厚度
                db u_min;   //控制下限
                db u_max;   //控制上限
                
                Parameters():lambda(50.0),K(2.5),Phi(0.1),u_min(-20.0),u_max(20.0){}
            }params;
            
            struct State{
                db u_total;  //总控制量
                db u_eq;     //等效控制
                db u_sw;     //切换控制
                db s;        //滑膜控制
                db e;        //误差
                db e_dot;    //误差导数
                int region;  //控制区域（1：边界层内，2：边界层外）
                
                State():u_total(0.0),u_eq(0.0),u_sw(0.0),s(0.0),e(0.0),e_dot(0.0),region(1){}
            }state;
            
            struct Performance{
                db error_sum_sq;      //误差平方和
                db control_energy;    //控制能量
                db max_error;         //最大误差
                int step_count;       //步数计数
                
                Performance():error_sum_sq(0.0),control_energy(0.0),
                              max_error(0.0),step_count(0){}
            }performance;
            //滑模面设计
            virtual db SlidingSurface(db e,db e_dot)
            {
                return e_dot + params.lambda * e;
            }
            
            virtual db EquivalentControl(db e,db e_dot,db ref,db ref_dot,db ref_ddot) = 0;
            virtual db SwitchingControl(db s){
                db sat_val = boundaryLayerSat(s,params.Phi);
                state.region = (std::abs(s) <= params.Phi)?1:2;
                return -params.K * sat_val;
            }
            
            db clampControl(db u){
                return std::clamp(u,params.u_min,params.u_max);
            }
            
            void updatePerformance(db error, db control)
            {
                performance.error_sum_sq += error * error;
                performance.control_energy += control * control;
                performance.max_error = std::max(performance.max_error,std::abs(error));
                performance.step_count++;
            }
            
        public:
              virtual ~SMC_Base() = default; 

              void setParameters(db lambda,db K,db Phi)
              {
                params.lambda = lambda;
                params.K = K;
                params.Phi = Phi;
              }
              
             virtual db update(db actual,db actual_dot,db reference,db reference_dot,db reference_ddot)
             {
                state.e = actual - reference;
                state.e_dot = actual_dot - reference_dot;
                 
                //计算滑模面
                 state.s = SlidingSurface(state.e,state.e_dot);
                 
                 //计算控制分量
                 state.u_eq = EquivalentControl(state.e,state.e_dot,reference,reference_dot,reference_ddot);
                 state.u_sw = SwitchingControl(state.s);
                 
                 state.u_total = state.u_eq + state.u_sw;
                 state.u_total = clampControl(state.u_total);
                 
                 updatePerformance(state.e,state.u_total);
                 
                 return state.u_total;
             }

              virtual void reset(){
                 state = State();
                 performance = Performance();
              }
    };
    
    class PID_SMC_Hybrid{
            private:
                struct PID_Params{
                    db Kp;
                    db Ki;
                    db Kd;
                    PID_Params():Kp(0.0),Ki(0.0),Kd(0.0){}
                }pid_params;
                
                struct SMC_Params{
                    db lambda;
                    db K;
                    db Phi;
                    SMC_Params():lambda(50.0),K(0.0),Phi(0.0){}                    
                }smc_params;
                
                struct State{
                    db error_integral;  //误差积分
                    db prev_error;      //上一次误差
                    db u_pid;
                    db u_smc;
                    db u_total;
                    db s;
                    State():error_integral(0.0),prev_error(0.0),u_pid(0.0),u_smc(0.0),u_total(0.0),s(0.0){}
                }state;
                
                db u_min,u_max;           //控制限幅
                db blend_factor;          //混合权重(0纯PID，1纯SMC)
            
            public:
                PID_SMC_Hybrid(db blend = 0.5):
                    u_min(-20.0),u_max(20.0),blend_factor(blend){}
                        
                void setPIDParameters(db Kp,db Ki,db Kd)
                {
                    pid_params.Kp = Kp;
                    pid_params.Ki = Ki;
                    pid_params.Kd = Kd;
                }
                
                void setSMCParameters(db lambda,db K,db Phi)
                {
                    smc_params.lambda = lambda;
                    smc_params.K = K;
                    smc_params.Phi = Phi;
                }
                
                void setControlLimits(db min_vel,db max_vel)
                {
                    u_min = min_vel;
                    u_max = max_vel;
                }
                
                void setBlendFactor(db factor){
                    blend_factor = std::clamp(factor,0.0,1.0);
                }
                
                db update(db actual,db actual_dot,
                          db reference,db reference_dot,db reference_ddot, db dt){
                                db error = actual - reference;
                                db error_dot = actual_dot - reference_dot;
                                //计算滑模面
                                state.s = error_dot + smc_params.lambda * error;
                              
                                //PID计算
                                state.error_integral += error * dt;
                                db error_deriv = (error - state.prev_error) / dt;
                                state.prev_error = error;
                              
                                state.u_pid = pid_params.Kp * error +
                                              pid_params.Ki * state.error_integral + 
                                              pid_params.Kd * error_deriv;
                              
                                //SMC控制
                                db sat_val = boundaryLayerSat(state.s,smc_params.Phi);
                                state.u_smc = -smc_params.K * sat_val;
                              
                                //混合控制
                                state.u_total = (1.0 - blend_factor) * state.u_pid +
                                                blend_factor * state.u_smc;
                                                
                                state.u_total = std::clamp(state.u_total,u_min,u_max);
                                
                                return state.u_total;
                              
                          }
                          
                          void reset(){
                            state = State();
                          }
    };
    
    //一阶滑动力学
    class SMC_FirstOrder:public SMC_Base{
        protected:
            db EquivalentControl(db e,db e_dot,db ref,db ref_dot,db ref_ddot)override{
                return -params.lambda * e_dot;
            }
        public:
            SMC_FirstOrder() = default;
    };
    
    //二阶滑膜动力学
    class SMC_SecondOrder : public SMC_Base{
        protected:
            db J_nom;       //标称惯量
            db B;           //阻尼系数
            
            db EquivalentControl(db e,db e_dot,
                                 db ref,db ref_dot,db ref_ddot)override{
                                    return J_nom * (ref_ddot - params.lambda * e_dot) + B * (e_dot + ref_dot);
                                 }
        public:
            SMC_SecondOrder(db J = 0.01,db b = 0.1):J_nom(J),B(b){}
                
            void setSystemParameters(db J,db b){
                J_nom = J;
                B = b;
            }
    };  
}
#endif
//复旦大学的SMC
#if SMC_MODE == 1
class SMC{
 public:
	float C;
	float K;
	float ref; //初始目标值
	float error_eps;//误差下限
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，°/s
	float epsilon; //边界饱和 
    float last_delta;
    float delta;
	float u;
    float s;
    float error;
	//初始化列表
	SMC(float C,float K,float ref,float error_eps,float u_max,float J,float epsilon,float delta):
	C(C),K(K),ref(ref),error_eps(error_eps),u_max(u_max),J(J),epsilon(epsilon),delta(delta){};
	//更新函数
	void SMC_Tick(float angle_now,float angle_vel);
    void SMC_Tick_4310(float angle_now,float angle_vel);
    void SMC_Tick_New(float angle_now,float angle_vel);
    void SMC_AngleSpeed(float Target_angle,float Target_angle_vel,float Target_angle_acc, float angle, float angle_vel);
    
 private:
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值
    float d_error;
 //滑模面

	// 饱和函数
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}
	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
    
    float Sat_New(float y, float delta)
    {
        if (fabs(y) <= delta)
            return y / delta;      // 线性区，输出范围 [-1, 1]
        else
            return (y > 0 ? 1 : -1);
    }
    
};
class SMC_PITCH{
    public:
    float C;
	float K;
    float C2;   //EIsmc参数
	float ref; //初始目标值
	float error_eps;//误差下限
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，°/s
	float epsilon; //边界饱和 
    float last_delta;
    float delta;
	float u;
    float s;
    float error;
    float dt;
    SMC_PITCH(float C, float K, float c2, float error_eps, float u_max, float J, float epsilon) : 
    C(C), K(K), C2(c2), error_eps(error_eps), u_max(u_max), J(J), epsilon(epsilon) {};
    float gm6020to_torq(float u);
    void  SMC_Tick(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel);

    private:
    float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值
    float d_error;
    float error_integral;
 //滑模面

	// 饱和函数
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}
	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
    
    float Sat_v2(float y)
    {
        if (fabs(y) <= 1)
            return y / 1;      // 线性区，输出范围 [-1, 1]
        else
            return (y > 0 ? 1 : -1);
    }
};
extern SMC YawSMC;
#endif

//四川大学的SMC
#if SMC_MODE == 2
#define SAMPLE_PERIOD 0.002
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

typedef struct {

    float tar_now;//当前目标值
    float tar_last;//上一次目标值
    float tar_differential;//目标值一阶导
    float tar_differential_last;//上一次目标值一阶导
    float tar_differential_second;//目标值二阶导

    float pos_get;//当前位置
    float vol_get;//当前速度

    float p_error;//位置误差
    float v_error;//速度误差（位置误差一阶导）

    float p_error_integral;//位置误差积分
    float v_error_integral;//速度误差积分
//    float v_error_integral_max; //积分限幅

    float pos_error_eps;   //误差精度
    float vol_error_eps;   //误差精度
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;   //EIsmc参数
    float c2;   //EIsmc参数

    float p;    //tfsmc参数，正奇数 p>q
    float q;    //tfsmc参数，正奇数
    float beta; //tfsmc参数，正数
    float epsilon; //ε噪声上限
}SlidingParam;

typedef struct {
    float u; //控制器输出
    float s; //滑模面计算储存

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max; //输出限幅
    Rmode flag; //符号和饱和切换，未用到
    float limit; //饱和函数上下限
}Sliding;


class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //滑膜参数设定 EXPONENT,POWER,VELSMC 参数
    void SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//滑膜参数设定 TFSMC 参数
    void SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC 参数设定

    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜位置误差更新
    void ErrorUpdate(float target,float vol_now); //滑模速度误差更新
    void Clear();
    void Integval_Clear();

    float SmcCalculate(); //滑模控制器计算函数
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //参量提取

private:


    Sliding smc;
    void OutContinuation(); //输出连续化
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};
#endif

#endif   //__SMC_H__
