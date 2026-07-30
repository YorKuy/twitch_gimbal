#include "SMC.h"

#define PI 3.14159
#if SMC_MDOE == 0

#endif

#if SMC_MODE == 1
void SMC::SMC_Tick_4310(float angle_now,float angle_vel) //anlge为当前位置(°),ang_vel为角速度(°/s)
{
	//读取参数
	const float deg2rad = PI / 180.0f;
    angle = angle_now * deg2rad;      // 转换为弧度
    ang_vel = angle_vel * deg2rad;    // 转换为弧度/秒
	error = angle - ref;
	ddref = (ref - refl) - dref; //这里对前馈进行了处理，没有严格单位统一
	dref = (ref - refl);   //其实就是0
	//误差下限处理
	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}
	//smc surface
	s = C * error + (ang_vel - dref); //滑模面设计  角速度*误差变化
	u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat_New(s,delta) - K * s);
	//控制量限幅
	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;
	//参数更新
	refl = ref;
}
void SMC::SMC_Tick(float angle_now,float angle_vel) //anlge为当前位置(°),ang_vel为角速度(°/s)
{
	//读取参数
    angle = angle_now ;      // 转换为弧度
    ang_vel = angle_vel;    // 转换为弧度/秒
	error = angle - ref;
	ddref = (ref - refl) - dref; //这里对前馈进行了处理，没有严格单位统一
	dref = (ref - refl);   //其实就是0
	//误差下限处理
	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}
	//smc surface
	s = C * error + (ang_vel - dref); //滑模面设计  角速度*误差变化
	u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
	//控制量限幅
	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;
	//参数更新
	refl = ref;
}
void SMC::SMC_AngleSpeed(float Target_angle,float Target_angle_vel,float Target_angle_acc, float angle, float angle_vel)
{
    // 1. 误差定义（期望 - 实际）
    error = angle - Target_angle;
    
    // 2. 速度误差（期望速度 - 实际速度）
      d_error = angle_vel - Target_angle_vel;
    
      // 误差下限处理：位置和速度都很小时才停控
      float vel_eps = fabs(C) * error_eps;
      if (vel_eps < 0.5f)
          vel_eps = 0.5f;
      if ((fabs(error) < error_eps) && (fabs(d_error) < vel_eps))
      {
          u = 0;
          return;
      }
     
    // 3. 滑模面
        s = C * error + d_error;
    
    // 4. 控制律（带边界层）
      const float sat_delta = (delta > 1e-6f) ? delta : 1.0f;
        u = J * (Target_angle_acc - C * d_error - K * s - epsilon * Sat_New(s, sat_delta));
    
    // 5. 限幅
    if (u > u_max)  u = u_max;
    if (u < -u_max) u = -u_max;
}
void SMC::SMC_Tick_New(float angle_now, float angle_vel)
{
    // 读取参数
    angle = angle_now;
    ang_vel = angle_vel;
    error = angle - ref;
    
    static float Ts = 0.001; // 例如1ms
    float delta = ref - refl;
    dref = delta / Ts;
    ddref = (delta - last_delta) / (Ts * Ts);
    last_delta = delta;
    
    // 误差下限处理
    if (fabs(error) < error_eps)
    {
        u = 0;
        return;
    }
    
    // 滑模面
    s = C * error + (ang_vel - dref);
    
    // 控制律输出力矩
    u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
    
    // 力矩限幅
    if (u > u_max) u = u_max;
    if (u < -u_max) u = -u_max;
    
    // 更新参数
    refl = ref;
}
void SMC_PITCH::SMC_Tick(float Target_angle, float Target_angle_vel, float Target_angle_acc, float angle, float angle_vel)
{
    // 1. 误差定义
    error = angle - Target_angle;

    // 2. 速度误差
    d_error = angle_vel - Target_angle_vel;
    //积分误差累加
    error_integral += error * 0.001f; // 采样周期为1ms 1000HZ

    // 3. 滑模面
    s = C * error + d_error + C2 * error_integral;

    // 4. 控制律
    u = J * (Target_angle_acc - C * d_error - C2 * error - K * s - epsilon * Sat(s));

    // 5. 限幅
    u = LIMIT(u, -u_max, u_max);
}
#endif

#if SMC_MODE == 2
void cSMC::Init()
{
    smc.param.J = 0;
    smc.param.K = 0;
    smc.param.c = 0;
    smc.param.epsilon = 0;
    smc.flag = EXPONENT;
    smc.u_max = 0;
    smc.limit = 0;

    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.v_error_integral = 0;
//    smc.error.v_error_integral_max = 0;
    smc.error.pos_error_eps = 0;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;
}

void cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EXPONENT,POWER,VELSMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.c = c;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}
void cSMC::SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///TFSMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.p = p;
    smc.param.q = q;
    smc.error.pos_error_eps = pos_esp;
    smc.param.beta = beta;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}

void cSMC::SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EISMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.c1 = c1;
    smc.param.c2 = c2;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}

void cSMC::ErrorUpdate(float target, float pos_now, float vol_now) //位置环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.p_error = pos_now - target;
    smc.error.v_error = vol_now - smc.error.tar_differential;
    smc.error.tar_last = smc.error.tar_now;

    smc.error.p_error_integral += (float)(smc.error.p_error * SAMPLE_PERIOD); ///位置误差积分项

    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}

void cSMC::ErrorUpdate(float target,float vol_now) //速度环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

//    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.v_error = vol_now - smc.error.tar_now;

    smc.error.v_error_integral += (float)(smc.error.v_error * SAMPLE_PERIOD); ///速度误差积分项
//    if(std::abs(smc.error.v_error_integral) > smc.error.v_error_integral_max) //积分限幅
//    {
//        smc.error.v_error_integral = smc.error.v_error_integral_max;
//    }

    smc.error.tar_last = smc.error.tar_now;

//    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}
void cSMC::Clear()
{
    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.v_error_integral = 0;
//    smc.error.v_error_integral_max = 0;
    smc.error.pos_error_eps = 0;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;

    smc.error.tar_differential_second = 0;
    smc.error.tar_differential_last = 0;
    smc.error.p_error_integral = 0;
}
void cSMC::Integval_Clear()
{
    smc.error.v_error_integral = 0;
    smc.error.p_error_integral = 0;
}

float cSMC::SmcCalculate()
{
    float u,fun;

    switch (smc.flag) {
        case EXPONENT:///线性滑模面，指数趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun + smc.error.tar_differential_second); //控制器计算,指数趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率

            break;
        case POWER:///线性滑模面，幂次趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun + smc.error.tar_differential_second); //控制器计算,幂次趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun); //控制器计算,幂次趋近率

            break;
        case TFSMC:///tfsmc
            static float pos_pow;//tfsmc 位置 幂
            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            pos_pow = std::pow(std::abs(smc.error.p_error),smc.param.q/smc.param.p);
            if(smc.error.p_error<=0) pos_pow = -pos_pow;


            smc.s = smc.param.beta * pos_pow + smc.error.v_error; //滑模面

            fun = Sat(smc.s);//饱和函数消除抖动

            if(smc.error.p_error!=0)
            {
                u = smc.param.J * (smc.error.tar_differential_second//目标值的二阶导 暂定是否删除，需测试
                             -smc.param.K * smc.s //s*K
                             -smc.param.epsilon * fun  //epsilon*SAT(S)
                             -smc.error.v_error * ((smc.param.q * smc.param.beta) * pos_pow) / (smc.param.p * smc.error.p_error)); //控制器计算
            }
            else u = 0;
            break;
        case VELSMC:///比例积分滑模面，指数趋近律，速度控制
            smc.s = smc.error.v_error + smc.param.c * smc.error.v_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * (smc.error.tar_differential - (smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算，速度控制
            break;

        case EISMC:///比例积分滑模面，指数趋近律，位置控制
            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c1 * smc.error.p_error + smc.error.v_error + smc.param.c2 * smc.error.p_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * ( (-smc.param.c1 * smc.error.v_error)- smc.param.c2 * smc.error.p_error - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率
            break;
    }

    smc.error.error_last = smc.error.p_error; //更新上一步的误差

    //控制量限幅
    if (u > smc.u_max)
    {
        u = smc.u_max;
    }
    if (u < -smc.u_max)
    {
        u = -smc.u_max;
    }
    smc.u = u;
    return u;
}

void cSMC::OutContinuation()
{
    if(smc.param.K != 0 && smc.param.c2 != 0)
    {
        smc.error.p_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c2 / smc.param.c2) * smc.error.p_error_integral;
        smc.error.v_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c / smc.param.c) * smc.error.v_error_integral;
    }
    smc.param_last = smc.param;
}


// 符号函数
float cSMC::Signal(float s)
{
    if (s > 0)
        return 1;
    else if (s == 0)
        return 0;
    else
        return -1;
}

//饱和函数
float cSMC::Sat(float s)
{
    float y;
    y = s / smc.param.epsilon;
    if (std::abs(y) <= smc.limit)
        return y;
    else
        return Signal(y);
}

const Sliding &cSMC::getSmc() const {
    return smc;
}

float cSMC::Out() {
    return smc.u;
}

void cSMC::SetOut(float out) {
    smc.u = out;
}
#endif
