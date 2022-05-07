### 基本概念
- P比例 : 误差err=比例*(期望目标-实际目标)
- I积分:  静态误差
- D微分:根据斜率超前调节
### 代码实现

```C
struct _pid {
    float ExpectedValue;//定义设定值
    float ActualValue;//定义实际值
    float err;//定义偏差值
    float err_last;//定义上一个偏差值
    float err_prev;//定义前一个的偏差值
    float Kp, Ki, Kd;//定义比例、积分、微分系数
}pid;
 
//增量式pid公式
float PID_Realize(float speed) 
{
    float index;
    pid.ExpectedValue = speed;
    pid.err = pid.ExpectedValue - pid.ActualValue;
    
    float incrementValue = pid.Kp*(pid.err - pid.err_last) + 
                           pid.Ki*pid.err + 
    pid.Kd*(pid.err - 2 * pid.err_last + pid.err_prev);
    
    pid.ActualValue += incrementValue;
    pid.err_prev     = pid.err_last;
    pid.err_last     = pid.err;
    
    return pid.ActualValue;
}
```

 

```C
//位置式,常常只用PD而忽略了I
typedef struct PID
{ 
  float P,I,D,limit;
}PID;
 
typedef struct Error
{
  float Current_Error;//当前误差
  float Last_Error;//上一次误差
  float Previous_Error;//上上次误差
}Error;
 
/*! 
 *  @brief      位置式PID
 *  @since      v1.0
 *  *sptr ：误差参数
 *  *pid:  PID参数
 *  NowPlace：当前位置
 *  Point：   预期位置  
 */
 
// 位置式PID控制
float PID_Realize(Error *sptr,PID *pid, int32 NowPlace, float Point)
{
 
    int32 iError,   // 当前误差
         Realize;   //实际输出  
 
    iError = Point - NowPlace;  // 计算当前误差
    sptr->Current_Error += pid->I * iError; // 误差积分
    sptr->Current_Error = sptr->Current_Error > pid->limit?pid->limit:sptr->Current_Error;//积分限幅
    sptr->Current_Error = sptr->Current_Error <-pid->limit?-pid->limit:sptr->Current_Error;
    Realize = pid->P * iError       //比例P
            + sptr->Current_Error   //积分I
            + pid->D * (iError - sptr->Last_Error);  //微分D
    sptr->Last_Error = iError;          // 更新上次误差
    return Realize; // 返回实际值
}    
```

### 增量式与位置式区别
- 增量式算法不需要做累加，控制量增量的确定仅与最近几次偏差采样值有关，计算误差对控制 量计算的影响较小。而位置式算法要用到过去偏差的累加值，容易产生较大的累加误差。 
- 增量式算法得出的是控制量的增量，例如在阀门控制中，只输出阀门开度的变化部分，误动作 影响小，必要时还可通过逻辑判断限制或禁止本次输出，不会严重影响系统的工作。 而位置式的输出直接对应对象的输出，因此对系统影响较大。
- 增量式PID控制输出的是控制量增量，并无积分作用，因此该方法适用于执行机构带积分部件的对象，如步进电机等，而位置式PID适用于执行机构不带积分部件的对象，如电液伺服阀。
- 在进行PID控制时，位置式PID需要有积分限幅和输出限幅，而增量式PID只需输出限幅
### 位置式PID优缺点
- 优点
  - 位置式PID是一种非递推式算法，可直接控制执行机构（如平衡小车），u(k)的值和执行机构的实际位置（如小车当前角度）是一一对应的，因此在执行机构不带积分部件的对象中可以很好应用
- 缺点
  - 每次输出均与过去的状态有关，计算时要对e(k)进行累加，运算工作量大。
### 增量式PID优缺点
- 优点
  - 误动作时影响小，必要时可用逻辑判断的方法去掉出错数据。
  - 手动/自动切换时冲击小，便于实现无扰动切换。当计算机故障时，仍能保持原值。
  - 算式中不需要累加。控制增量Δu(k)的确定仅与最近3次的采样值有关。
- 缺点
- 积分截断效应大，有稳态误差；
- 溢出的影响大。有的被控对象用增量式则不太好；

### 积分饱和
- 就是限制在一个区间内,如果积分累计和(A ,B)在此区间外,则设为区间的边缘,如小于A,则设为A