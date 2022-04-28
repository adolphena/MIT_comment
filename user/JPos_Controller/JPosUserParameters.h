#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class JPosUserParameters : public ControlParameters {
public:
  JPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd),
        INIT_PARAMETER(zero),
        INIT_PARAMETER(calibrate)
      {}

  DECLARE_PARAMETER(double, tau_ff);        //关节转矩前馈  
  DECLARE_PARAMETER(double, kp);            //
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, zero);          //猜测是零位？
  DECLARE_PARAMETER(double, calibrate);     //calibrate：校准
};

#endif //PROJECT_JPOSUSERPARAMETERS_H



//测试
//第二次测试
//第三次测试
//网络到本地
