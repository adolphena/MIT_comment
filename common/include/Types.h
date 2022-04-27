#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include "cppTypes.h"

struct MasterConfig {
  RobotType _robot;                           //机器人种类 robot type 为枚举变量类型 仅仅包含cheetah3和Mimicheetah
  bool simulated = false;                     //是否为仿真
  bool load_from_file = false;                //是否从文件加载参数
};

#endif  // PROJECT_TYPES_H
