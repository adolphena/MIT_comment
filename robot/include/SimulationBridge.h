/*! @file SimulationBridge.h
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <thread>

#include "ControlParameters/RobotParameters.h"
#include "RobotRunner.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"

class SimulationBridge {
 public:
  explicit SimulationBridge(RobotType robot, RobotController* robot_ctrl) :           //explicit关键字：阻止隐式转换的发生，即必须为类名加（参数1，参数2）
    _robot(robot) {                                                                   //为该类成员变量赋值，类初始化
     _fakeTaskManager = new PeriodicTaskManager;                                      //创建一个新的
    _robotRunner = new RobotRunner(robot_ctrl, _fakeTaskManager, 0, "robot-task");    /* robotrunner类继承父类为PeriodicTask类
                                                                                      此处第一个参数为主函数创建的控制器类的指针；
                                                                                      第二个参数为一周期任务管理器PeriodicTaskManager类指针；
                                                                                      第三参数为一浮点数
                                                                                      第四参数为字符串指针 
                                                                                      */

    _userParams = robot_ctrl->getUserControlParameters();                             /* 调用控制器的函数getUserControlParameters(),
                                                                                      此处返回值为ControlParameters类指针，
                                                                                       */                                                                    
 }




  void run();
  void handleControlParameters();
  void runRobotControl();
  ~SimulationBridge() {                                                               //析构函数，释放周期任务
    delete _fakeTaskManager;
    delete _robotRunner;
  }
  void run_sbus();

 private:
  PeriodicTaskManager taskManager;              
  bool _firstControllerRun = true;
  PeriodicTaskManager* _fakeTaskManager = nullptr;
  RobotType _robot;
  RobotRunner* _robotRunner = nullptr;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  RobotControlParameters _robotParams;
  ControlParameters* _userParams = nullptr;
  u64 _iterations = 0;

  std::thread* sbus_thread;
};

#endif  // PROJECT_SIMULATIONDRIVER_H
