/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
                                                                          //---------------包含三个参数----------------
int main_helper(int argc, char** argv, RobotController* ctrl) {           //ctrl为指向控制器类指针
    printUsage();                                                         
  if (argc != 3 && argc != 4) {                                           //传入命令参数不等于3个或者4个（第一个参数默认为可执行文件路径字符串）；第二个参数：机器人型号                                      
    return EXIT_FAILURE;                                                  //第三个参数：仿真还是现实
  }                                                                       //第四个参数：是否从文件加载参数                    


  if (argv[1][0] == '3') {                                                //第一个参数表示机器人为cheetah3
    gMasterConfig._robot = RobotType::CHEETAH_3;                          //gMasterConfig为确认配置结构体
  } else if (argv[1][0] == 'm') {                                         //机器人类型为minicheetah
    gMasterConfig._robot = RobotType::MINI_CHEETAH;                       //配置为minicheetah                                     
  } else {                                                                
    printUsage();                                     
    return EXIT_FAILURE;                                      
  }

  if (argv[2][0] == 's') {                                                //仿真模式
    gMasterConfig.simulated = true;                                       //gMasterConfig配置                        
  } else if (argv[2][0] == 'r') {                                         //真实模式                 
    gMasterConfig.simulated = false;                                      //                                                                     
  } else {                                                                                                             
    printUsage();                                         
    return EXIT_FAILURE;                                          
  }                                         

  if(argc == 4 && argv[3][0] == 'f') {                                    //从文件加载参数
    gMasterConfig.load_from_file = true;                                  //
    printf("Load parameters from file\n");
  } else {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");                             //
  }

  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n",
         gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah"       //a？b：c判断语句  a真则b，另外c
                                                         : "Cheetah 3");        //
  printf("        Driver: %s\n", gMasterConfig.simulated                        //是否仿真环境，是，则打印"Development Simulation Driver"，否则打印"Quadruped Driver"
                                     ? "Development Simulation Driver"          //
                                     : "Quadruped Driver");

  // dispatch the appropriate driver      dispatch:派遣
  if (gMasterConfig.simulated) {                                                //仿真模式为真
    if(argc != 3) {                                                             //主函数参数数量不是三个（包含默认参数，即实际输入为两个参数）
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {                      //机器人为minicheetah
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);            //创建simulationbridge类，同时调用构造函数，传入参数为机器人呢类型以及控制器指针
      simulationBridge.run();                                                   //调用simulationbridge类成员函数run,该函数作用为
      printf("[Quadruped] SimDriver run() has finished!\n");
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  } else {
#ifdef linux
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      Cheetah3HardwareBridge hw(ctrl);
      hw.run();
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
#endif
  }

  return 0;
}
