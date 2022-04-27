/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
  // init shared memory:                                                    //此共享内存类_sharedMemory为仿真桥成员变量，此处对该类赋值
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);           //共享内存命名？  括号里的是字符串："development-simulator"
  _sharedMemory().init();                                                   //共享内存初始化（生成共享内存空间）？

  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);        //install_segfault_handler函数形参为字符指针
                                                                            //segfault:断层
                                                                            //void install_segfault_handler(char* error_message);
                                                                            //猜测此处为处理报错

  // init Quadruped Controller
  try {
    printf("[Simulation Driver] Starting main loop...\n");
    bool firstRun = true;
    for (;;) {
      // wait for our turn to access the shared memory                            
      // on the first loop, this gives the simulator a chance to put stuff in     
      // shared memory before we start
      _sharedMemory().waitForSimulator();                                     //等待共享内存可以访问,第一次循环中，给仿真器时间开辟共享内存

      if (firstRun) {                                                         //第一次循环，检查机器人模型是否与输入值匹配
        firstRun = false;
        // check that the robot type is correct:
        if (_robot != _sharedMemory().simToRobot.robotType) {             
          printf(
            "simulator and simulatorDriver don't agree on which robot we are "
            "simulating (robot %d, sim %d)\n",
            (int)_robot, (int)_sharedMemory().simToRobot.robotType);
          throw std::runtime_error("robot mismatch!");                        //抛出异常：机器人类型不匹配
        }
      }

      // the simulator tells us which mode to run in
      _simMode = _sharedMemory().simToRobot.mode;                                           //从共享内存查看仿真器此时处于何种模式
                                                                                            /* 仿真环境存在四种模式：
                                                                                            1、需要新的控制参数
                                                                                            2、仿真已经就绪，可以运行控制器
                                                                                            3、仿真器仅仅自检是否运行正常
                                                                                            4、仿真结束 */

      switch (_simMode) {                                                                   //判断仿真模式
        case SimulatorMode::RUN_CONTROL_PARAMETERS:  // there is a new control              //请求新的控制参数
          // parameter request
          handleControlParameters();                                                        //处理控制参数 
          break;                                                                            //跳出switch
        case SimulatorMode::RUN_CONTROLLER:  // the simulator is ready for the
          // next robot controller run
          _iterations++;                                                                    //iteration：迭代
          runRobotControl();                                                                //运行控制器
          break;
        case SimulatorMode::DO_NOTHING:  // the simulator is just checking to see 
          // if we are alive yet
          break;
        case SimulatorMode::EXIT:  // the simulator is done with us                         //仅此情况退出
          printf("[Simulation Driver] Transitioned to exit mode\n");
          return;
          break;
        default:
          throw std::runtime_error("unknown simulator mode");
      }

      // tell the simulator we are done
      _sharedMemory().robotIsDone();
    }             //------------------------for循环结束

  } catch (std::exception& e) {                                 //异常处理
    strncpy(_sharedMemory().robotToSim.errorMessage, e.what(), sizeof(_sharedMemory().robotToSim.errorMessage));
    _sharedMemory().robotToSim.errorMessage[sizeof(_sharedMemory().robotToSim.errorMessage) - 1] = '\0';
    throw e;
  }

}

/*!     注意：此处原作者注释为处理一个控制参数，故猜测串行处理共享内存请求，确保每个请求都有回应
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters() {
  ControlParameterRequest& request =                              //记录共享内存sim To Robot请求值
      _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response =                            //记录共享内存robot To Sim回应值
      _sharedMemory().robotToSim.controlParameterResponse;
  if (request.requestNumber <= response.requestNumber) {          //仿真器请求数量应大于控制器回应数量，逻辑含义为：此时需要一个控制器回应
    // nothing to do!
    printf(
        "[SimulationBridge] Warning: the simulator has run a ControlParameter "
        "iteration, but there is no new request!\n");
    return;
  }
  // sanity check    正确性检测                                       sanity：神智正常，明智，正常
  u64 nRequests = request.requestNumber - response.requestNumber;                //u64  unsigned long long 64位整型
  assert(nRequests == 1);                                                       /* assert的作用是现计算表达式 expression ，
                                                                                如果其值为假（即为0），那么它先向stderr打印一条出错信息，
                                                                                然后通过调用 abort 来终止程序运行。
                                                                                */

  response.nParameters = _robotParams.collection._map                   //_robotParams为仿真桥成员变量，此处统计控制器参数数量nParameters？
                             .size();  // todo don't do this every single time?

  switch (request.requestKind) {    //switch内部变量生命周期不超过switch范围
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {        //仿真器共享内存通过变量名设置控制器参数
      std::string name(request.name);                                   //记录仿真器需要设置控制器参数的变量名
      ControlParameter& param = _robotParams.collection.lookup(name);   /* 创建一个ControlParameter引用，查询是否存在控制参数名为name的控制参数
                                                                        lookup函数返回一个实际的控制参数，并保存至param
                                                                        注：_robotParams，该类为仿真桥成员变量，类型：RobotControlParameters
                                                                         */

      // type check
      if (param._kind != request.parameterKind) {                       //比较数值类型
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      param.set(request.value, request.parameterKind);                  //仿真请求值与控制器中存在参数匹配，修改控制器中的该值与类型

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened          达成一次请求回应
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      std::string name(request.name);
      if(!_userParams) {
        printf("[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n");
      } else {
        ControlParameter& param = _userParams->collection.lookup(name);             /* 此处为_userParams，为仿真桥成员变量，
                                                                                    类型为ControlParameters*，为指针
                                                                                     */

        // type check
        if (param._kind != request.parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        param.set(request.value, request.parameterKind);
      }

      // respond:
      response.requestNumber =
          request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind =
          request.parameterKind;       // just for debugging print statements
      response.value = request.value;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      printf("%s\n", response.toString().c_str());

    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;  // acknowledge
      response.parameterKind =
          request.parameterKind;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind =
          request.requestKind;  // just for debugging print statements

      printf("%s\n", response.toString().c_str());
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    printf("[Simulator Driver] First run of robot controller...\n");
    if (_robotParams.isFullyInitialized()) {
      printf("\tAll %ld control parameters are initialized\n",
             _robotParams.collection._map.size());
    } else {
      printf(
          "\tbut not all control parameters were initialized. Missing:\n%s\n",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();        /* auto关键字，自动判断返回值类型，从而赋值，
                                                                                                此处auto推导为ControlParameters类型；
                                                                                                _robotRunner:仿真桥成员变量，RobotRunner*类型
                                                                                                 */

    if(userControlParameters) {
      if (userControlParameters->isFullyInitialized()) {
        printf("\tAll %ld user parameters are initialized\n",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        printf(
            "\tbut not all control parameters were initialized. Missing:\n%s\n",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }


    _robotRunner->driverCommand =
        &_sharedMemory().simToRobot.gamepadCommand;
    _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;
    _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;
    _robotRunner->robotType = _robot;
    _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
    _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;
    _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;
    _robotRunner->tiBoardCommand =
        _sharedMemory().robotToSim.tiBoardCommand;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData =
        &_sharedMemory().robotToSim.visualizationData;
    _robotRunner->cheetahMainVisualization =
        &_sharedMemory().robotToSim.mainCheetahVisualization;

    _robotRunner->init();
    _firstControllerRun = false;

    sbus_thread = new std::thread(&SimulationBridge::run_sbus, this);
  }
  _robotRunner->run();
}

/*!
 * Run the RC receive thread
 */
void SimulationBridge::run_sbus() {
  printf("[run_sbus] starting...\n");
  int port = init_sbus(true);  // Simulation
  while (true) {
    if (port > 0) {
      int x = receive_sbus(port);
      if (x) {
        sbus_packet_complete();
      }
    }
    usleep(5000);
  }
}
