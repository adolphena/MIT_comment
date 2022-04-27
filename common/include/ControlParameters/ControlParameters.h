/*! @file ControlParameters.h
 *  @brief Interface to set gains/control parameters for simulator and robot
 *  These are designed to be updated infrequently.  For high frequency data,
 * consider using Driver Inputs or adding to the Robot Debug Data instead.
 *
 * ControlParameter: a single value, either a double, float, vec3, or s64.  Each
 * control parameter must have a unique name Control parameters know their type
 * as well as if they've been initialized or not ControlParameters must be
 * initialized, either from reading from a file, reading from LCM, or some other
 * way
 *
 * See test_ControlParameters for an example of how this works
 */

#ifndef PROJECT_CONTROLPAREMETERS_H
#define PROJECT_CONTROLPAREMETERS_H

#include <map>
#include <mutex>
#include <string>
#include "Utilities/utilities.h"
#include "cTypes.h"

#define CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH 64

/* 该宏定义可以理解为先声明变量，后初始化

后一句先初始化一个type类型的name，以及一个名为param_name的ControlParameter类
反斜杠\为一行未写完，续写的意思

两句加一起等同于  ControlParameter param_name(name字符串, type类型的name变量, collection)

#：构串操作符
构串操作符 # 只能修饰带参数的宏的形参，它将实参的字符序列（而不是实参代表的值）转化为字符串常量

##：合并操作符
合并操作符将出现在其左右的字符序列合并成一个新的标识符

 */
#define INIT_PARAMETER(name) param_##name(#name, name, collection)
#define DECLARE_PARAMETER(type, name) \
  type name;                          \
  ControlParameter param_##name;

/*!
 * Data types supported for control parameters
 */
enum class ControlParameterValueKind : u64 {
  FLOAT = 0,
  DOUBLE = 1,
  S64 = 2,
  VEC3_DOUBLE = 3,
  VEC3_FLOAT = 4
};

ControlParameterValueKind getControlParameterValueKindFromString(const std::string& str);

std::string controlParameterValueKindToString(
    ControlParameterValueKind valueKind);

/*!
 * Pointer to control parameter data
 */
union ControlParameterValuePtr {
  float* f;
  double* d;
  s64* i;
  float* vec3f;
  double* vec3d;
};

/*!
 * Value of a control parameter
 */
union ControlParameterValue {
  float f;
  double d;
  s64 i;
  float vec3f[3];
  double vec3d[3];
};

std::string controlParameterValueToString(ControlParameterValue v,
                                          ControlParameterValueKind kind);

class ControlParameter;

/*!
 * ControlParameterCollections contains a map of all the control parameters.
 */
class ControlParameterCollection {
 public:                                    //传入参数为变量名name
  explicit ControlParameterCollection(const std::string& name) : _name(name) {}               //禁止隐式构造，必须全参数

  /*!
   * Use this to add a parameter for the first time in the
   * RobotControlParameters or SimulatorControlParameters. This should only be
   * used during initialization of a ControlParameter.
   *
   * Throws exception if you try to add a parameter twice.
   */
  void addParameter(ControlParameter* param, const std::string& name) {               //添加一个名为name的变量，并为其添加值
    if (mapContains(_map, name)) {
      printf(
          "[ERROR] ControlParameterCollection %s: tried to add parameter %s "
          "twice!\n",
          _name.c_str(), name.c_str());                       //通过string类对象的成员函数c_str()把string 对象转换成c中的字符串样式
      throw std::runtime_error("Control parameter error [" + _name + "]: parameter " + name + " appears twice!");
    }
    _map[name] = param;                     //仍然完成值替换
  }

  /*!
   * Lookup a control parameter by its name.
   * This does not modify the set field of the control parameter!           the set field of：集合域
   *
   * Throws exception if parameter isn't found
   */
  ControlParameter& lookup(const std::string& name) {         //寻找索引为name的变量，并返回其值
    if (mapContains(_map, name)) {
      return *_map[name];          //此处返回_map容器中索引为name的一项的值，该值类型为控制参数指针，此处加*返回值为具体的control parameter
    } else {
      throw std::runtime_error("Control parameter " + name +
                               " wasn't found in parameter collection " +
                               _name);
    }
  }

  std::string
  printToIniString();  //!< print all control parameters in the INI file format
  std::string printToYamlString();  //!< print all control parameters in the
                                    //!< YAML file format
  bool checkIfAllSet();  //!< are all the control parameters initialized?
  void clearAllSet();
  void deleteAll();

  std::map<std::string, ControlParameter*> _map;      //定义_map对象，string作为索引，值为ControlParameter*

 private:
  std::string _name;
};

/*!
 * A single control parameter.  Note that this representation doesn't store the value, just a pointer
 * to an existing variable.
 */
class ControlParameter {
 public:

  /*!
   * Construct control parameter for a double
   * @param name : name of parameter              参数名称（字符串类型）
   * @param value : reference to value            参数值
   * @param collection : collection to add to     
   * @param units : name of units
   */
  ControlParameter(const std::string& name, double& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.d = &value;
    _kind = ControlParameterValueKind::DOUBLE;
    collection.addParameter(this, name);
  }

  /*!
   * Construct control parameter for a float
   * @param name : name of parameter
   * @param value : reference to value
   * @param collection : collection to add to
   * @param units : name of units
   */
  ControlParameter(const std::string& name, float& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.f = &value;
    _kind = ControlParameterValueKind::FLOAT;
    collection.addParameter(this, name);
  }

  /*!
   * Construct control parameter for an s64
   * @param name : name of parameter
   * @param value : reference to value
   * @param collection : collection to add to
   * @param units : name of units
   */
  ControlParameter(const std::string& name, s64& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.i = &value;
    _kind = ControlParameterValueKind::S64;
    collection.addParameter(this, name);
  }

  /*!
   * Construct control parameter for a list of 3 floats
   * @param name : name of parameter
   * @param value : reference to value
   * @param collection : collection to add to
   * @param units : name of units
   */
  ControlParameter(const std::string& name, Vec3<float>& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.vec3f = value.data();
    _kind = ControlParameterValueKind::VEC3_FLOAT;
    collection.addParameter(this, name);
  }

  /*!
   * Construct control parameter for a list of 3 doubles
   * @param name : name of parameter
   * @param value : reference to value
   * @param collection : collection to add to
   * @param units : name of units
   */
  ControlParameter(const std::string& name, Vec3<double>& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.vec3d = value.data();
    _kind = ControlParameterValueKind::VEC3_DOUBLE;
    collection.addParameter(this, name);
  }

  /*!
   * Construct control parameter for a given type without
   * adding it to a collection
   * @param name : name of parameter
   * @param kind : type of data to be stored
   */
  ControlParameter(const std::string& name, ControlParameterValueKind kind) {
    _name = name;
    truncateName();
    _kind = kind;
    _value.vec3d = (double*)&_staticValue;
  }

  /*!
   * Make sure that the control parameter name is short enough to fit
   * in the control parameter request message.
   */
  void truncateName() {
    if (_name.length() > CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH) {
      printf("[Error] control parameter name %s is too long, shortening to ", _name.c_str());
      _name.resize(CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH - 1);
      printf("%s\n", _name.c_str());
    }
  }

  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeDouble(double d) {
    if (_kind != ControlParameterValueKind::DOUBLE) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a double!");
    }
    _set = true;
    *_value.d = d;
  }


  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeFloat(float f) {
    if (_kind != ControlParameterValueKind::FLOAT) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a float!");
    }
    _set = true;
    *_value.f = f;
  }

  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeInteger(s64 i) {
    if (_kind != ControlParameterValueKind::S64) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as an integer!");
    }
    _set = true;
    *_value.i = i;
  }

  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeVec3f(const Vec3<float>& v) {
    if (_kind != ControlParameterValueKind::VEC3_FLOAT) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a vector3f");
    }
    _set = true;
    _value.vec3f[0] = v[0];
    _value.vec3f[1] = v[1];
    _value.vec3f[2] = v[2];
  }

  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeVec3d(const Vec3<double>& v) {
    if (_kind != ControlParameterValueKind::VEC3_DOUBLE) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a vector3d");
    }
    _set = true;
    _value.vec3d[0] = v[0];
    _value.vec3d[1] = v[1];
    _value.vec3d[2] = v[2];
  }

  /*!
   * Set a control parameter by value.
   * Performs type checking
   * @param value : value to set
   * @param kind : kind of the value
   */
  void set(ControlParameterValue value, ControlParameterValueKind kind) {
    if (kind != _kind) {
      throw std::runtime_error("Control parameter type mismatch in set");
    }
    switch (kind) {
      case ControlParameterValueKind::FLOAT:
        *_value.f = value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
        *_value.d = value.d;
        break;
      case ControlParameterValueKind::S64:
        *_value.i = value.i;
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        _value.vec3f[0] = value.vec3f[0];
        _value.vec3f[1] = value.vec3f[1];
        _value.vec3f[2] = value.vec3f[2];
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        _value.vec3d[0] = value.vec3d[0];
        _value.vec3d[1] = value.vec3d[1];
        _value.vec3d[2] = value.vec3d[2];
        break;
      default:
        throw std::runtime_error("Control parameter invalid kind in set");
    }
    _set = true;
  }

  /*!
   * Get the value of a control parameter.  Does type checking - you must provide
   * the correct type.
   * @param kind : the kind of the control parameter
   * @return the value of the control parameter
   */
  ControlParameterValue get(ControlParameterValueKind kind) {
    ControlParameterValue value;
    if (kind != _kind) {
      throw std::runtime_error("Control parameter type mismatch in get");
    }
    switch (_kind) {
      case ControlParameterValueKind::FLOAT:
        value.f = *_value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
        value.d = *_value.d;
        break;
      case ControlParameterValueKind::S64:
        value.i = *_value.i;
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        value.vec3f[0] = _value.vec3f[0];
        value.vec3f[1] = _value.vec3f[1];
        value.vec3f[2] = _value.vec3f[2];
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        value.vec3d[0] = _value.vec3d[0];
        value.vec3d[1] = _value.vec3d[1];
        value.vec3d[2] = _value.vec3d[2];
        break;
      default:
        throw std::runtime_error("Control parameter invalid kind in get");
    }
    return value;
  }

  /*!
   * Convert the value to a string that works in a YAML file
   */
  std::string toString() {
    std::string result;
    switch (_kind) {
      case ControlParameterValueKind::DOUBLE:
        result += numberToString(*_value.d);
        break;
      case ControlParameterValueKind::FLOAT:
        result += numberToString(*_value.f);
        break;
      case ControlParameterValueKind::S64:
        result += std::to_string(*_value.i);
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        result += "[";
        result += numberToString(_value.vec3f[0]) + ", ";
        result += numberToString(_value.vec3f[1]) + ", ";
        result += numberToString(_value.vec3f[2]) + "]";
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        result += "[";
        result += numberToString(_value.vec3d[0]) + ", ";
        result += numberToString(_value.vec3d[1]) + ", ";
        result += numberToString(_value.vec3d[2]) + "]";
        break;
      default:
        result += "<unknown type " + std::to_string((u32)(_kind)) +
                  "> (add it yourself in ControlParameters.h!)";
        break;
    }
    return result;
  }

  /*!
   * Set the value of a control parameter from a string.
   * Numbers are just numbers.
   * Vec3's are like [1,2,3]
   * @param value : the string representing the value
   * @return if the set was successful
   */
  bool setFromString(const std::string& value) {
    switch (_kind) {
      case ControlParameterValueKind::DOUBLE:
        *_value.d = std::stod(value);
        break;
      case ControlParameterValueKind::FLOAT:
        *_value.f = std::stof(value);
        break;
      case ControlParameterValueKind::S64:
        *_value.i = std::stoll(value);
        break;
      case ControlParameterValueKind::VEC3_FLOAT: {
        Vec3<float> v = stringToVec3<float>(value);
        _value.vec3f[0] = v[0];
        _value.vec3f[1] = v[1];
        _value.vec3f[2] = v[2];
      } break;
      case ControlParameterValueKind::VEC3_DOUBLE: {
        Vec3<double> v = stringToVec3<double>(value);
        _value.vec3d[0] = v[0];
        _value.vec3d[1] = v[1];
        _value.vec3d[2] = v[2];
      } break;
      default:
        return false;
    }
    return true;
  }

  bool _set = false;
  ControlParameterValuePtr _value;
  std::string _name;
  std::string _units;
  ControlParameterValueKind _kind;
  ControlParameterValue _staticValue;

 private:
};

/*!
 * Parent class for groups of parameters
 * RobotParameters and SimulatorParameters inherit from this class.                   inherit：继承
 * This will track if all parameters are initialized so you are sure
 * that the robot has received all parameters before starting
 */
class ControlParameters {                                                                     
 public:
  /*!
   * Construct a control parameter group
   * @param name : Each control parameter group must have a unique name so the ini files don't
   * mixed up
   */
  ControlParameters(const std::string& name) : collection(name), _name(name) {}

  /*!
   * If true, all parameters have been initialized in one way or another
   */
  bool isFullyInitialized() { return collection.checkIfAllSet(); }

  /*!
   * Directly initialize a given control parameter by name.
   * @param name : name of parameter to initialize
   * @param d : double value to initialize with
   */
  void initializeDouble(const std::string& name, double d) {
    collection.lookup(name).initializeDouble(d);
  }

  /*!
   * Directly initialize a given control parameter by name.
   * @param name : name of parameter to initialize
   * @param f : float value to initialize with
   */
  void initializeFloat(const std::string& name, float f) {
    collection.lookup(name).initializeFloat(f);
  }

  /*!
   * Directly initialize a given control parameter by name.
   * @param name : name of parameter to initialize
   * @param i : s64 value to initialize with
   */
  void initializeInteger(const std::string& name, s64 i) {
    collection.lookup(name).initializeInteger(i);
  }

  /*!
   * Directly initialize a given control parameter by name.
   * @param name : name of parameter to initialize
   * @param v : list of 3 floats value to initialize with
   */
  void initializeVec3f(const std::string& name, Vec3<float>& v) {
    collection.lookup(name).initializeVec3f(v);
  }

  /*!
   * Directly initialize a given control parameter by name.
   * @param name : name of parameter to initialize
   * @param v : list of 3 double value to initialize with
   */
  void initializeVec3d(const std::string& name, Vec3<double>& v) {
    collection.lookup(name).initializeVec3d(v);
  }

  /*!
   * Lock the access mutex.  Control parameters are shared between UI, LCM, simulator, and robot threads
   */
  void lockMutex() { _mutex.lock(); }

  /*!
   * Unlock the access mutex. Control parameters are shared between UI, LCM, simulator, and robot threads
   */
  void unlockMutex() { _mutex.unlock(); }

  void writeToIniFile(const std::string& path);
  void initializeFromIniFile(const std::string& path);
  void initializeFromYamlFile(const std::string& path);
  void defineAndInitializeFromYamlFile(const std::string& path);

  void writeToYamlFile(const std::string& path);

  std::string generateUnitializedList();

  ControlParameterCollection collection;

 protected:
  std::string _name;
  std::mutex _mutex;
};

#endif  // PROJECT_CONTROLPAREMETERS_H
