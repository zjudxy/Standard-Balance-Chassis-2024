/** 
 * @file      chassis_iksolver.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2023-12-13
 * @brief     基于约束的轮式移动机器人运动学逆求解器
 * @details   标准轮是各类车轮(转向轮、swedish轮等)的基础，且可以理想化为一个圆形。通过该圆形施加滚动约束和无侧滑约束，可以求解出满足底盘坐标系下速度指令的各个车轮的转速。
 * @par last edit time  2024-02-24
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   2.1.1
 * 
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 * 
 * @attention 只考虑底盘轮组与单一平面接触的情况
 * 
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 2.0.0 | 2024-01-24 | ZhouShichan | 1. 首次完成 |
 * | 2.1.0 | 2024-02-11 | ZhouShichan | 对求解方式进行了优化，改变了容器实现方式，更改部分接口函数 |
 * | 2.1.1 | 2024-02-13 | ZhouShichan | 修复固定标准轮、Swedish 轮求解异常；修复 Swedish 轮 gamma 值初始化异常； |
 * | 2.1.2 | 2024-02-24 | ZhouShichan | 修复 ChassisIKSolver 的成员函数 getThetaVelRefAll getIsNoSideSlipAll 获取数据异常 |
 * 
 * @par v2.1.1
 * 1. 修复固定标准轮、Swedish 轮求解异常
 * 2. 修复 Swedish 轮 gamma 值初始化异常
 * 
 * @par v2.1.0
 * 1. 各种轮子的求解方式相互独立，对转向标准轮和球轮的求解添加了转角优化
 * 2. 链表容器更改为 std::list ，在 Ozone 中不再支持直接查看内部数据，需要通过额外的全局变量查看
 * 3. ChassisIKSolver 添加底盘旋转中心的设置函数，求解函数不再支持通过指针返回逆解结果，需要通过其他函数单独获取
 * 
 * @par 相关链接
 * [内部飞书](https://g6ursaxeei.feishu.cn/wiki/wikcnob2XRghAIPINsoeG6AYy5f)
 * [Github Wiki](https://zju-helloworld.github.io/Wiki/%E7%BB%84%E4%BB%B6%E8%AF%B4%E6%98%8E/%E6%9C%BA%E5%99%A8%E4%BA%BA%E9%80%9A%E7%94%A8%E7%BB%84%E4%BB%B6/%E7%AE%97%E6%B3%95/%E8%BD%AE%E5%BC%8F%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%BA%95%E7%9B%98%E7%BA%A6%E6%9D%9F%E6%B1%82%E8%A7%A3/) 推荐
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_IKSOLVER_HPP_
#define HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <list>

#include "base.hpp"
#include "system.hpp"
namespace hello_world
{
namespace chassis_ik_solver
{

/* Exported macro ------------------------------------------------------------*/

/**
 * @brief 车轮类型
 * 
 * 这个枚举定义了所有可能的车轮类型，包括：
 * 
 * - 固定标准轮
 * - 转向标准轮
 * - 脚轮(解算麻烦又用不到，暂时不实现)
 * - 瑞典轮
 *  - 麦克纳姆轮，一种特殊的瑞典轮，且 gamma = pi / 4
 *  - 全向轮，一种特殊的瑞典轮，且 gamma = 0
 * - 球轮
 */
enum WheelType {
  kNotImplementedWheel = 0u,  ///< 未实现
  kFixedStandardWheel,        ///< 固定标准轮
  kSteeredStandardWheel,      ///< 转向标准轮
  kCastorWheel,               ///< 脚轮
  kSwedishWheel,              ///< 瑞典轮
  kMecanumWheel,              ///< 麦克纳姆轮(瑞典轮的一种)
  kOmniWheel,                 ///< 全向轮(瑞典轮的一种)
  kSphericalWheel,            ///< 球轮
};
// 逆解结果
enum IkSolveStatus {
  kIkSolveOk = 0u,            ///< 逆解求解成功
  kLeakVelAngFdb = 1u << 1,   ///< 缺少速度角反馈
  kRadiusTooSmall = 1u << 2,  ///< 车轮半径过小
  kLengthTooSmall = 1u << 3,  ///< 车轮中心到转动中心的距离过小
  kFailReturn = 1u << 4,      ///< 传递结果失败
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class MoveVec
{
 public:
  explicit MoveVec(float x = 0, float y = 0, float w = 0) { data_[0] = x, data_[1] = y, data_[2] = w; };

  float& x() { return data_[0]; };
  float& y() { return data_[1]; };
  float& w() { return data_[2]; };
  float x() const { return data_[0]; };
  float y() const { return data_[1]; };
  float w() const { return data_[2]; };

  float* vec() { return data_; };
  const float* vec() const { return data_; };

  float& operator[](size_t idx) { return data_[idx]; };
  const float& operator[](size_t idx) const { return data_[idx]; };

  void rotate(float ang, MoveVec* res_ptr = nullptr) const;

 private:
  float data_[3];  // 矢量数据 [x, y, w]
};

class PosVec
{
 public:
  explicit PosVec(float x = 0, float y = 0) { data_[0] = x, data_[1] = y; };

  float& x() { return data_[0]; };
  float& y() { return data_[1]; };
  float x() const { return data_[0]; };
  float y() const { return data_[1]; };

  float* vec() { return data_; };
  const float* vec() const { return data_; };

  float operator[](size_t idx) const { return data_[idx]; };
  float& operator[](size_t idx) { return data_[idx]; };

  float norm() const;
  float ang() const;
  float dist(const PosVec& other) const { return (*this - other).norm(); };

  PosVec operator+(const PosVec& other) const { return PosVec(data_[0] + other.data_[0], data_[1] + other.data_[1]); };
  PosVec operator-(const PosVec& other) const { return PosVec(data_[0] - other.data_[0], data_[1] - other.data_[1]); };

  PosVec& operator+=(const PosVec& other)
  {
    data_[0] += other.data_[0];
    data_[1] += other.data_[1];
    return *this;
  };
  PosVec& operator-=(const PosVec& other)
  {
    data_[0] -= other.data_[0];
    data_[1] -= other.data_[1];
    return *this;
  };

 private:
  float data_[2];  ///< 位置矢量数据 [x, y]
};

/** 
 * @note 如何确定车轮转动矢量：
 * @note 1. 车轮正向旋转时，地面接触点的线速度方向的反方向；
 * @note 2. 车轮法向量与底盘坐标系Z轴叉乘得到的向量即为车轮转动矢量。
 * @attention 对于转向标准轮、球轮、脚轮的逆解需要实时更新进行优化。
 * @attention 对于麦克纳姆轮，由于代码中 gamma 是固定的 pi/4，因此需要特别确定其转向角度使其 gamma 值在车辆上满足 pi/4。
 */
struct WheelParams {
  uint32_t opt_mask = 0u;            ///< 优化选项掩码
  float theta_vel_fdb = 0.0f;        ///< 反馈的车轮转向角度
  float d_castor = 0.0f;             ///< 车轮中心到车轮垂直转轴的距离，单位：m
  float gamma = 0.0f;                ///< 辊子转轴与车轮转轴之间的最小夹角，单位：rad
  float radius = 0.0f;               ///< 车轮半径，单位：m
  PosVec wheel_pos = PosVec(0, 0);   ///< 车轮垂直转轴在底盘坐标系下的位置，单位：m
  PosVec* center_pos_ptr = nullptr;  ///< 底盘旋转中心位置，单位：m
};

struct IkSolveRes {
  bool is_no_side_slip = false;  ///< 此次运动指令是否满足无侧滑约束
  float rot_spt = 0.0f;          ///< 逆解的车轮转动速度，单位：rad/s
  float theta_vel_ref = 0.0f;    ///< 逆解的车轮转向角度，单位：rad
};

class Wheel : public MemMang
{
 public:
  enum OptMask { kOptNone = 0u };

  explicit Wheel(const WheelParams& params) { setParams(params); };
  explicit Wheel(const Wheel& wheel) { setParams(wheel.params_); };

  virtual IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr = nullptr) = 0;

  // 逆解结果获取接口
  /**
   * @brief 获取逆运动学求解结果的引用。
   *
   * @return const IkSolveRes& 逆运动学求解结果的常量引用。
   */
  const IkSolveRes& getIkSolveRes() const { return iksolve_res_; };

  /**
   * @brief 获取指示是否无侧滑条件的标志。
   *
   * @return bool 如果没有侧滑发生，则返回true，否则返回false。
   */
  bool getIsNoSideSlip() const { return iksolve_res_.is_no_side_slip; };

  /**
   * @brief 获取旋转速度值。
   *
   * @return float 返回旋转速度(rotational speed)的值。
   */
  float getRotSpd() const { return iksolve_res_.rot_spt; };

  /**
   * @brief 获取参考的角速度值。
   *
   * @return float 返回参考的角速度(theta velocity reference)值。
   */
  float getThetaVelRef() const { return iksolve_res_.theta_vel_ref; };

  // 参数设置接口
  /**
   * @brief 设置轮子参数。
   *
   * @param params 轮子参数的结构体。
   */
  void setParams(const WheelParams& params)
  {
    setOptMask(params.opt_mask);
    setThetaVelFdb(params.theta_vel_fdb);
    setDCaster(params.d_castor);
    setRadius(params.radius);
    setWheelPos(params.wheel_pos);
    setCenterPos(params.center_pos_ptr);
    setGamma(params.gamma);
    setPosCallback();
  };

  /**
   * @brief 设置优化掩码。
   *
   * @param opt_mask 优化选项掩码。
   */
  void setOptMask(uint32_t opt_mask) { params_.opt_mask = opt_mask; };

  /**
   * @brief 设置反馈的速度矢量角。
   *
   * @param theta_vel 反馈的速度矢量角。
   */
  void setThetaVelFdb(float theta_vel) { params_.theta_vel_fdb = theta_vel; };

  /**
   * @brief 设置轮子半径，并确保半径不小于一个预设的最小值。
   *
   * @param radius 要设置的轮子半径。
   */
  void setRadius(float radius) { params_.radius = radius < 0.01 ? 0.01 : radius; };

  /**
   * @brief 设置轮子位置。
   *
   * @param pos 轮子位置的向量。
   */
  void setWheelPos(const PosVec& pos)
  {
    params_.wheel_pos = pos;
    setPosCallback();
  };

  /**
   * @brief 设置底盘转轴的位置指针。
   *
   * @param pos_ptr 指向底盘转轴的位置的指针。
   */
  void setCenterPos(PosVec* pos_ptr)
  {
    params_.center_pos_ptr = pos_ptr;
    setPosCallback();
  };

  /**
   * @brief 设置Gamma值，虚函数，可能在子类中实现具体功能。
   *
   * @param gamma Gamma值。
   */
  virtual void setGamma(float gamma) { params_.gamma = 0; };

  /**
   * @brief 设置轮子垂直旋转轴和轮子中心的距离，虚函数，可能在子类中实现具体功能。
   *
   * @param d 轮子垂直旋转轴和轮子中心的距离。
   */
  virtual void setDCaster(float d) { params_.d_castor = 0; };

  // 参数获取接口
  /**
   * @brief 获取当前设置的轮子参数。
   * @return WheelParams 已设置的轮子参数的结构体。
   */
  WheelParams getParams() const { return params_; };

  /**
   * @brief 获取设置的优化掩码值。
   * @return uint32_t 当前设置的优化掩码。
   */
  uint32_t getOptMask() const { return params_.opt_mask; };

  /**
   * @brief 获取设置的反馈的速度矢量角。
   *
   * @return float 当前设置的反馈的速度矢量角。
   */
  float getThetaVelFdb() const { return params_.theta_vel_fdb; };

  /**
   * @brief 获取轮子垂直旋转轴和轮子中心的距离。
   *
   * @return float 当前设置的轮子垂直旋转轴和轮子中心的距离。
   */
  float getDCaster() const { return params_.d_castor; };

  /**
   * @brief 获取当前 alpha 值。
   *
   * @return float 当前 alpha 值。
   */
  float getAlpha() const { return alpha_; };

  /**
   * @brief 根据传入的速度矢量角计算Beta值。
   *
   * @param theta_vel 速度矢量角。
   * @return float 计算出的Beta值。
  */
  float getBeta(float theta_vel) const { return theta_vel - alpha_ + M_PI_2; };

  /**
   * @brief 获取Gamma值。
   *
   * @return float 当前设置的Gamma值。
   */
  float getGamma() const { return params_.gamma; };

  /**
   * @brief 获取轮子半径。
   *
   * @return float 当前设置的轮子半径。
   */
  float getRadius() const { return params_.radius; };

  /**
   * @brief 获取车轮垂直旋转轴到底盘旋转中心的距离。
   *
   * @return float 车轮垂直旋转轴到底盘旋转中心的距离。
   */
  float getL() const { return l_; };

  /**
   * @brief 获取轮子位置。
   *
   * @return PosVec 当前设置的轮子位置。
   */
  PosVec getWheelPos() const { return params_.wheel_pos; };

  /**
   * @brief 获取指向底盘转轴位置的指针。
   *
   * @return const PosVec* 指向当前设置的底盘转轴位置的指针。
   */
  const PosVec* getCenterPos() const { return params_.center_pos_ptr; };

 protected:
  void setPosCallback()
  {
    PosVec dist = params_.wheel_pos;
    if (params_.center_pos_ptr != nullptr) {
      dist -= *params_.center_pos_ptr;
    }
    alpha_ = dist.ang();
    l_ = dist.norm();
  };

  float alpha_ = 0;  ///< x 轴正方向与底盘旋转中心到车轮垂直旋转轴的有向线段的夹角
  float l_ = 0;      ///< 底盘旋转中心到车轮垂直旋转轴的有向线段长度
  IkSolveRes iksolve_res_ = {.is_no_side_slip = false, .rot_spt = 0, .theta_vel_ref = 0};
  WheelParams params_ = {
      .opt_mask = 0,
      .theta_vel_fdb = 0,
      .d_castor = 0,
      .gamma = 0,
      .radius = 0,
      .wheel_pos = PosVec(0, 0),
      .center_pos_ptr = nullptr,
  };
};

class FixedStandardWheel : public Wheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit FixedStandardWheel(const WheelParams& params) : Wheel(params){};

  virtual IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr = nullptr) override;
};

class SwedishWheel : public Wheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit SwedishWheel(const WheelParams& params) : Wheel(params) { setGamma(params.gamma); };

  virtual IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr = nullptr) override;

  virtual void setGamma(float gamma) override { params_.gamma = gamma; }
};

class MecanumWheel : public SwedishWheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit MecanumWheel(const WheelParams& params) : SwedishWheel(params){};

  virtual void setGamma(float gamma) override { params_.gamma = M_PI_4; }
};

class OmniWheel : public SwedishWheel
{
 public:
  typedef Wheel::OptMask OptMask;

  explicit OmniWheel(const WheelParams& params) : SwedishWheel(params){};

  virtual void setGamma(float gamma) override { params_.gamma = 0; }
};

class SteeredStandardWheel : public Wheel
{
 public:
  enum OptMask {
    kOptNone = 0,
    kUseThetaVelFdb = 1u << 0,            ///< 使用反馈的车轮转角计算车轮转速
    kMinThetaVelDelta = 1u << 1,          ///< 通过转速反向使车轮转角变化最小
    kCosRotSpd = 1u << 2,                 ///< 当使用期望的车轮转角计算车轮转速时，对转速乘以 sin 函数
    kKeepLastThetaVelRefWhen0 = 1u << 3,  ///< 当车轮转速为 0 时，保持上一次的速度矢量角度
  };

  explicit SteeredStandardWheel(const WheelParams& params) : Wheel(params){};

  virtual IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr = nullptr) override;
};

class SphericalWheel : public Wheel
{
 public:
  enum OptMask {
    kOptNone = 0,
    kUseThetaVelFdb = 1u << 0,    ///< 使用反馈的车轮转角计算车轮转速
    kMinThetaVelDelta = 1u << 1,  ///< 通过转速反向使车轮转角变化最小
    kCosRotSpd = 1u << 2,         ///< 当使用期望的车轮转角计算车轮转速时，对转速乘以 sin 函数
    kAsServer = 1u << 3,          ///< 作为随动轮使用
  };
  explicit SphericalWheel(const WheelParams& params) : Wheel(params){};

  virtual IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr = nullptr) override;
};

class CastorWheel : public Wheel
{
 public:
  explicit CastorWheel(const WheelParams& params) : Wheel(params){};

  IkSolveStatus ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr) override;
  virtual void setDCaster(float d) { params_.d_castor = fabsf(d); };

 private:
  float d_beta;
  uint32_t last_tick_;
};

class ChassisIkSolver : public MemMang
{
 public:
  typedef std::list<Wheel*, Allocator<Wheel*>> WheelList;

  explicit ChassisIkSolver(const PosVec& control_center = PosVec(0, 0)) : wheel_list_(WheelList()), control_center_(control_center){};
  ~ChassisIkSolver() { clear(); };

  /**
   * @brief 求解轮式移动机器人的底盘逆向运动学问题
   * 
   * @param v 输入的移动向量
   * @param theta_vel_fdbs_ptr 传入速度矢量角度反馈数组的指针，默认为 nullptr。对于转向标准轮、球轮和脚轮，如果为 nullptr 则意味着缺少实时的速度矢量角度，将采用 0 进行计算。
   * @return IkSolveStatus 返回逆向运动学解的状态。
   */
  IkSolveStatus solve(const MoveVec& v, float* theta_vel_fdbs_ptr);

  /** 
   * @brief 求解轮式移动机器人的底盘逆向运动学问题
   * 
   * @param v 输入的移动向量
   * @param theta_i2r 移动向量的参考坐标系到底盘坐标系的旋转角度，右手定则判定旋转正方向，单位：弧度。
   * @param theta_vel_fdbs_ptr 传入速度矢量角度反馈数组的指针，默认为 nullptr。对于转向标准轮、球轮和脚轮，如果为 nullptr 则意味着缺少实时的速度矢量角度，将采用 0 进行计算。
   * @return IkSolveStatus 返回逆向运动学解的状态。
   */
  IkSolveStatus solve(const MoveVec& v, float theta_i2r, float* theta_vel_fdbs_ptr);
  // 链表操作
  /**
   * @brief 获取轮子链表容器中元素的数量。
   *
   * @return size_t 链表容器中轮子的数量。
   */
  size_t size() const { return wheel_list_.size(); };

  /**
   * @brief 向轮子链表容器末尾添加一个新的轮子，类型和参数由传入参数指定。
   *
   * @param wheel_type 要添加的轮子类型。
   * @param params 要添加的轮子的参数。
   * @return bool 如果添加成功则返回 true，否则返回 false。
   */
  bool append(WheelType wheel_type, const WheelParams& params);

  /**  @brief 从链表容器中删除末尾元素并释放对应内存 */
  void erase_tail();

  /** @brief 清空链表容器并释放所有对应的内存 */
  void clear();

  /**
   * @brief 获取链表容器中指定下标位置的轮子对象的常量指针。
   *
   * 如果下标有效，即在容器大小范围内，则返回指定的轮子对象指针；如果下标无效，则返回 nullptr。
   *
   * @param idx 轮子对象在链表中的下标。从 0 开始计数。
   * @return const Wheel* 指向链表中对应下标的轮子对象的常量指针。
 */
  const Wheel* getWheel(size_t idx) const;

  /**
     * @brief 获取链表容器中指定下标位置的轮子对象的常量指针。
     *
     * 如果下标有效，即在容器大小范围内，则返回指定的轮子对象指针；如果下标无效，则返回 nullptr。
     *
     * @param idx 轮子对象在链表中的下标。从 0 开始计数。
     * @return Wheel* 指向链表中对应下标的轮子对象的指针。
   */
  Wheel* getWheel(size_t idx);

  // 参数接口

  /**
 * @brief 设置底盘旋转中心的位置。
 *
 * @param x 旋转中心的x坐标。
 * @param y 旋转中心的y坐标。
 */
  void setCenterPos(float x, float y);

  /**
 * @brief 获取底盘旋转中心的位置。
 *
 * @return const PosVec& 底盘旋转中心位置的引用。
 */
  const PosVec& getCenterPos() const { return control_center_; };

  /**
 * @brief 获取指定索引对应的逆解算结果。
 *
 * @param idx 逆解算结果的索引。
 * @return const IkSolveRes& 指定索引对应的逆解算结果的引用。
 */
  const IkSolveRes& getIkSolveRes(size_t idx) const;

  /**
 * @brief 获取指定轮子是否无侧滑的状态。
 *
 * @param idx 轮子的索引。
 * @return bool 指定轮子是否无侧滑。
 */
  bool getIsNoSideSlip(size_t idx) const;

  /**
 * @brief 获取指定轮子的旋转速度。
 *
 * @param idx 轮子的索引。
 * @return float 指定轮子的旋转速度。
 */
  float getRotSpd(size_t idx) const;

  /**
 * @brief 获取指定轮子的速度矢量角。
 *
 * @param idx 轮子的索引。
 * @return float 指定轮子的速度矢量角。
 */
  float getThetaVelRef(size_t idx) const;

  /**
 * @brief 获取所有轮子的逆解算结果。
 *
 * @param[out] iksolve_ress_ptr 指向逆解算结果数组的指针。
 * @return IkSolveStatus 执行结果状态。
 */
  IkSolveStatus getIkSolveResAll(IkSolveRes* iksolve_ress_ptr) const;

  /**
 * @brief 获取所有轮子的旋转速度。
 *
 * @param[out] rot_spds_ptr 指向旋转速度数组的指针。
 * @return IkSolveStatus 执行结果状态。
 */
  IkSolveStatus getRotSpdAll(float* rot_spds_ptr) const;

  /**
 * @brief 获取所有轮子的速度矢量角。
 *
 * @param[out] theta_vel_refs_ptr 指向速度矢量角数组的指针。
 * @return IkSolveStatus 执行结果状态。
 */
  IkSolveStatus getThetaVelRefAll(float* theta_vel_refs_ptr) const;

  /**
 * @brief 获取所有轮子是否无侧滑的状态。
 *
 * @param[out] is_no_side_slips_ptr 指向无侧滑状态数组的指针。
 * @return IkSolveStatus 执行结果状态。
 */
  IkSolveStatus getIsNoSideSlipAll(bool* is_no_side_slips_ptr) const;

 private:
  WheelList wheel_list_ = WheelList();    ///< 轮子链表
  PosVec control_center_ = PosVec(0, 0);  ///< 底盘旋转中心位置
  MoveVec vel_r_ = MoveVec(0, 0, 0);      ///< 底盘坐标系下的速度
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
Wheel* CreateWheel(WheelType wheel_type, const WheelParams& params);
}  // namespace chassis_ik_solver

}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_IK_SOLVER_CHASSIS_IKSOLVER_HPP_ */
