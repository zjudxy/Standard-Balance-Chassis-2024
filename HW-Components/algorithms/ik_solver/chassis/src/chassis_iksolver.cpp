/** 
 * @file      chassis_iksolver.cpp
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
 * | 2.1.2 | 2024-02-24 | ZhouShichan | ChassisIKSolver 的成员函数 getThetaVelRefAll getIsNoSideSlipAll 获取数据异常 |
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
/* Includes ------------------------------------------------------------------*/

#include "chassis_iksolver.hpp"

#include "base.hpp"
#include "chassis_iksolver_math.hpp"
namespace hello_world
{

namespace chassis_ik_solver
{

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported function definitions ---------------------------------------------*/

void MoveVec::rotate(float ang, MoveVec* res_ptr) const
{
  if (res_ptr == nullptr) {
    return;
  }

  float c, s;
  _sin_cos(ang, &s, &c);

  res_ptr->x() = c * x() + s * y();
  res_ptr->y() = -s * x() + c * y();
  res_ptr->w() = w();  // w is not changed by rotation.
}

float PosVec::norm() const { return _sqrt(_vec_dot(data_, data_, 2)); }

float PosVec::ang() const { return _atan2(y(), x()); }

IkSolveStatus FixedStandardWheel::ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  if (params_.radius < 0.001) {
    status = (IkSolveStatus)(status | IkSolveStatus::kRadiusTooSmall);
    params_.radius = 0.001;
  }

  float aby = params_.theta_vel_fdb + M_PI_2;
  float by = aby - alpha_;
  float sin_aby = 0, cos_aby = 0, sin_by = 0, cos_by = 0;
  _sin_cos(aby, &sin_aby, &cos_aby);
  _sin_cos(by, &sin_by, &cos_by);

  float J1[3] = {sin_aby, -cos_aby, -l_ * cos_by};
  float C1[3] = {cos_aby, sin_aby, l_ * sin_by};

  // 无侧滑约束求解
  float no_side_slip_val = _vec_dot(C1, v.vec(), 3);
  iksolve_res_.is_no_side_slip = (-0.001f < no_side_slip_val) && (no_side_slip_val < 0.001f);
  iksolve_res_.theta_vel_ref = params_.theta_vel_fdb;

  // 滚动约束求解
  float rot_spd = _vec_dot(J1, v.vec(), 3) / params_.radius;
  iksolve_res_.rot_spt = rot_spd;
  // 记录结果
  if (res_ptr != nullptr) {
    *res_ptr = iksolve_res_;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kFailReturn);
  }
  return status;
};

IkSolveStatus SwedishWheel::ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  if (params_.radius < 0.001) {
    status = (IkSolveStatus)(status | IkSolveStatus::kRadiusTooSmall);
    params_.radius = 0.001;
  }

  float aby = params_.theta_vel_fdb + M_PI_2 + getGamma();
  float by = aby - alpha_;
  float sin_aby = 0, cos_aby = 0, sin_by = 0, cos_by = 0;
  _sin_cos(aby, &sin_aby, &cos_aby);
  _sin_cos(by, &sin_by, &cos_by);

  float J1[3] = {sin_aby, -cos_aby, -l_ * cos_by};
  // float C1[3] = {cos_aby, sin_aby, l_ * sin_by};

  // 无侧滑约束求解
  // float no_side_slip_val = _vec_dot(C1, v.vec(), 3);
  iksolve_res_.is_no_side_slip = true;
  iksolve_res_.theta_vel_ref = params_.theta_vel_fdb;

  // 滚动约束求解
  float cos_gamma = _cos(getGamma());
  float rot_spd = 0;
  if (fabs(cos_gamma) > 0.001) {
    rot_spd = _vec_dot(J1, v.vec(), 3) / (params_.radius * cos_gamma);
  }
  iksolve_res_.rot_spt = rot_spd;
  // 记录结果
  if (res_ptr != nullptr) {
    *res_ptr = iksolve_res_;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kFailReturn);
  }
  return status;
};

IkSolveStatus SteeredStandardWheel::ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  if (theta_vel_fdb_ptr != nullptr) {
    params_.theta_vel_fdb = *theta_vel_fdb_ptr;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kLeakVelAngFdb);
  }

  if (params_.radius < 0.001) {
    status = (IkSolveStatus)(status | IkSolveStatus::kRadiusTooSmall);
    params_.radius = 0.001;
  }

  float a = alpha_;
  float beta_fdb = getBeta(params_.theta_vel_fdb);
  float sin_a, cos_a;
  float sin_b_fdb, cos_b_fdb;
  _sin_cos(a, &sin_a, &cos_a);
  _sin_cos(beta_fdb, &sin_b_fdb, &cos_b_fdb);
  float var1 = v.x() * cos_a + v.y() * sin_a;
  float var2 = v.x() * sin_a - v.y() * cos_a - l_ * v.w();

  // 无侧滑约束求解
  float beta_ref = 0;
  if (params_.opt_mask & kKeepLastThetaVelRefWhen0) {
    beta_ref = getBeta(iksolve_res_.theta_vel_ref);
  }

  if (!IsZeroVec(v.vec(), 3)) {
    beta_ref = _atan2(var1, var2);
  }
  float theta_vel_delta = 0.0f;
  float rot_spd_dir = 1.0f;
  if (params_.opt_mask & kMinThetaVelDelta) {
    theta_vel_delta = beta_ref - beta_fdb;
    if (fabsf(theta_vel_delta) / M_PI_2 > 1) {
      beta_ref += PI;
      rot_spd_dir = -1.0f;
    }
  }
  iksolve_res_.theta_vel_ref = NormPeriodData(-PI, PI, beta_ref + alpha_ - M_PI_2);

  float no_side_slip_val = var1 * cos_b_fdb - var2 * sin_b_fdb;
  iksolve_res_.is_no_side_slip = (-0.001f < no_side_slip_val) && (no_side_slip_val < 0.001f);

  float sin_b, cos_b;
  float rot_spd = 0;

  if (params_.opt_mask & OptMask::kUseThetaVelFdb) {
    sin_b = sin_b_fdb, cos_b = cos_b_fdb;
    rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
    rot_spd *= rot_spd_dir;
    // ! 可能还需要反向
  } else {
    _sin_cos(beta_ref, &sin_b, &cos_b);
    rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
    if (params_.opt_mask & OptMask::kCosRotSpd) {
      rot_spd *= _cos(theta_vel_delta);
    } else {
      rot_spd *= rot_spd_dir;
    }
  }
  iksolve_res_.rot_spt = rot_spd;

  // TODO(ZhouShichan): 之后加一个启停优化
  // 记录结果
  if (res_ptr != nullptr) {
    *res_ptr = iksolve_res_;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kFailReturn);
  }
  return status;
};
IkSolveStatus SphericalWheel::ikSolve(const MoveVec& v, IkSolveRes* res_ptr, const float* theta_vel_fdb_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  if (theta_vel_fdb_ptr != nullptr) {
    params_.theta_vel_fdb = *theta_vel_fdb_ptr;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kLeakVelAngFdb);
  }

  if (params_.radius < 0.001) {
    status = (IkSolveStatus)(status | IkSolveStatus::kRadiusTooSmall);
    params_.radius = 0.001;
  }

  float a = alpha_;
  float beta_fdb = getBeta(params_.theta_vel_fdb);
  float sin_a, cos_a;
  float sin_b_fdb, cos_b_fdb;
  _sin_cos(a, &sin_a, &cos_a);
  _sin_cos(beta_fdb, &sin_b_fdb, &cos_b_fdb);
  float var1 = v.x() * cos_a + v.y() * sin_a;
  float var2 = v.x() * sin_a - v.y() * cos_a - l_ * v.w();

  // 无侧滑约束求解
  float beta_ref = 0;
  if (!IsZeroVec(v.vec(), 3)) {
    beta_ref = _atan2(var1, var2);
  }
  float theta_vel_delta = 0.0f;
  float rot_spd_dir = 1.0f;
  if (params_.opt_mask & kMinThetaVelDelta) {
    theta_vel_delta = beta_ref - beta_fdb;
    if (fabsf(theta_vel_delta) / M_PI_2 > 1) {
      beta_ref += PI;
      rot_spd_dir = -1.0f;
    }
  }
  iksolve_res_.theta_vel_ref = NormPeriodData(-PI, PI, beta_ref + alpha_ - M_PI_2);

  if (params_.opt_mask & OptMask::kAsServer) {
    iksolve_res_.is_no_side_slip = true;
  } else {
    float no_side_slip_val = var1 * cos_b_fdb - var2 * sin_b_fdb;
    iksolve_res_.is_no_side_slip = (-0.001f < no_side_slip_val) && (no_side_slip_val < 0.001f);
  }

  float sin_b, cos_b;
  float rot_spd = 0;

  if (params_.opt_mask & OptMask::kUseThetaVelFdb) {
    sin_b = sin_b_fdb, cos_b = cos_b_fdb;
    rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
    rot_spd *= rot_spd_dir;
    // ! 可能还需要反向
  } else {
    _sin_cos(beta_ref, &sin_b, &cos_b);
    rot_spd = (var1 * sin_b + var2 * cos_b) / params_.radius;
    if (params_.opt_mask & OptMask::kCosRotSpd) {
      rot_spd *= _cos(theta_vel_delta);
    } else {
      rot_spd *= rot_spd_dir;
    }
  }
  iksolve_res_.rot_spt = rot_spd;

  // TODO(ZhouShichan): 之后加一个启停优化
  // 记录结果
  if (res_ptr != nullptr) {
    *res_ptr = iksolve_res_;
  } else {
    status = (IkSolveStatus)(status | IkSolveStatus::kFailReturn);
  }
  return status;
};

IkSolveStatus ChassisIkSolver::solve(const MoveVec& v, float* theta_vel_fdbs_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  size_t idx = 0;

  vel_r_ = v;

  for (auto wheel : wheel_list_) {
    IkSolveRes res;

    float* theta_vel_fdb_ptr = theta_vel_fdbs_ptr == nullptr ? nullptr : theta_vel_fdbs_ptr + idx;

    IkSolveStatus wheel_status = wheel->ikSolve(v, &res, theta_vel_fdb_ptr);

    status = IkSolveStatus(status | wheel_status);

    idx++;
  }
  return status;
};

IkSolveStatus ChassisIkSolver::solve(const MoveVec& v, float theta_i2r, float* theta_vel_fdbs_ptr)
{
  IkSolveStatus status = IkSolveStatus::kIkSolveOk;

  MoveVec v_r;
  v.rotate(theta_i2r, &v_r);

  size_t idx = 0;

  vel_r_ = v_r;

  for (auto wheel : wheel_list_) {
    IkSolveRes res;

    float* theta_vel_fdb_ptr = theta_vel_fdbs_ptr == nullptr ? nullptr : theta_vel_fdbs_ptr + idx;

    IkSolveStatus wheel_status = wheel->ikSolve(v_r, &res, theta_vel_fdb_ptr);

    status = IkSolveStatus(status | wheel_status);

    idx++;
  }
  return status;
};

bool ChassisIkSolver::append(WheelType wheel_type, const WheelParams& params)
{
  Wheel* wheel_ptr = CreateWheel(wheel_type, params);
  if (wheel_ptr == nullptr) {
    return false;
  } else {
    wheel_ptr->setCenterPos(&control_center_);
    wheel_list_.push_back(wheel_ptr);
    return true;
  }
};

void ChassisIkSolver::erase_tail()
{
  if (size()) {
    auto wheel_ptr = wheel_list_.back();
    delete wheel_ptr;
    wheel_ptr = nullptr;
    wheel_list_.pop_back();
  }
};

void ChassisIkSolver::clear()
{
  while (size()) {
    erase_tail();
  }
};

const Wheel* ChassisIkSolver::getWheel(size_t idx) const
{
  if (idx >= size()) {
    return nullptr;
  }

  auto iter = wheel_list_.begin();
  std::advance(iter, idx);
  return *iter;
};

Wheel* ChassisIkSolver::getWheel(size_t idx)
{
  if (idx >= size()) {
    return nullptr;
  }

  auto iter = wheel_list_.begin();
  std::advance(iter, idx);
  return *iter;
};

void ChassisIkSolver::setCenterPos(float x, float y)
{
  control_center_.x() = x;
  control_center_.y() = y;
  for (auto wheel : wheel_list_) {
    wheel->setCenterPos(&control_center_);
  }
};

const IkSolveRes& ChassisIkSolver::getIkSolveRes(size_t idx) const
{
  HW_ASSERT(size() > 0, "ChassisIkSolver must has a wheel when use this function, but %d now", size());
  const Wheel* wheel_ptr = getWheel(idx);
  return wheel_ptr->getIkSolveRes();
};

bool ChassisIkSolver::getIsNoSideSlip(size_t idx) const { return getIkSolveRes(idx).is_no_side_slip; };

float ChassisIkSolver::getRotSpd(size_t idx) const { return getIkSolveRes(idx).rot_spt; };

float ChassisIkSolver::getThetaVelRef(size_t idx) const { return getIkSolveRes(idx).theta_vel_ref; };

IkSolveStatus ChassisIkSolver::getIkSolveResAll(IkSolveRes* iksolve_ress_ptr) const
{
  if (iksolve_ress_ptr == nullptr) {
    return IkSolveStatus::kFailReturn;
  }
  for (auto i : wheel_list_) {
    *iksolve_ress_ptr = i->getIkSolveRes();
    iksolve_ress_ptr++;
  }
  return IkSolveStatus::kIkSolveOk;
};

IkSolveStatus ChassisIkSolver::getRotSpdAll(float* rot_spds_ptr) const
{
  if (rot_spds_ptr == nullptr) {
    return IkSolveStatus::kFailReturn;
  }
  for (auto i : wheel_list_) {
    *rot_spds_ptr = i->getRotSpd();
    rot_spds_ptr++;
  }
  return IkSolveStatus::kIkSolveOk;
};

IkSolveStatus ChassisIkSolver::getThetaVelRefAll(float* theta_vel_refs_ptr) const
{
  if (theta_vel_refs_ptr == nullptr) {
    return IkSolveStatus::kFailReturn;
  }
  for (auto i : wheel_list_) {
    *theta_vel_refs_ptr = i->getThetaVelRef();
    theta_vel_refs_ptr++;
  }
  return IkSolveStatus::kIkSolveOk;
};

IkSolveStatus ChassisIkSolver::getIsNoSideSlipAll(bool* is_no_side_slips_ptr) const
{
  if (is_no_side_slips_ptr == nullptr) {
    return IkSolveStatus::kFailReturn;
  }

  for (auto i : wheel_list_) {
    *is_no_side_slips_ptr = i->getIsNoSideSlip();
    is_no_side_slips_ptr++;
  }
  return IkSolveStatus::kIkSolveOk;
};

Wheel* CreateWheel(WheelType wheel_type, const WheelParams& params)
{
  switch (wheel_type) {
    case WheelType::kFixedStandardWheel:
      return (Wheel*)new FixedStandardWheel(params);
      break;
    case WheelType::kSteeredStandardWheel:
      return (Wheel*)new SteeredStandardWheel(params);
      break;
    case WheelType::kSwedishWheel:
      return (Wheel*)new SwedishWheel(params);
      break;
    case WheelType::kMecanumWheel:
      return (Wheel*)new MecanumWheel(params);
      break;
    case WheelType::kOmniWheel:
      return (Wheel*)new OmniWheel(params);
      break;
    case WheelType::kSphericalWheel:
      return (Wheel*)new SphericalWheel(params);
      break;
    default:
      return nullptr;
  };
};
/* Private function definitions ----------------------------------------------*/

}  // namespace chassis_ik_solver

}  // namespace hello_world
