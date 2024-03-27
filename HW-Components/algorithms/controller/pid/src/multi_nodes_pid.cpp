/** 
 * @file      multi_nodes_pid.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief     
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * 
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 * 
 * @attention 
 * 
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Includes ------------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#include "assert.hpp"
#include "multi_nodes_pid.hpp"
namespace hello_world
{
namespace pid
{
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void MultiNodesPid::setControllerDims(size_t size)
{
  if (type_ == Type::kMultiNodesPidTypeCascade) {
    ref_dim_ = 1;
  } else if (type_ == Type::kMultiNodesPidTypeParallel) {
    ref_dim_ = size;
  } else {
    HW_ASSERT(0, "Illegal multi nodes pid type %d", type_);
  }
  ffd_dim_ = size;
  ffd_dim_ = 1;
  out_dim_ = 1;
};

/* Exported function definitions ---------------------------------------------*/

MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit)
{
  type_      = type;
  out_limit_ = out_limit;
  setControllerDims(0);
};
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit, size_t size)
{
  type_      = type;
  out_limit_ = out_limit;
  pids_.resize(size);
  setControllerDims(size);
};
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit, const PidList& pids)
{
  type_       = type;
  out_limit_  = out_limit;
  size_t size = 0;
  for (auto iter = pids.begin(); iter != pids.end(); iter++, size++) {
    pids_.push_back(Pid(*iter));
  }
  setControllerDims(size);
};
MultiNodesPid::MultiNodesPid(Type type, OutLimit out_limit, const ParamsList& params_list)
{
  type_       = type;
  out_limit_  = out_limit;
  size_t size = 0;
  for (auto iter = params_list.begin(); iter != params_list.end(); iter++, size++) {
    pids_.push_back(Pid(*iter));
  }
  setControllerDims(size);
};

void MultiNodesPid::push_back(const Pid& pid)
{
  pids_.push_back(Pid(pid));
  setControllerDims(pids_.size());
};

void MultiNodesPid::push_back(const Params& params)
{
  pids_.push_back(Pid(params));
  setControllerDims(pids_.size());
};

void MultiNodesPid::remove(size_t idx)
{
  HW_ASSERT(idx < pids_.size(), "Illegal index %d", idx);
  auto iter = pids_.begin();
  std::advance(iter, idx);
  pids_.erase(iter);
  setControllerDims(pids_.size());
};

MultiNodesPid::Pid& MultiNodesPid::getPidAt(size_t idx)
{
  HW_ASSERT(idx < pids_.size(), "Illegal index %d", idx);
  auto iter = pids_.begin();
  std::advance(iter, idx);
  return *iter;
};

MultiNodesPid::State MultiNodesPid::reset()
{
  for (auto iter = pids_.begin(); iter != pids_.end(); iter++) {
    iter->reset();
  }
  calc_state_ = CalcState::kPidCalcStateNone;
  return State::kControllerStateOk;
};
MultiNodesPid::State MultiNodesPid::calc(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls)
{
  if (type_ == Type::kMultiNodesPidTypeCascade) {
    return calcCascade(ref_ls, fdb_ls, ffd_ls, out_ls);
  } else if (type_ == Type::kMultiNodesPidTypeParallel) {
    return calcParallel(ref_ls, fdb_ls, ffd_ls, out_ls);
  } else {
    HW_ASSERT(0, "Illegal multi nodes pid type %d", type_);
  }
  return State::kControllerStateError;
};
MultiNodesPid::State MultiNodesPid::calcCascade(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls)
{
  HW_ASSERT(type_ == Type::kMultiNodesPidTypeCascade, "Illegal multi nodes pid type %d", type_);
  calc_state_ = CalcState::kPidCalcStateNone;

  if (fdb_ls == nullptr) {
    out_         = 0;
    calc_state_ |= CalcState::kPidCalcStateLackFdb;
    return State::kControllerStateError;
  }

  float ref = 0;
  if (ref_ls != nullptr) {
    ref = ref_ls[0];
  } else {
    calc_state_ |= CalcState::kPidCalcStateLackRef;
  }

  float        out          = 0;
  float        zero_ffd     = 0;
  const float* zero_ffd_ptr = &zero_ffd;

  size_t i = 0;
  for (auto& pid : pids_) {
    // 由于 pid 会记录 前馈值 传入 nullptr ，在此传入一个指向 0 的值
    pid.calc(&ref, &fdb_ls[i], zero_ffd_ptr, &out);
    ref = out;
    i++;
  }

  if (ffd_ls == nullptr) {
    out_ = out_limit_.calc(out);
  } else {
    out_ = out_limit_.calc(out + ffd_ls[0]);
  }

  if (out_ls != nullptr) {
    out_ls[0] = out_;
  } else {
    calc_state_ |= CalcState::kPidCalcStateFailedOut;
  }

  calc_state_ |= CalcState::kPidCalcStateOk;
  return calc_state_ == CalcState::kPidCalcStateOk ? State::kControllerStateOk : State::kControllerStateError;
};
MultiNodesPid::State MultiNodesPid::calcParallel(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls)
{
  HW_ASSERT(type_ == Type::kMultiNodesPidTypeParallel, "Illegal multi nodes pid type %d", type_);

  calc_state_ = CalcState::kPidCalcStateNone;

  if (fdb_ls == nullptr) {
    out_         = 0;
    calc_state_ |= CalcState::kPidCalcStateLackFdb;
    return State::kControllerStateError;
  }

  if (ref_ls == nullptr) {
    calc_state_ |= CalcState::kPidCalcStateLackRef;
  }

  float        out          = 0;
  float        zero_ffd     = 0;
  const float* zero_ffd_ptr = &zero_ffd;

  size_t i = 0;
  for (auto& pid : pids_) {
    // 由于 pid 会记录 前馈值 传入 nullptr ，在此传入一个指向 0 的值
    float node_out = 0;
    float ref      = ref_ls == nullptr ? 0 : ref_ls[i];
    pid.calc(&ref, &fdb_ls[i], zero_ffd_ptr, &node_out);
    out += node_out;
    i++;
  }

  if (ffd_ls == nullptr) {
    out_ = out_limit_.calc(out);
  } else {
    out_ = out_limit_.calc(out + ffd_ls[0]);
  }

  if (out_ls != nullptr) {
    out_ls[0] = out_;
  } else {
    calc_state_ |= CalcState::kPidCalcStateFailedOut;
  }

  calc_state_ |= CalcState::kPidCalcStateOk;
  return calc_state_ == CalcState::kPidCalcStateOk ? State::kControllerStateOk : State::kControllerStateError;
};
/* Private function definitions ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world
