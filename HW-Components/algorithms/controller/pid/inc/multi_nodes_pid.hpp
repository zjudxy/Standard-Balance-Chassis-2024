/** 
 * @file      multi_nodes_pid.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONENTS_ALGORITHMS_CONTROLLER_MULTI_NODES_PID_HPP_
#define HWCOMPONENTS_ALGORITHMS_CONTROLLER_MULTI_NODES_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <list>

#include "allocator.hpp"
#include "assert.hpp"
#include "basic_pid.hpp"
#include "controller_base.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace pid
{
/* Exported constants --------------------------------------------------------*/
enum MultiNodesPidType {
  kMultiNodesPidTypeCascade = 0,  ///< 串行 PID
  kMultiNodesPidTypeParallel,     ///< 并行 PID
};

/* Exported types ------------------------------------------------------------*/

class MultiNodesPid : public Controller
{
 public:
  typedef MultiNodesPidType Type;
  typedef ControllerState   State;
  typedef BasicPid          Pid;
  typedef BasicPidParams    Params;
  typedef BasicPidDatas     Datas;
  typedef BasicPidCalcState CalcState;

  typedef std::list<Params, Allocator<Params>> ParamsList;
  typedef std::list<Pid, Allocator<Pid>>       PidList;

  /**
   * @brief 构造函数，初始化 PID 控制器的类型和输出限制
   * 
   * @param type PID 控制器的类型
   * @param out_limit PID 控制器的输出限制
   */
  explicit MultiNodesPid(Type type, OutLimit out_limit);

  /**
   * @brief 构造函数，初始化 PID 控制器的类型、输出限制和大小
   * 
   * @param type PID 控制器的类型
   * @param out_limit PID 控制器的输出限制
   * @param size PID 控制器的大小
   */
  explicit MultiNodesPid(Type type, OutLimit out_limit, size_t size);

  /**
   * @brief 构造函数，初始化 PID 控制器的类型、输出限制和 PID 列表
   * 
   * @param type PID 控制器的类型
   * @param out_limit PID 控制器的输出限制
   * @param pids PID 控制器的列表
   */
  explicit MultiNodesPid(Type type, OutLimit out_limit, const PidList& pids);

  /**
   * @brief 构造函数，初始化 PID 控制器的类型、输出限制和参数列表
   * 
   * @param type PID 控制器的类型
   * @param out_limit PID 控制器的输出限制
   * @param params_list PID 控制器的参数列表
   */
  explicit MultiNodesPid(Type type, OutLimit out_limit, const ParamsList& params_list);

  /**
   * @brief 向PID控制器集合尾部添加一个已经存在的PID对象。
   * @param pid 一个PID对象的引用。
   */
  void    push_back(const Pid& pid);
  /**
   * @brief 根据给定的参数创建一个PID控制器，并将其添加到集合尾部。
   * @param params PID参数结构体。
   */
  void    push_back(const Params& params);
  /**
   * @brief 从集合中移除位于特定索引位置的PID控制器。
   * @param idx 要移除PID控制器的索引。从 0 开始计数。
   * @attention 如果 idx 超出范围，将会触发断言，将导致程序异常终止。
   */
  void    remove(size_t idx);
  /**
   * @brief 返回集合中PID控制器的数量。
   * @return size_t PID控制器的数量。
   */
  size_t  size() const { return pids_.size(); };
  /** 
   * @brief 获得顺序容器中第 idx 个元素的引用
   * @param idx 顺序容器中的索引。从 0 开始计数。
   * @return Pid& 元素的引用
   * @attention 如果 idx 超出范围，将会触发断言，将导致程序异常终止。
   */
  Pid&    getPidAt(size_t idx);
  /** 
   * @brief 获得顺序容器中第 idx 个 PID 对象中的参数对象的引用。
   * @param idx 顺序容器中的索引，从 0 开始计数。
   * @return Params& 参数对象的引用。
   * @attention 如果 idx 超出范围，将会触发断言，将导致程序异常终止。
   * @note 通过引用实现数值传递，可通过该函数修改参数
   */
  Params& getParamsAt(size_t idx) { return getPidAt(idx).params(); };
  /** 
   * @brief 获得顺序容器中第 idx 个 PID 对象中的数据副本。
   * @param idx 顺序容器中的索引，从 0 开始计数。
   * @return Datas PID对象包含的数据的副本。
   * @attention 如果 idx 超出范围，将会触发断言，将导致程序异常终止。
   */
  Datas   getDatasAt(size_t idx) { return getPidAt(idx).datas(); };

  /**
   * @brief 重置所有PID控制器状态。
   * @return 返回控制器重置后的状态。
   */
  State reset() override;
  /** 返回上一次计算的结果 */
  float out() const { return out_; };
  /** 
   * @brief 根据当前的PID类型执行对应的计算逻辑。
   * 
   * @param ref_ls 参考值列表的指针。
   * @param fdb_ls 反馈值列表的指针。
   * @param ffd_ls 前馈值列表的指针。
   * @param out_ls 输出值列表的指针用于存放计算结果。
   * @return State 返回控制器的状态。
   * @attention 如果类型不是串行或并行PID，将会触发断言，将导致程序异常终止。
   * @see calcCascade
   * @see calcParallel
   */
  State calc(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls) override;
  /** 
   * @brief 执行级联PID控制器计算。
   * 
   * 级联计算，对每个节点的输出作为下一个节点的输入。
   * 
   * @param ref_ls 参考值列表的指针。
   * @param fdb_ls 反馈值列表的指针。
   * @param ffd_ls 前馈值列表的指针。
   * @param out_ls 输出值列表的指针用于存放计算结果。
   * @return State 返回控制器的状态。
   * @attention 如果控制器类型不符，将触发断言。
   * @attention 如果 fdb_ls==nullptr 将会不进行计算，直接返回 State::kControllerStateError。
   * @attention 如果 ref_ls == nullptr ，将 0 作为初始节点的参考值。
   */
  State calcCascade(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls);
  /** 
   * @brief 执行并行PID控制器计算。
   * 
   * 并行计算，对每个节点进行独立计算，然后累加结果。
   * 
   * @param ref_ls 参考值列表的指针。
   * @param fdb_ls 反馈值列表的指针。
   * @param ffd_ls 前馈值列表的指针。
   * @param out_ls 输出值列表的指针用于存放计算结果。
   * @return State 返回控制器的状态。
   * @attention 如果传入的指针为空或控制器类型不符，将触发断言。
   * @attention 如果 fdb_ls==nullptr 将会不进行计算，直接返回 State::kControllerStateError。
   * @attention 如果 ref_ls == nullptr ，将 0 作为所有节点的参考值。
   */
  State calcParallel(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls);

 private:
  PidList  pids_       = PidList();                        ///< PID控制器列表
  Type     type_       = kMultiNodesPidTypeCascade;        ///< PID类型
  OutLimit out_limit_  = OutLimit(true, 0, 0);             ///< 输出限幅优化器
  float    out_        = 0.0f;                             ///< 输出值
  uint32_t calc_state_ = CalcState::kPidCalcStateLackRef;  ///< 计算状态

  void setControllerDims(size_t size);
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_CONTROLLER_MULTI_NODES_PID_HPP_ */
