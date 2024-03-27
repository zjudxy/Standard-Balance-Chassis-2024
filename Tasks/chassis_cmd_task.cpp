/**
 *******************************************************************************
 * @file      : chassis_cmd_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0     yyyy-mm-dd        dxy           <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#include "chassis_cmd_task.hpp"
#include "DT7.hpp"
#include "chassis.hpp"
#include "system.h"
#include "chassis_cmd_params.hpp"
#include "tools.h"
#include "chassis_sense_task.hpp"
#include "chassis_comm_task.hpp"
static void CalMoveScale(void);

static void SwitchCmdSrc(void);

static void JoystickCmd(void);

static void LeverCmd(void);

static void KeyMouseCmd(void);

static float PolyVal(const float *p, float x, uint8_t n);
void RcInit(void);

float move_scale; // 水平移动缩放比
float gyro_speed; // 小陀螺旋转速度
namespace remote_control = hello_world::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
/* External variables --------------------------------------------------------*/

remote_control::DT7 *rc_ptr, *last_rc_ptr, *curr_rc_ptr;
/* Private function prototypes -----------------------------------------------*/

void RcInit(void);

/**
 * @brief       完成指令生成任务的初始化
 * @retval      None
 * @note        None
 */
void CmdInit(void)
{

  kMoveScaleCoff =
      -kMaxLimSpeed / ((kAbsMechanicalLimAng[FRONT][0] - kAbsMechanicalLimAng[FRONT][1]) *
                       (kAbsMechanicalLimAng[FRONT][0] - kAbsMechanicalLimAng[FRONT][1]) / 4);

  // 默认遥控器为控制源
  robot.cmd_src = REMOTE;

  RcInit();
}

/**
 * @brief       根据外部输入生成控制指令
 * @retval      None
 * @note        None
 */
void CmdTask(void)
{
  memcpy(curr_rc_ptr, rc_ptr, sizeof(remote_control::DT7)); // 保持本次指令处理中指令不变
  remote_control::DT7 tmp_rc;
  memcpy(&tmp_rc, rc_ptr, sizeof(remote_control::DT7)); // 当前指令备份

  /* 清除上一次因不满足切换条件而未切换的指令模式 */
  robot.cmd.chassis_mode_ref = robot.chassis_mode.ref;
  robot.cmd.steer_mode_ref = robot.steer_mode.ref;
  // 根据外部输入指令切换指令源
  SwitchCmdSrc();
  // 计算移动缩放比
  CalMoveScale();

  /* 根据指令源处理对应指令 */
  switch (robot.cmd_src)
  {
  case REMOTE:
    JoystickCmd();
    LeverCmd();
    break;
  case KEY_MOUSE:
    KeyMouseCmd();
    break;
  default:
    break;
  }

  if(robot.Init_finish_flag){
    robot.cmd.start_comm = true;
  }

  memcpy(last_rc_ptr, &tmp_rc, sizeof(remote_control::DT7)); // 更新上一次指令
}

void RcInit(void)
{
  rc_ptr = new remote_control::DT7;
  curr_rc_ptr = new remote_control ::DT7;
  last_rc_ptr = new remote_control::DT7;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);
}

/**
 * @brief       根据外部输入指令切换指令源
 * @retval      None
 * @note        None
 */
static void SwitchCmdSrc(void)
{
  switch (robot.cmd_src)
  {
  case REMOTE:
    /* 键鼠有指令时切换为键鼠 */
    if (curr_rc_ptr->key_W() || curr_rc_ptr->mouse_l_btn(false) || curr_rc_ptr->mouse_r_btn(false))
    {
      robot.cmd_src = KEY_MOUSE;
    }
    // if (curr_rc.kb.key_code || curr_rc.mouse.l || curr_rc.mouse.r) {
    //     robot.cmd_src = KEY_MOUSE;
    // }
    break;
  case KEY_MOUSE:
    /* 遥控器拨杆有指令时切换为遥控器 */
    // if (curr_rc_ptr->rc_l_switch() != last_rc_ptr->rc_l_switch() || curr_rc_ptr->rc_r_switch() != last_rc_ptr->rc_l_switch())
    // {
    //   robot.cmd_src = REMOTE;
    // }
    if (fabs(curr_rc_ptr->rc_lv() > 0.1f) || fabs(curr_rc_ptr->rc_lh() > 0.1f) || fabs(curr_rc_ptr->rc_rv() > 0.1f) || fabs(curr_rc_ptr->rc_rh() > 0.1f))
    {
      robot.cmd_src = REMOTE;
    }
    break;
  default:
    break;
  }
}
uint8_t gyro_dir = 0;
#define kGyrospeed 3.0f
float32_t gyro_tick = 0;
float32_t kpoly[6] = {-0.5964, 13.04, -109.3, 416.4, -672.1, 993.9};
int gyro_calibrate_count = 0;
float32_t cmd_angle_last = 0.0f, cmd_angle_curr = 0.0f;
//  p1 =     -0.5964  (-1.256, 0.06311)
//        p2 =       13.04  (1.469, 24.61)
//        p3 =      -109.3  (-185.7, -32.85)
//        p4 =       416.4  (181.8, 651.1)
//        p5 =      -672.1  (-1002, -342.5)
//        p6 =       993.9  (827.8, 1160)
/**
 * @brief       根据遥控器遥杆获取指令
 * @retval      None
 * @note        None
 */
static void JoystickCmd(void)
{

  /* 获取云台指令 */
  robot.cmd.gimbal_x = curr_rc_ptr->rc_lh();
  robot.cmd.gimbal_y = curr_rc_ptr->rc_lv();

  float vx = 0, vy = 0; // 操作手视角期望的x，y方向移动速度

  float sYaw, cYaw;
  arm_sin_cos_f32(R2D(robot.yaw_motor->angle()), &sYaw, &cYaw);
  switch (robot.chassis_mode.curr)
  {
  case CHASSIS_MATURE:
    switch (robot.steer_mode.curr)
    {
    case STEER_MOVE:
      // robot.cmd.dpos = curr_rc_ptr->rc_rv() * move_scale;
      // robot.cmd.yaw_ang -= curr_rc_ptr->rc_lh() * kMaxRotSpeed * kCtrlPeriod;
      ANGLE_ACROSS0_HANDLE_RAD(robot.cmd.yaw_ang, 0);
      vx = curr_rc_ptr->rc_rv() * move_scale;
      if (fabsf(robot.yaw_motor->angle()) < PI / 2)
      {
        robot.cmd.yaw_ang = LimDiff(0, robot.cmd.yaw_ang, kMaxRotSpeed);
      }
      else
      {
        if (robot.cmd.yaw_ang > 0)
        {
          robot.cmd.yaw_ang = LimDiff(PI, robot.cmd.yaw_ang, kMaxRotSpeed);
        }
        else
        {
          robot.cmd.yaw_ang = LimDiff(-PI, robot.cmd.yaw_ang, kMaxRotSpeed);
        }
      }
      // robot.cmd.yaw_ang = LimDiff(0, robot.cmd.yaw_ang, kMaxRotSpeed);
      // robot.cmd.yaw_ang = LimDiff(0+PI*(robot.cmd.gimbal_reverse_seq%2), robot.cmd.yaw_ang, kMaxRotSpeed);
      robot.cmd.gyro_speed = 0;
      robot.cmd.gyro_dir = 0;

      // if (robot.cmd.roll_ang_fix)
      // {
      //   robot.cmd.roll_ang = 0;
      // }
      // else
      // {
      //   if (fabsf(robot.yaw_motor->angle()) < PI / 2)
      //   {
      //     robot.cmd.roll_ang = kRollMaxAng * curr_rc_ptr->rc_rh();
      //   }
      //   else
      //   {
      //     robot.cmd.roll_ang = -kRollMaxAng * curr_rc_ptr->rc_rh();
      //   }
      // }

      robot.cmd.dpos = vx * cYaw + vy * sYaw; // 速度合成
      break;
    case STEER_GYRO:
      /* 根据底盘原始旋转方向选择陀螺方向 */
      // //测试滞后用
      // if (robot.cmd.gyro_dir == 0)
      // {
      //   if (robot.chassis_states.yaw_omg > 0)
      //   {
      //     robot.cmd.gyro_dir = kCcwDir;
      //   }
      //   else
      //   {
      //     robot.cmd.gyro_dir = kCwDir;
      //   }
      // }
      // robot.cmd.dpos = cos((robot.control_tick-gyro_tick)/1000.0f*kGyrospeed)*1.8f;
      // ANGLE_ACROSS0_HANDLE_RAD(robot.cmd.yaw_ang, 0);

      // return ;
      robot.cmd.gyro_dir = kCcwDir;

      float32_t vx, vy;

      vx = curr_rc_ptr->rc_rv();
      vy = -curr_rc_ptr->rc_rh();
      robot.gyro_forward_angle = atan2(vy, vx);
      robot.gyro_forward_speed = sqrt(vx * vx + vy * vy);
      if (robot.steer_mode.ref != STEER_GYRO || fabs(robot.gyro_forward_speed) > 0.1f)
      {
        robot.cmd.gyro_speed = LimDiff(robot.sup_cap->get_remain_present() * kMinGyroSpeed, robot.cmd.gyro_speed, kMaxGyroAcc);
      }
      else
      {
        robot.cmd.gyro_speed = LimDiff(robot.sup_cap->get_remain_present() * kMaxGyroSpeed, robot.cmd.gyro_speed, kMaxGyroAcc);
      }
      // cmd_angle_curr = -robot.gyro_forward_angle - D2R(20);
      // robot.gyro_forward_speed = robot.gyro_forward_speed * move_scale
      // * cos(angle_offset + 0.21f + cmd_angle_curr - robot.gyro_forward_angle + robot.chassis_states.yaw_ang);

      robot.gyro_forward_speed = robot.gyro_forward_speed * move_scale * cos(robot.yaw_motor->angle() + robot.gyro_forward_angle + angle_offset);
      // cos(robot.gyro_forward_angle-angle_offset);
      // 指令角度
      cmd_angle_curr = robot.gyro_forward_angle + robot.imu_datas.euler_vals[YAW] - robot.yaw_motor->angle();
      if (abs(cmd_angle_last - cmd_angle_curr) * kCtrlPeriod < D2R(30.0f) && sqrt(vx * vx + vy * vy) > 0.05f)
      {
        gyro_calibrate_count++;
      }
      else
      {
        gyro_calibrate_count = 0;
      }
      if (gyro_calibrate_count > 200)
      {
        robot.cmd.gyro_cal_flag = true;
      }
      else
      {
        robot.cmd.gyro_cal_flag = false;
      }
      cmd_angle_last = cmd_angle_curr;
      robot.gyro_forward_speed *= robot.sup_cap->get_remain_present();
      robot.cmd.dpos = robot.gyro_forward_speed;
      // robot.cmd.gyro_speed = curr_rc_ptr->rc_lh();
      robot.cmd.gyro_dir = 1.0f;

      // robot.cmd.gyro_speed = 1.0f;
      // robot.cmd.gyro_speed = robot.cmd.gyro_speed * 5.5f;

      robot.cmd.yaw_ang += robot.cmd.gyro_dir * robot.cmd.gyro_speed * kCtrlPeriod;
      ANGLE_ACROSS0_HANDLE_RAD(robot.cmd.yaw_ang, robot.chassis_states.yaw_ang);

      /* 限制期望角度与实际角度的偏差 */
      LIMIT_MAX(
          robot.cmd.yaw_ang, robot.chassis_states.yaw_ang + kYawAngRefMaxBias,
          robot.chassis_states.yaw_ang - kYawAngRefMaxBias);
      ANGLE_RANGE_HANDLE_RAD(robot.cmd.yaw_ang);
      break;
    case STEER_DEPART:
      vx = curr_rc_ptr->rc_rv() * move_scale;

      robot.cmd.dpos = vx;
      robot.cmd.yaw_ang -= curr_rc_ptr->rc_rh() * kMaxRotSpeed * kCtrlPeriod;
      robot.cmd.roll_ang = 0;
      break;

    default:
      break;
    }
    break;
  default:
    break;
  }
}

#define MINIPC_debug
// #define fly_debug
/**
 * @brief       根据遥控器拨杆获取指令
 * @retval      None
 * @note        None
 */
static void LeverCmd(void)
{
  static bool shoot_flag = false;
  static bool jump_flag = false;
  static bool gimbal_reverse_flag = false; // 云台反向标志位
  static bool steer_mode_change = false;
  static bool auto_jump_flag = false;
  static float32_t wheel_val = 0;
  static int gimbal_reset_count = 20;
  robot.cmd.roll_ang_fix = true; // 默认roll轴角度不可改

  

  switch (curr_rc_ptr->rc_l_switch())
  {
  case remote_control::SwitchState::kSwitchStateUp: // 死亡模式
    robot.cmd.chassis_mode_ref = CHASSIS_DEAD;
    robot.cmd.gimbal_enable = false;
    robot.cmd.shooter_enable = false;
    robot.cmd.gimbal_ctrl_mode = GIMBAL_MANUAL;
    robot.cmd.steer_mode_ref = STEER_DEPART;
    gimbal_reset_count = 20;
    robot.cmd.height = kHeightMin;
    break;
  case remote_control::SwitchState::kSwitchStateMid: // 运动模式
    robot.cmd.chassis_mode_ref = CHASSIS_MATURE;
    robot.cmd.steer_mode_ref = STEER_MOVE;
    robot.cmd.gimbal_enable = true;
    robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
    robot.cmd.shooter_enable = true;
    robot.cmd.height = kHeightMin;
    robot.cmd.speed_limit = true;
    switch (curr_rc_ptr->rc_r_switch())
    {
    case remote_control::SwitchState::kSwitchStateDown:
      robot.cmd.height = kHeightMin;
      break;
    case remote_control::SwitchState::kSwitchStateMid:
      robot.cmd.height = kHeightMid;
      break;
    case remote_control::SwitchState::kSwitchStateUp:
      robot.cmd.height = 0.28f;
    default:
      break;
    }
#ifdef MINIPC_debug
    if (gimbal_reset_count > 0)
    {
      gimbal_reset_count--;
      robot.cmd.gimbal_enable = false;
      robot.cmd.gimbal_ctrl_mode = GIMBAL_MANUAL;
    }
    switch (curr_rc_ptr->rc_r_switch()) // 自瞄测试
    {
    case remote_control::SwitchState::kSwitchStateDown:
      robot.cmd.auto_shoot = false;
      robot.cmd.auto_aim_mode = AUTO_AIM_CLOSE;
      break;
    case remote_control::SwitchState::kSwitchStateMid:
      robot.cmd.auto_shoot = false;
      robot.cmd.auto_aim_mode = PREDICT;

      break;
    case remote_control::SwitchState::kSwitchStateUp:
      robot.cmd.auto_shoot = true;
      robot.cmd.auto_aim_mode = ANTI_TWIST;
      // robot.comm.chassis2gimbal.auto_aim_mode = ANTI_TWIST;
      break;
    default:
      break;
    }
#endif
    break;
  case remote_control::SwitchState::kSwitchStateDown: // 躺尸自瞄模式
    gyro_tick = robot.control_tick;
    robot.cmd.height = kHeightMid;
    robot.cmd.steer_mode_ref = STEER_GYRO;
#ifdef MINIPC_debug
    robot.cmd.chassis_mode_ref = CHASSIS_DEAD;
    robot.cmd.steer_mode_ref = STEER_DEPART;
    switch (curr_rc_ptr->rc_r_switch()) // 自瞄测试
    {
    case remote_control::SwitchState::kSwitchStateDown:
      robot.cmd.auto_shoot = false;
      robot.cmd.auto_aim_mode = AUTO_AIM_CLOSE;
      break;
    case remote_control::SwitchState::kSwitchStateMid:
      robot.cmd.auto_shoot = false;
      robot.cmd.auto_aim_mode = PREDICT;

      break;
    case remote_control::SwitchState::kSwitchStateUp:
      robot.cmd.auto_shoot = true;
      robot.cmd.auto_aim_mode = ANTI_TWIST;
      // robot.comm.chassis2gimbal.auto_aim_mode = ANTI_TWIST;
      break;
    default:
      break;
    }

#endif
    robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
    robot.cmd.shooter_enable = true;
    robot.cmd.gimbal_enable = true;
    gimbal_reset_count = 20;
#ifdef fly_debug
    robot.cmd.steer_mode_ref = STEER_MOVE;
    // robot.cmd.speed_limit = false;
    if (last_rc_ptr->rc_l_switch() != curr_rc_ptr->rc_l_switch() && (robot.comm.chassis2gimbal.reverse_seq == robot.comm.gimbal2chassis.reverse_finish_seq))
    {
      robot.cmd.gimbal_reverse_seq++;
    }
#endif

    break;
  default:
    break;
  }
  // 发弹控制
  static int shoot_count = 0;
  robot.cmd.shoot = false;
  if (robot.cmd.shooter_enable)
  {
    if (curr_rc_ptr->rc_wheel() > 8.0f)
    {
      shoot_count++;
      switch (curr_rc_ptr->rc_r_switch())
      {
      case remote_control::SwitchState::kSwitchStateMid:
        if (shoot_count == 10)
        {
          robot.cmd.shoot = true;
        }
        break;
      case remote_control::SwitchState::kSwitchStateUp:
        if (shoot_count % 100 == 0)
        {
          robot.cmd.shoot = true;
        }
      default:
        break;
      }
    }
    else
    {
      shoot_count = 0;
    }
  }
  //   switch (curr_rc_ptr->rc_r_switch())
  // {
  // case remote_control::SwitchState::kSwitchStateDown:
  //   robot.cmd.height = kHeightMin;
  //   break;
  // case remote_control::SwitchState::kSwitchStateMid:
  //   robot.cmd.height = kHeightMid;
  //   break;
  // case remote_control::SwitchState::kSwitchStateUp:
  //   robot.cmd.height = 0.28f;
  // default:
  //   break;
  // }
  // if(curr_rc_ptr->rc_r_switch()!=last_rc_ptr->rc_r_switch()){
  //   switch (curr_rc_ptr->rc_r_switch())
  // {
  // case remote_control::SwitchState::kSwitchStateDown:
  //   robot.gyro_forward_speed-=0.01f;
  //   break;
  // case remote_control::SwitchState::kSwitchStateMid:

  //   break;
  // case remote_control::SwitchState::kSwitchStateUp:
  //   robot.gyro_forward_speed+=0.01f;
  // default:
  //   break;
  // }}
}
/**
 * @brief       根据键鼠获取指令
 * @retval      None
 * @note        None
 */
static void KeyMouseCmd(void)
{
  static uint16_t auto_mode_close_count = 0; // 切出自瞄计时
  static bool auto_jump_flag = false;

  robot.cmd.speed_limit = true;
  robot.cmd.auto_shoot = false;

  /* CTRL + SHIFT + _ */
  if (curr_rc_ptr->key_CTRL() && curr_rc_ptr->key_SHIFT(false))
  {

    if (curr_rc_ptr->key_B(true))
    { // CTRL + SHIFT + B: 进入RECOVERY中的微调
      curr_rc_ptr->key_CTRL();
      curr_rc_ptr->key_SHIFT();
      robot.cmd.recovery_tune = true;
      robot.cmd.chassis_mode_ref = CHASSIS_MATURE;
      robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
      robot.cmd.steer_mode_ref = STEER_MOVE;
      robot.cmd.gimbal_enable = true;
      robot.cmd.shooter_enable = true;
      robot.cmd.height = kHeightMin; // 最低腿长
    }
    if (curr_rc_ptr->key_Z())
    {
      robot.comm.judge2gimbal.infantry_types.type_3 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_4 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_5 = NORMAL_INFANTRY;
    }
    else if (curr_rc_ptr->key_X())
    {
      robot.comm.judge2gimbal.infantry_types.type_3 = BALANCE_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_4 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_5 = NORMAL_INFANTRY;
    }
    else if (curr_rc_ptr->key_C())
    {
      robot.comm.judge2gimbal.infantry_types.type_3 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_4 = BALANCE_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_5 = NORMAL_INFANTRY;
    }
    else if(curr_rc_ptr->key_V()){
      robot.comm.judge2gimbal.infantry_types.type_3 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_4 = NORMAL_INFANTRY;
      robot.comm.judge2gimbal.infantry_types.type_5 = BALANCE_INFANTRY;
    }
  }
  // else if (curr_rc_ptr->key_B(true)) {// CTRL + SHIFT + B: minipc关机
  //     robot.cmd.auto_aim_mode = MINIPC_SHUTDOWN;
  // } else if (curr_rc_ptr->key_R(true)) {// CTRL + SHIFT + R: minipc重启
  //     robot.cmd.auto_aim_mode = MINIPC_REBOOT;
  // }

  // if (curr_rc_ptr->key_W(true)) {
  //     if (!auto_jump_flag) {
  //         auto_jump_flag = true;
  //         robot.cmd.auto_jump = true;
  //     }
  // } else {
  //     auto_jump_flag = false;
  // }


  /* CTRL + _ */
  if (curr_rc_ptr->key_CTRL() && !curr_rc_ptr->key_SHIFT())
  {
    if (curr_rc_ptr->key_V(true))
    {
      robot.cmd.chassis_mode_ref = CHASSIS_DEAD; // CTRL + V: 进入DEAD模式
      robot.cmd.steer_mode_ref = STEER_DEPART;
      robot.cmd.gimbal_enable = false;
      robot.cmd.shooter_enable = false;
    }
    else if (curr_rc_ptr->key_B(true))
    {

      robot.cmd.chassis_mode_ref = CHASSIS_MATURE; // CTRL + B: 进入MATURE模式
      robot.cmd.recovery_tune = false;
      robot.cmd.steer_mode_ref = STEER_MOVE;
      robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
      robot.cmd.gimbal_enable = true;
      robot.cmd.height = kHeightMin; // 最低腿长
      robot.cmd.shooter_enable = true;
    }
    else if (curr_rc_ptr->key_G(true))
    { // CTRL + G: 重画UI
      robot.cmd.ui_redraw = true;
    }

    if (robot.chassis_mode.curr != CHASSIS_JUMP)
    {
      if (curr_rc_ptr->key_Z(true))
      {
        robot.cmd.height = kHeightMin; // CTRL + Z: 最低腿长
      }
      else if (curr_rc_ptr->key_X(true))
      {
        robot.cmd.height = kHeightMid; // CTRL + X: 中等腿长
      }
      else if (curr_rc_ptr->key_C(true))
      {
        robot.cmd.height = kHeightMax; // CTRL + C: 最高腿长
      }
      else if (curr_rc_ptr->key_W(true))
      {
        robot.cmd.height = LimDiff(kHeightMax, robot.cmd.height, kHeightAdjDiff); // CTRL + W: 调高腿长
      }
      else if (curr_rc_ptr->key_S(true))
      {
        robot.cmd.height = LimDiff(kHeightMin, robot.cmd.height, kHeightAdjDiff); // CTRL + S: 调低腿长
      }
    }
    /* 限制小陀螺时的腿长 */
    if (robot.steer_mode.curr == STEER_GYRO)
    {
      LIMIT_MAX(robot.cmd.height, kGyroHeightMin, kGyroHeightMax);
    }
  }

  // /* SHIFT + _ */
  if (curr_rc_ptr->key_SHIFT() && !curr_rc_ptr->key_CTRL())
  { // SHIFT: 解除速度限制同时，腿长恢复为中等高度
    robot.cmd.speed_limit = false;
    if (curr_rc_ptr->key_W() || curr_rc_ptr->key_S())
    {
      robot.cmd.height = kHeightMid;
      // robot.cmd.speed_limit = false;
    }

    // if (curr_rc_ptr->key_X() || curr_rc_ptr->key_C()) {
    //     robot.cmd.auto_shoot = true;
    // }

    if (curr_rc_ptr->key_A())
    {
      if (robot.steer_mode.curr == STEER_MOVE)
      {
        if (fabsf(robot.yaw_motor->angle() < PI / 2))
        {
          robot.cmd.roll_ang = LimDiff(-kRollMaxAng, robot.cmd.roll_ang, kRollAngDiff);
        }
        else
        {
          robot.cmd.roll_ang = LimDiff(kRollMaxAng, robot.cmd.roll_ang, kRollAngDiff);
        }
      }
    }
    else if (curr_rc_ptr->key_D())
    {
      if (robot.steer_mode.curr == STEER_MOVE)
      {
        if (fabsf(robot.yaw_motor->angle()) < PI / 2)
        {
          robot.cmd.roll_ang = LimDiff(kRollMaxAng, robot.cmd.roll_ang, kRollAngDiff);
        }
        else
        {
          robot.cmd.roll_ang = LimDiff(-kRollMaxAng, robot.cmd.roll_ang, kRollAngDiff);
        }
      }
    }
    else
    {
      robot.cmd.roll_ang = LimDiff(0, robot.cmd.roll_ang, kRollAngDiff);
    }
  }
  else
  {
    robot.cmd.roll_ang = LimDiff(0, robot.cmd.roll_ang, kRollAngDiff);
    robot.cmd.speed_limit = true;
  }

  // 单键控制部分
  if (curr_rc_ptr->key_R())
  {
    // robot.cmd.gimbal_reverse_seq = robot.comm.gimbal2chassis.reverse_finish_seq;
    robot.cmd.steer_mode_ref = STEER_MOVE; // R: 进入MOVE模式
    robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;

  }
  else if (curr_rc_ptr->key_E())
  {
    // robot.cmd.gimbal_reverse_seq = robot.comm.gimbal2chassis.reverse_finish_seq;
    robot.cmd.steer_mode_ref = STEER_GYRO; // E: 进入GYRO模式
    robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
    robot.cmd.shooter_enable = true;
  }
  else if (curr_rc_ptr->key_Q())
  {
    // robot.cmd.gimbal_reverse_seq = robot.comm.gimbal2chassis.reverse_finish_seq;
    robot.cmd.steer_mode_ref = STEER_MOVE; // Q: 退出小陀螺
    robot.cmd.gimbal_enable = true;
    robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
  }
  if (curr_rc_ptr->key_F())
  {
    if (robot.comm.chassis2gimbal.reverse_seq == robot.comm.gimbal2chassis.reverse_finish_seq)
    {
      robot.cmd.gimbal_reverse_seq++;
    }
  }

  if (curr_rc_ptr->key_G())
  {
    robot.cmd.ignore_overheated = true; // G: 无视枪口热量
  }
  else
  {
    robot.cmd.ignore_overheated = false;
  }

  /* 根据底盘模式生成指令 */
  float vx = 0, vy = 0;
  static float gyro_speed_forw = 0;
  switch (robot.chassis_mode.curr)
  {
  case CHASSIS_MATURE:
    robot.cmd.recovery_tune = false;

    float sYaw, cYaw;

    arm_sin_cos_f32(R2D(robot.yaw_motor->angle()), &sYaw, &cYaw);

    /* 根据转向模式生成指令 */
    switch (robot.steer_mode.curr)
    {
    case STEER_MOVE:
      if (fabsf(robot.yaw_motor->angle()) < PI / 2)
      {
        robot.cmd.yaw_ang = LimDiff(0, robot.cmd.yaw_ang, kMaxRotSpeed);
      }
      else
      {
        if (robot.cmd.yaw_ang > 0)
        {
          robot.cmd.yaw_ang = LimDiff(PI, robot.cmd.yaw_ang, kMaxRotSpeed);
        }
        else
        {
          robot.cmd.yaw_ang = LimDiff(-PI, robot.cmd.yaw_ang, kMaxRotSpeed);
        }
      }
      // robot.cmd.yaw_ang = LimDiff(0+PI*(robot.cmd.gimbal_reverse_seq%2), robot.cmd.yaw_ang, kMaxRotSpeed);
      // robot.cmd.yaw_ang = LimDiff(0, robot.cmd.yaw_ang, kMaxRotSpeed);
      robot.cmd.gyro_speed = 0;
      robot.cmd.gyro_dir = 0;

      if (curr_rc_ptr->key_W())
      {
        vx = move_scale;
      }
      else if (curr_rc_ptr->key_S())
      {
        vx = -move_scale;
      }
      else if ((curr_rc_ptr->key_A() || curr_rc_ptr->key_D())&&!curr_rc_ptr->key_SHIFT()&&!curr_rc_ptr->key_SHIFT())
      {
        robot.cmd.steer_mode_ref = STEER_DEFENSE;
      }
      // else if (curr_rc_ptr->key_A() || curr_rc_ptr->key_D())
      // {
      //   robot.cmd.steer_mode_ref = STEER_DEFENSE;
      // }
      robot.cmd.dpos = vx * cYaw + vy * sYaw; // 速度合成
      break;
    case STEER_DEFENSE:
      robot.cmd.gyro_speed = 0;
      robot.cmd.gyro_dir = 0;
      // robot.cmd.yaw_ang = 0.0f;
      if (robot.yaw_motor->angle() > 0)
      {
        robot.cmd.yaw_ang = LimDiff(PI / 2, robot.cmd.yaw_ang, kMaxRotSpeed);
      }
      else
      {
        robot.cmd.yaw_ang = LimDiff(-PI / 2, robot.cmd.yaw_ang, kMaxRotSpeed);
      }

      if (curr_rc_ptr->key_W() || curr_rc_ptr->key_S())
      {
        robot.cmd.steer_mode_ref = STEER_MOVE;
      }

      if (curr_rc_ptr->key_A()&&!curr_rc_ptr->key_SHIFT()&&!curr_rc_ptr->key_SHIFT())
      {
        vy = move_scale;
      }
      else if (curr_rc_ptr->key_D()&&!curr_rc_ptr->key_SHIFT()&&!curr_rc_ptr->key_SHIFT())
      {
        vy = -move_scale;
      }
      robot.cmd.dpos = vx * cYaw + vy * sYaw; // 速度合成
      break;
    case STEER_GYRO:
      /* 根据底盘原始旋转方向选择陀螺方向 */
      // //测试滞后用
      // if (robot.cmd.gyro_dir == 0)
      // {
      //   if (robot.chassis_states.yaw_omg > 0)
      //   {
      //     robot.cmd.gyro_dir = kCcwDir;
      //   }
      //   else
      //   {
      //     robot.cmd.gyro_dir = kCwDir;
      //   }
      // }
      // robot.cmd.dpos = cos((robot.control_tick-gyro_tick)/1000.0f*kGyrospeed)*1.8f;
      // ANGLE_ACROSS0_HANDLE_RAD(robot.cmd.yaw_ang, 0);

      // return ;
      robot.cmd.gyro_dir = kCcwDir;

      float32_t vx, vy;
      vx = (curr_rc_ptr->key_W() - curr_rc_ptr->key_S()) * 1.0f;
      vy = (curr_rc_ptr->key_A() - curr_rc_ptr->key_D()) * 1.0f;
      robot.gyro_forward_angle = atan2(vy, vx);
      gyro_speed_forw = LimDiff((vx * vx + vy * vy) / 2.5, gyro_speed_forw, kMaxGyroForwardAcc);
      if (robot.steer_mode.ref != STEER_GYRO || fabs(gyro_speed_forw) > 0.1f)
      {
        robot.cmd.gyro_speed = LimDiff(robot.sup_cap->get_remain_present() * kMinGyroSpeed, robot.cmd.gyro_speed, kMaxGyroAcc);
      }
      else
      {
        robot.cmd.gyro_speed = LimDiff(robot.sup_cap->get_remain_present() * kMaxGyroSpeed, robot.cmd.gyro_speed, kMaxGyroAcc);
      }
      // cmd_angle_curr = -robot.gyro_forward_angle - D2R(20);
      // robot.gyro_forward_speed = robot.gyro_forward_speed * move_scale
      // * cos(angle_offset + 0.21f + cmd_angle_curr - robot.gyro_forward_angle + robot.chassis_states.yaw_ang);

      robot.gyro_forward_speed = gyro_speed_forw * move_scale * cos(robot.yaw_motor->angle() + robot.gyro_forward_angle + angle_offset + D2R(120.0f));
      // cos(robot.gyro_forward_angle-angle_offset);
      // 指令角度
      cmd_angle_curr = robot.gyro_forward_angle + robot.imu_datas.euler_vals[YAW] - robot.yaw_motor->angle();
      if (abs(cmd_angle_last - cmd_angle_curr) * kCtrlPeriod < D2R(30.0f) && sqrt(vx * vx + vy * vy) > 0.05f)
      {
        gyro_calibrate_count++;
      }
      else
      {
        gyro_calibrate_count = 0;
      }
      if (gyro_calibrate_count > 200)
      {
        robot.cmd.gyro_cal_flag = true;
      }
      else
      {
        robot.cmd.gyro_cal_flag = false;
      }
      cmd_angle_last = cmd_angle_curr;
      robot.gyro_forward_speed *= robot.sup_cap->get_remain_present();
      robot.cmd.dpos = robot.gyro_forward_speed;
      // robot.cmd.gyro_speed = curr_rc_ptr->rc_lh();
      robot.cmd.gyro_dir = 1.0f;
      // robot.cmd.gyro_speed = 1.0f;
      // robot.cmd.gyro_speed = robot.cmd.gyro_speed * 5.5f;

      robot.cmd.yaw_ang += robot.cmd.gyro_dir * robot.cmd.gyro_speed * kCtrlPeriod;
      ANGLE_ACROSS0_HANDLE_RAD(robot.cmd.yaw_ang, robot.chassis_states.yaw_ang);

      /* 限制期望角度与实际角度的偏差 */
      LIMIT_MAX(
          robot.cmd.yaw_ang, robot.chassis_states.yaw_ang + kYawAngRefMaxBias,
          robot.chassis_states.yaw_ang - kYawAngRefMaxBias);
      ANGLE_RANGE_HANDLE_RAD(robot.cmd.yaw_ang);
      break;

    case STEER_DEPART:
      robot.cmd.gyro_speed = 0;
      robot.cmd.gyro_dir = 0;

      if (curr_rc_ptr->key_W() || curr_rc_ptr->key_S())
      {
        robot.cmd.steer_mode_ref = STEER_MOVE;
      }
      // else if (curr_rc_ptr->key_A() || curr_rc_ptr->key_D())
      // {
      //   robot.cmd.steer_mode_ref = STEER_DEFENSE;
      // }
      break;
    }
    break;
  case CHASSIS_RECOVERY:
    if (robot.mode_state.recovery == RECOVERY_STATE_TUNE)
    {
      if (curr_rc_ptr->key_W())
      {
        robot.cmd.dpos = kTuneMoveScale;
      }
      else if (curr_rc_ptr->key_S())
      {
        robot.cmd.dpos = -kTuneMoveScale;
      }
      else
      {
        robot.cmd.dpos = 0;
      }

      if (curr_rc_ptr->key_A())
      {
        robot.cmd.yaw_ang += kTuneTurnScale;
      }
      else if (curr_rc_ptr->key_D())
      {
        robot.cmd.yaw_ang -= kTuneTurnScale;
      }
    }
    break;
  case CHASSIS_DEAD:
    robot.cmd.dpos = 0;
    robot.cmd.height = kHeightMin;
    robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
    robot.cmd.recovery_tune = false;
    break;
  case CHASSIS_JUMP:
    robot.cmd.auto_jump = false;
    break;
  default:
    break;
  }
  /* 云台指令生成 */

  robot.cmd.gimbal_x = curr_rc_ptr->mouse_x() / kMouseMoveLim;
  // if (robot.steer_mode.curr != STEER_GYRO)
  // {
  //   robot.cmd.gimbal_x += (-curr_rc_ptr->key_A() * (curr_rc_ptr->key_SHIFT() ? 0 : 1.0f) + curr_rc_ptr->key_D() * (curr_rc_ptr->key_SHIFT() ? 0 : 1.0f));
  // }

  robot.cmd.gimbal_y = -curr_rc_ptr->mouse_y() / kMouseMoveLim;
  LIMIT_MAX(robot.cmd.gimbal_x, 1, -1);
  LIMIT_MAX(robot.cmd.gimbal_y, 1, -1);
  robot.cmd.auto_aim_mode = AUTO_AIM_CLOSE;
  if (robot.comm.chassis2gimbal.gimbal_ctrl_mode == GIMBAL_AUTO)
  {
    if (curr_rc_ptr->mouse_r_btn(false))
    {
      robot.cmd.auto_aim_mode = ANTI_TWIST;
      robot.cmd.auto_shoot = false;
      if (curr_rc_ptr->key_Z())
      {
        robot.cmd.auto_shoot = true;
      }
    }
  }
  robot.cmd.shoot = false;

  if (curr_rc_ptr->mouse_l_btn(false) && robot.control_tick % 80 == 0)
  {
    if(robot.referee_ptr->PERFORMANCE->getData().power_management_shooter_output){
      robot.cmd.shooter_enable = true;
    }
    robot.cmd.shoot = true;
  }
  // if (robot.comm.chassis2gimbal.gimbal_ctrl_mode == GIMBAL_AUTO) {
  //     if ((curr_rc_ptr->key_Z() && robot.comm.chassis2gimbal.auto_aim_mode == ANTI_TWIST) ||
  //         (curr_rc_ptr->key_X() && robot.comm.chassis2gimbal.auto_aim_mode == SMALL_BUFF) ||
  //         (curr_rc_ptr->key_C() && robot.comm.chassis2gimbal.auto_aim_mode == BIG_BUFF) ||
  //         (curr_rc_ptr->mouse_r_btn(false) && robot.comm.chassis2gimbal.auto_aim_mode == PREDICT)) {

  //     } else {  // 退出自瞄
  //         robot.cmd.gimbal_ctrl_mode = GIMBAL_MANUAL;
  //         robot.cmd.auto_aim_mode = AUTO_AIM_CLOSE;
  //     }
  // } else if (robot.comm.chassis2gimbal.gimbal_ctrl_mode == GIMBAL_MANUAL) {
  //     if (auto_mode_close_count * kCtrlPeriod < kAutoModeDeadZoneTimeThres) {
  //         auto_mode_close_count++;
  //     } else {                     // 防止不同自瞄模式切换过快
  //         if (curr_rc_ptr->key_Z()) {  // Z: 反陀螺
  //             robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
  //             robot.cmd.auto_aim_mode = ANTI_TWIST;
  //             robot.cmd.auto_shoot = true;
  //             auto_mode_close_count = 0;
  //         } else if (curr_rc_ptr->key_X()) {  // X: 小符
  //             robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
  //             robot.cmd.auto_aim_mode = SMALL_BUFF;
  //             auto_mode_close_count = 0;
  //         } else if (curr_rc_ptr->key_C()) {  // C: 大符
  //             robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
  //             robot.cmd.auto_aim_mode = BIG_BUFF;
  //             auto_mode_close_count = 0;
  //         } else if (curr_rc_ptr->mouse_r_btn(false)) {  // 右键: 预测
  //             robot.cmd.gimbal_ctrl_mode = GIMBAL_AUTO;
  //             robot.cmd.auto_aim_mode = PREDICT;
  //             auto_mode_close_count = 0;
  //         }
  //     }
  // }
}

/**
 * @brief       计算移动缩放比
 * @retval      None
 * @note        None
 */
static void CalMoveScale(void)
{
  move_scale = kMaxMovingSpeed;
  if (robot.cmd.speed_limit)
  {
    /* 进行速度限制 */
    float joint_angs[2];
    static float tmp;

    for (uint8_t i = 0; i < 2; i++)
    {
      /* 计算当前腿长在稳定状态时关节角度 */
      robot.five_rods_cal.invKinematics(
          &robot.five_rods_cal, joint_angs + FRONT, joint_angs + BEHIND, robot.leg_states[i].curr.state.height, PI / 2);

      /* 二次函数插值计算移动缩放比 */
      tmp = kMoveScaleCoff * (joint_angs[FRONT] - kAbsMechanicalLimAng[FRONT][0]) * (joint_angs[FRONT] - kAbsMechanicalLimAng[FRONT][1]);

      move_scale = MIN(move_scale, tmp);
    }

    LIMIT_MAX(move_scale, kMaxMovingSpeed, 0); // 限制移动缩放比在合理范围
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart5)
  {
    if (Size == remote_control::kRcRxDataLen)
    {
      HAL_IWDG_Refresh(&hiwdg1);

      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);
    return;
  }
  if (huart == &huart1)
  {

    robot.referee_ptr->decode(referee_rx_data, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_data, robot.referee_ptr->get_rx_dma_len());
    return;
  }
}

/**
 * @brief       compute polynomial
 * @param       *p: ptr to array of polynomial coefficient
 * @param       x: independent variable
 * @param       n: highest power of polynomial
 * @retval      None
 * @note        polynomial power from n to 0
 */
static float PolyVal(const float *p, float x, uint8_t n)
{
  float result = 0;
  for (uint8_t i = 0; i <= n; i++)
  {
    result = result * x + p[i];
  }

  return result;
}