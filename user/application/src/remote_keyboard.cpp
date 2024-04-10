/**
 *******************************************************************************
 * @file      : remote_keyboard.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "remote_keyboard.h"

#include "gimbal.h"
#include "motor_pidmodify.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t trigger_flag;                                                        // 拨弹盘开关控制
uint8_t fric_flag;                                                           // 摩擦轮状态标志
uint8_t shoot_mode;                                                          // 射击模式标志
PidSwitchMode pitchpid_switchflag = base_pid, yawpid_switchflag = base_pid;  // 针对不同控制模式，在同一电机执行不同任务时，设置不同的PID参数
/* External variables --------------------------------------------------------*/
extern uint8_t remote_key_press[16];
extern uint8_t referee_key_press[16];
/* Private function prototypes -----------------------------------------------*/
void PidFlagInit();
void PidModeSwitch();
void RemoteDisconShootCtrl();
void PidAdjust();
void PidAdjustByError();

void ModeTask()
{
    /*遥控器控制模式选择*/
    // 右拨杆在上，遥控器控制模式
    if (remote.GetS2() == 1) {
        /*单发模式控制,与下面的代码只能存在一个*/
        // fric_flag=1;
        // shoot_mode = 0;
        // RemoteDisconShootCtrl();
        // PidAdjust();
        // RemoteAimingTargetSet();
        // MotorStart();


        /*左拨杆在上——只转摩擦轮模式，中——拨弹盘低速模式，下——高速模式或（遥控器切换到自瞄模式，需要使用时取消注释）*/
        // if (remote.GetS1() == 1 || remote.GetS1() == 3) {
        fric_flag = 1;
        shoot_mode = 1;
        PidAdjust();
        RemoteAimingTargetSet();
        MotorStart();
        // } else {
        //     fric_flag = 1;
        //     shoot_mode = 1;
        //     PidAdjust();
        //     AutoAimingTargetSet();
        // MotorStart();
        // }
    }

    // 右拨杆在中，键鼠模式
    if (remote.GetS2() == 3) {
        // 当启用空中支援且有剩余发弹时间(默认向下取整0.9==0）时才能用键鼠模式控制，否则为全停模式

        referee.aerial_robot_support_data_.airforce_status = 2;//测试用
        referee.aerial_robot_support_data_.time_remain = 1;//测试用
        if (referee.aerial_robot_support_data_.airforce_status == 2 && referee.aerial_robot_support_data_.time_remain > 0) {
            fric_flag = 1;
            shoot_mode = 1;
            // 当键盘R键按下期间，键鼠模式切换到自瞄模式，松开换回键鼠手控模式
            if (remote_key_press[KEY_R] || referee_key_press[KEY_R] == 1) {
                PidAdjust();
                AutoAimingTargetSet();
                MotorStart();
            } else {
                PidAdjust();
                KeymouseAimingTargetSet();
                MotorStart();
            }
        } else {
            fric_flag = 0;
            shoot_mode = 0;
            PidAdjust();
            GimbalStop2TargetSet();
            MotorStart();
            gimbal.output_speed_[0] = 0;
            gimbal.output_speed_[1] = 0;
            shoot.fric_output_[0] = 0;
            shoot.fric_output_[1] = 0;
            shoot.trig_output_ = 0;
        }
    }

    // 右拨杆在下，急停模式
    if (remote.GetS2() == 2) {
        fric_flag = 0;
        shoot_mode = 0;
        // 左拨杆在上或中，发弹急停 OR 左拨杆在下，切换到全停模式（拨弹盘，摩擦轮目标速度设为0，双轴目标位置设为0度,输出强制为0,电机CAN信号直接发送0）
        if (remote.GetS1() == 1 || remote.GetS1() == 3) {
            PidAdjust();
            GimbalStop1TargetSet();
            MotorStart();
        } else {
            PidAdjust();
            GimbalStop2TargetSet();
            MotorStart();
            gimbal.output_speed_[0] = 0;
            gimbal.output_speed_[1] = 0;
            shoot.fric_output_[0] = 0;
            shoot.fric_output_[1] = 0;
            shoot.trig_output_ = 0;
        }
    }
}

void PidAdjust()
{
    PidFlagInit();
    PidAdjustByError();
    PidModeSwitch();
}

/**
 * @brief       PID参数标志初始化
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidFlagInit()
{
    pitchpid_switchflag = base_pid;
    yawpid_switchflag = base_pid;
}

/**
 * @brief       不同模式不同任务下PID设置选择判断,更改controlmode_pidset_flag进入不同PID模式，更改xxx_pid_switchflag进行不同任务下不同设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidModeSwitch()
{
    // Pitch轴PID调制
    switch (pitchpid_switchflag) {
        case base_pid:
            break;
        case pitch1_pid:
            PitchPidDemo1();
            break;
        case pitch2_pid:
            PitchPidDemo2();
            break;
        case pitch3_pid:
            PitchPidDemo3();
            break;
        case pitch4_pid:
            PitchPidDemo4();
            break;
        case pitch5_pid:
            PitchPidDemo5();
            break;
    }
    // Yaw轴PID调制
    switch (yawpid_switchflag) {
        case base_pid:
            YawPidDemo2();
            break;
        case yaw1_pid:
            YawPidDemo2();
            break;
        case yaw2_pid:
            YawPidDemo2();
            break;
    }
}

void PidAdjustByError()
{
    // 当Pitch轴误差达到一定范围时，更改PID参数进行自适应调节，分俯仰角调节以适应重心后偏
    if (gimbal.angle_[0].GetMeasure() > 2.f) {
        if (gimbal.angle_[0].GetError() > 2.5f || gimbal.angle_[0].GetError() < -2.5f) {
            pitchpid_switchflag = base_pid;
        } else if (gimbal.angle_[0].GetError() > 0.5f || gimbal.angle_[0].GetError() < -0.5f) {
            pitchpid_switchflag = pitch1_pid;
        } else {
            pitchpid_switchflag = pitch2_pid;
        }
    } else {
        if (gimbal.angle_[0].GetError() > 3.5f || gimbal.angle_[0].GetError() < -3.5f) {
            pitchpid_switchflag = pitch3_pid;
        } else if (gimbal.angle_[0].GetError() > 0.35f || gimbal.angle_[0].GetError() < -0.35f) {
            pitchpid_switchflag = pitch4_pid;
        } else {
            pitchpid_switchflag = pitch5_pid;
        }
    }

    // 当Yaw轴误差达到一定范围时，更改PID参数进行自适应调节
    if (gimbal.angle_[1].GetError() > 3.5f || gimbal.angle_[1].GetError() < -3.5f) {
        yawpid_switchflag = base_pid;
    } else if (gimbal.angle_[1].GetError() > 1.5f || gimbal.angle_[1].GetError() < -1.5f) {
        yawpid_switchflag = yaw1_pid;
    } else {
        yawpid_switchflag = yaw2_pid;
    }
}

/**
 * @brief      不连续射击模式控制
 *   @arg       None
 * @retval      None
 * @note        S1由上拨至中（顺序固定），不连续射击一次；拨至下时，连续射击
 */
void RemoteDisconShootCtrl()
{
    if (remote.GetS1() == 1) {
        trigger_flag = 0;
        shoot.SetFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(0.0f);
    }

    if (remote.GetS1() == 3 && trigger_flag == 0) {
        shoot.SetFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(45 * 1.0f);
        trigger_flag = 1;
    }
    if (remote.GetS1() == 2) {
        shoot.SetFlag(SPEED_FLAG);
    }
}