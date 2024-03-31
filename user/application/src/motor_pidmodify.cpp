/**
 *******************************************************************************
 * @file      : motor_modify.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, University of Science and Technology Beijing.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_pidmodify.h"

#include "gimbal.h"
#include "ins.h"
#include "remote.h"
#include "shoot.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
GimbalTargetSylloge gimbaltarget;
/* Private function prototypes -----------------------------------------------*/
void RemoteAimingTargetSet();
void KeymouseAimingTargetSet();
void AutoAimingTargetSet();
void MotorStart();
void GimbalStop1TargetSet();
void GimbalStop2TargetSet();

/**
 * @brief       初始化模式：遥控模式——基本模式：PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidSetRemote()
{
    /**
     * 左右摩擦轮3508电机的PID参数初始化
     */
    shoot.fric_speed_[0].Init(8.f, 0.f, 0.f, 1680.f * 4.f, 0.f);  // 限定最大值，防止突震，结合调试确定，参考速度闭环输出曲线 最大输出-16384-16384
    shoot.fric_speed_[1].Init(8.f, 0.f, 0.f, 1680.f * 4.f, 0.f);
    /**
     * 拨弹盘2006电机的PID参数初始化
     */
    shoot.trigger_pos_.Init(10.f, 0.f, 0.f, 216.f * 1.f, 0.f);    // 依据减速比(36/1*60)=2160*n得n转每秒
    shoot.trigger_speed_.Init(8.f, 0.f, 0.f, 1000.f * 5.f, 0.f);  // 限定最大值，防止突震，最大输出-10000-10000
    /**
     * Pitch轴2006电机的PID参数初始化
     */
    gimbal.angle_[0].Init(45.f, 0.f, 0.f, 216.f * 1.f, 0.0f);  // 依据减速比(36/1*60)=2160*n得n转每秒
    gimbal.speed_[0].Init(30.f, 0.f, 2.5f, 1000.f * 4.5f, 0.0f);
    ;  // 限定最大值，防止突震，最大输出-10000-10000   (25.f, .15f, 3.f, 1000.f * 3.8f, 0.0f);

    /**
     * Yaw轴6020电机的PID参数初始化
     */
    gimbal.angle_[1].Init(30.f, 10.f, 10.f, 60.f * 15.f, 0.0f);   // n转每秒
    gimbal.speed_[1].Init(20.f, 10.f, 10.f, 3000.f * 2.f, 0.0f);  // 限定最大值，防止突震，最大输出-30000-30000
}

/**
 * @brief       遥控模式——仰角Pitch轴模式1:当Pitch轴为接近目标值时，减小双环KP，减少KD，避免超调；增大KI，进一步弥补静差，提高控制精度。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemotePitchPidDemo1()
{
    gimbal.angle_[0].Init(45.f, 0.f, 0.f, 216.f * 1.f, 0.f);
    gimbal.speed_[0].Init(25.f, 1.f, 1.f, 1000.f * 3.5f, 0.f);
}

/**
 * @brief       遥控模式——仰角Pitch轴模式2:当Pitch轴更为接近目标值时，增大双环KP，减少KD，增强抗干扰能力；增大KI，最大弥补静差，提高控制精度。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemotePitchPidDemo2()
{
    gimbal.angle_[0].Init(66.f, 0.f, 0.f, 216.f * 1.f, 0.f);
    gimbal.speed_[0].Init(55.f, 1.5f, 0.f, 1000.f * 10.f, 0.f);
}

/**
 * @brief       遥控模式——俯角Pitch轴模式3:为方便调节，Pitch轴调节过程将模式3阶段设置为俯角调试起始阶段。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemotePitchPidDemo3()
{
    gimbal.angle_[0].Init(45.f, 0.f, 0.f, 216.f * 1.f, 0.f);
    gimbal.speed_[0].Init(35.f, 0.f, 2.f, 1000.f * 6.f, 0.f);
}

/**
 * @brief       遥控模式——俯角Pitch轴模式4:当Pitch轴为接近目标值时，减小双环KP，减少KD，避免超调；增大KI，进一步弥补静差，提高控制精度。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemotePitchPidDemo4()
{
    gimbal.angle_[0].Init(45.f, 0.f, 0.f, 216.f * 1.f, 0.f);
    gimbal.speed_[0].Init(20.f, 1.5f, 0.5f, 1000.f * 5.f, 0.f);
}

/**
 * @brief       遥控模式——俯角Pitch轴模式5:当Pitch轴更为接近目标值时，增大双环KP，减少KD，增强抗干扰能力；增大KI，最大弥补静差，提高控制精度。
 * @brief       同时最终输出应该控制上限，避免过冲引发震荡。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemotePitchPidDemo5()
{
    gimbal.angle_[0].Init(66.f, 0.f, 0.f, 216.f * 1.f, 0.f);
    gimbal.speed_[0].Init(50.f, 2.5f, 0.f, 1000.f * 10.f, 0.f);
}

/**
 * @brief       遥控模式——Yaw轴模式1:当yaw轴为接近目标值时，减小双环KP，减少KD，避免超调。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemoteYawPidDemo1()
{
    gimbal.angle_[1].Init(10.f, 0.f, 5.f, 60.f * 15.f, 0.f);
    gimbal.speed_[1].Init(5.f, 0.f, 5.f, 3000.f * 1.f, 0.f);
}

/**
 * @brief       遥控模式——Yaw轴模式2:当yaw轴更为接近目标值时，增大双环KP，减少KD，增强抗干扰能力。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemoteYawPidDemo2()
{
    gimbal.angle_[1].Init(20.f, 0.f, 1.f, 60.f * 15.f, 0.f);
    gimbal.speed_[1].Init(20.f, 0.f, 1.f, 3000.f * 3.f, 0.f);
}

/**
 * @brief       键鼠模式PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidSetKeymouse()
{
}

/**
 * @brief       键鼠模式PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void KeymPitchPidDemo1()
{
}

/**
 * @brief       自瞄模式PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidSetAutoaim()
{
}

/**
 * @brief       自瞄模式PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void AutoPitchPidDemo1()
{
}

/**
 * @brief       开启遥控器控制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemoteControlMode()
{
    RemoteAimingTargetSet();
    MotorStart();
}

/**
 * @brief       开启键鼠控制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void KeymouseControlMode()
{
    KeymouseAimingTargetSet();
    MotorStart();
}

/**
 * @brief       开启自瞄控制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void AutoControlMode()
{
    AutoAimingTargetSet();
    MotorStart();
}

/**
 * @brief       急停模式1
 *   @arg       None
 * @retval      None
 * @note        None
 */
void GimbalStop1ControlMode()
{
    GimbalStop1TargetSet();
    MotorStart();
}

/**
 * @brief       急停模式2
 *   @arg       None
 * @retval      None
 * @note        None
 */
void GimbalStop2ControlMode()
{
    GimbalStop2TargetSet();
    MotorStart();
}

/**
 * @brief       开启云台电机PID输出计算
 *   @arg       None
 * @retval      None
 * @note        None
 */
void MotorStart()
{
    shoot.Control();
    gimbal.Control();
}

/**
 * @brief      遥控器模式，电机目标值设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RemoteAimingTargetSet()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 100.f * 60.f;  // =6000 无减速箱，依据n*60得n转每秒
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    /*35s支援时间，估计25s发弹时间，发弹量500，一转8发，62.5转，则预计比赛时速度需要62.5/25=2.5转/秒*/
    gimbaltarget.turn_magazine_target = 2.5f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    if (remote.GetCh1() < 2.f && remote.GetCh1() > -2.f) {
        gimbaltarget.pitch_target = remote.GetCh1() * 0.f;
    } else if (remote.GetCh1() > 2.f) {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 20.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 20.0f);   // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为20度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    if (remote.GetCh2() < 2.f && remote.GetCh2() > -2.f) {
        gimbaltarget.yaw_target = remote.GetCh2() * 0.f;
    } else {
        gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    }
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);
}

/**
 * @brief      键鼠模式，电机目标值设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void KeymouseAimingTargetSet()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 100.f * 60.f;  // =6000 无减速箱，依据n*60得n转每秒
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 1.125f * 60.0f * 36.0f;  //=2430 依据减速比n*60*（36/1）得n转每秒
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    float pitch_target = remote.GetMouseY();
    if (remote.GetMouseY() < 1.f && remote.GetMouseY() > -1.f) {
        pitch_target = 0.f;
    }  // 死区设置，防止误漂移。
    /* 实测陀螺仪抬头为负，低头为正，第一人称，鼠标前移抬头，后移低头*/
    gimbaltarget.pitch_target += pitch_target * 0.00085f;  // 根据鼠标灵敏度结合操作手的操作习惯实际测试后调整数值。测试鼠标DPI为1600。鼠标前移负值，后移正值。
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 20.0f);    // 抬头最大值角度为10度，低头最大角度为20度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);   // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    float yaw_target = remote.GetMouseX();
    if (remote.GetMouseX() < 1.f && remote.GetMouseX() > -1.f) {
        yaw_target = 0.f;
    }
    gimbaltarget.yaw_target += yaw_target * 0.00075f;
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);
}

/**
 * @brief      自瞄模式，电机目标值设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void AutoAimingTargetSet()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 100.f * 60.f;  // =6000 无减速箱，依据n*60得n转每秒
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 1000;
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    if (remote.GetCh1() < 1.f && remote.GetCh1() > -1.f) {
        gimbaltarget.pitch_target = 0.f;
    } else if (remote.GetCh1() > 1.f) {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 20.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 20.0f);   // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为20度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右45度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);
}

/**
 * @brief      发弹急停模式，程序控制摩擦轮速度为0，拨弹盘速度为0，双轴可用
 *   @arg       None
 * @retval      None
 * @note        None
 */
void GimbalStop1TargetSet()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 0;
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 0;
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    if (remote.GetCh1() < 1.f && remote.GetCh1() > -1.f) {
        gimbaltarget.pitch_target = remote.GetCh1() * 0.f;
    } else if (remote.GetCh1() > 1.f) {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 20.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 20.0f);   // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为20度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    if (remote.GetCh2() < 5.f && remote.GetCh2() > -5.f) {
        gimbaltarget.yaw_target = remote.GetCh2() * 0.f;
    } else {
        gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    }
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);
}

/**
 * @brief      全停模式，程序控制摩擦轮速度为0，拨弹盘速度为0，双轴回到0度位置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void GimbalStop2TargetSet()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 0;
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 0;
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    gimbaltarget.pitch_target = 0;
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    gimbaltarget.yaw_target = 0;
    gimbal.SetYawPosition(gimbaltarget.yaw_target);
}
