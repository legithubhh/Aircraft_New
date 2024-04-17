/**
 *******************************************************************************
 * @file      : motor_pidmodify.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      RM2024      Jason Li        Victory
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
float pitch_target;
float yaw_target;
GimbalTargetSylloge gimbaltarget;
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       初始化模式：遥控模式——基本模式：PID参数集中调制
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidSetInitial()
{
    /**
     * 左右摩擦轮3508电机的PID参数初始化
     */
    shoot.fric_speed_[0].Init(8.f, 0.f, 0.f, 1680.f * 4.f, 0.f);  // 限定最大值，防止突震，结合调试确定，参考速度闭环输出曲线 最大输出-16384-16384
    shoot.fric_speed_[1].Init(8.f, 0.f, 0.f, 1680.f * 4.f, 0.f);
    /**
     * 拨弹盘2006电机的PID参数初始化
     */
    shoot.trigger_pos_.Init(50.f, 0.f, 0.f, 216.f * 10.f, 0.f);   // 依据减速比(36/1*60)=2160*n得n转每秒（最快）
    shoot.trigger_speed_.Init(6.f, 0.f, 0.f, 1000.f * 6.f, 0.f);  // 限定最大值，防止突震，最大输出-10000-10000

    /**
     * Yaw轴3508电机的PID参数初始化
     */
    gimbal.yaw_angle.Init(40.f, 0.f, 0.f, 80.f * 5.f, 0.0f);
    gimbal.yaw_speed.Init(20.f, 10.f, 0.f, 3000.f * 1.f, 0.0f);

    /**
     * Pitch轴DM电机的PID参数初始化
     */
    gimbal.pitch_angle.Init(0.1f, 0.f, 0.f, 1.f * 3.f, 0.0f);//输出限幅控制最大力矩
    gimbal.pitch_torque.Init(1.f, 0.f, 0.f, 1.f * 3.f, 0.0f);//调整最大力矩

}

/**
 * @brief       遥控模式——Yaw轴模式1:当yaw轴为接近目标值时，减小双环KP，避免超调。增加KD，快速抑制震荡。增加KI，避免输出不够而卡住。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void YawPidDemo1()
{
    gimbal.yaw_angle.Init(35.f, 0.f, 2.5f, 80.f * 1.75f, 0.f);
    gimbal.yaw_speed.Init(10.f, 20.f, 5.f, 3000.f * 0.6f, 0.f);
}

// gimbal.yaw_speed.Init(10.f, 20.f, 5.f, 3000.f * 0.6f, 0.f);

// gimbal.yaw_speed.Init(10.f, 10.f, 5.f, 3000.f * 0.6f, 0.f);

/**
 * @brief       遥控模式——Yaw轴模式2:当yaw轴更为接近目标值时，增大双环KP，增加KD，增强抗干扰能力；取消积分，避免静态干扰。
 *   @arg       None
 * @retval      None
 * @note        None
 */
void YawPidDemo2()
{
    gimbal.yaw_angle.Init(66.f, 0.f, 1.85f, 99.f * 1.f, 0.f);//输出限幅控制最大速度
    gimbal.yaw_speed.Init(66.f, 0.f, 1.85f, 6534.f * 1.f, 0.f);
    gimbal.yaw_speed.i_out_ = 0;
}

// gimbal.yaw_angle.Init(75.f, 0.f, 1.54f, 112.5f * 1.f, 0.f);
// gimbal.yaw_speed.Init(75.f, 0.f, 1.54f, 8437.5f * 1.f, 0.f);

// gimbal.yaw_angle.Init(72.f, 0.f, 1.4f, 108.f * 1.f, 0.f);
// gimbal.yaw_speed.Init(72.f, 0.f, 1.4f, 7776.f * 1.f, 0.f);

// gimbal.yaw_angle.Init(70.f, 0.f, 1.31f, 105.f * 1.f, 0.f);
// gimbal.yaw_speed.Init(70.f, 0.f, 1.31f, 7350.f * 1.f, 0.f);

// gimbal.yaw_angle.Init(69.f, 0.f, 1.215f, 103.5f * 1.f, 0.f);
// gimbal.yaw_speed.Init(69.f, 0.f, 1.215f, 7141.5f * 1.f, 0.f);上限Kp，防止震荡与抵抗外力（稳定性）最优性价比

// gimbal.yaw_angle.Init(66.f, 0.f, 0.9f, 99.f * 1.f, 0.f);
// gimbal.yaw_speed.Init(66.f, 0.f, 0.9f, 6534.f * 1.f, 0.f);

// gimbal.yaw_angle.Init(50.f, 0.f, 0.55f, 75.f * 1.f, 0.f);
// gimbal.yaw_speed.Init(50.f, 0.f, 0.55f, 3750.f * 1.f, 0.f);

// Kp越大，俯仰角抖动越厉害，Kd大了可能导致在稳定后出现自震荡，小了则可能抑制不了比例环节导致震荡。向左转抖动比向右转厉害

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
    gimbaltarget.friction_wheel_target = 100.f * 2.f;  // =6000 无减速箱，依据n*60得n转每秒
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    /*35s支援时间，估计25s发弹时间，发弹量500，一转8发，62.5转，则预计比赛时速度需要62.5/25=2.5转/秒*/
    if (remote.GetS1() == 3) {
        gimbaltarget.turn_magazine_target = 1.f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (remote.GetS1() == 2) {
        gimbaltarget.turn_magazine_target = 2.f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    }

    /*位置式角度控制*/

    // // Pitch轴目标值设置
    // if (remote.GetCh1() < 2.f && remote.GetCh1() > -2.f) {
    //     gimbaltarget.pitch_target = remote.GetCh1() * 0.f;
    // } else if (remote.GetCh1() > 2.f) {
    //     gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 8.f;
    // }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    // else {
    //     gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    // }
    // VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 8.0f);    // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为8度
    // gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // // Yaw轴目标值设置
    // if (remote.GetCh2() < 2.f && remote.GetCh2() > -2.f) {
    //     gimbaltarget.yaw_target = remote.GetCh2() * 0.f;
    // } else {
    //     gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    // }
    // VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    // gimbal.SetYawPosition(-gimbaltarget.yaw_target);

    /*位置增量式角度控制*/

    // Pitch轴目标值设置
    if (remote.GetCh1() > 600.f || remote.GetCh1() < -600.f) {
        pitch_target = remote.GetCh1() / 660.f * 0.01f;
    } else {
        pitch_target = 0.f;
    }
    gimbaltarget.pitch_target += pitch_target;
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 8.0f);    // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为8度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    if (remote.GetCh2() > 600.f || remote.GetCh2() < -600.f) {
        yaw_target = remote.GetCh2() / 660.f * 0.015f;
    } else {
        yaw_target = 0.f;
    }
    gimbaltarget.yaw_target += yaw_target;
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);
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
    pitch_target = remote.GetMouseY();
    if (remote.GetMouseY() < 3.f && remote.GetMouseY() > -3.f) {
        pitch_target = 0.f;
    }  // 死区设置，防止误漂移。
    /* 实测陀螺仪抬头为负，低头为正，第一人称，鼠标前移抬头，后移低头*/
    gimbaltarget.pitch_target += pitch_target * 0.00085f;  // 根据鼠标灵敏度结合操作手的操作习惯实际测试后调整数值。测试鼠标DPI为1600。鼠标前移负值，后移正值。
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 20.0f);    // 抬头最大值角度为10度，低头最大角度为20度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);   // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    yaw_target = remote.GetMouseX();
    if (remote.GetMouseX() < 3.f && remote.GetMouseX() > -3.f) {
        yaw_target = 0.f;
    }
    gimbaltarget.yaw_target += yaw_target * 0.00075f;
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);
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
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 8.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 8.0f);    // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为8度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 负号使得遥控器抬头为正，低头为负

    // Yaw轴目标值设置
    gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右45度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);
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

    /*位置式角度控制*/

    // Pitch轴目标值设置
    if (remote.GetCh1() < 2.f && remote.GetCh1() > -2.f) {
        gimbaltarget.pitch_target = remote.GetCh1() * 0.f;
    } else if (remote.GetCh1() > 2.f) {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 8.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 8.0f);    // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为8度
    gimbal.SetPitchPosition(gimbaltarget.pitch_target);  // 根据实际情况调整正负号

    // Yaw轴目标值设置
    if (remote.GetCh2() < 2.f && remote.GetCh2() > -2.f) {
        gimbaltarget.yaw_target = remote.GetCh2() * 0.f;
    } else {
        gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 30.f;
    }
    VAL_LIMIT(gimbaltarget.yaw_target, -30.0f, 30.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右30度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);
}

/**
 * @brief      全停模式，拨弹盘，摩擦轮目标速度设为0，双轴目标位置设为0度,输出强制为0,电机CAN信号直接发送0
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
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);
}
