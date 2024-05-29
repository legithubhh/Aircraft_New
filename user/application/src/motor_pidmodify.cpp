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

#include "ins.h"
#include "remote.h"
#include "remote_keyboard.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float pitch_target;
float yaw_target;
GimbalTargetSylloge gimbaltarget;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void HaltOutput();
extern int16_t GetRefMouseX();
extern int16_t GetRefMouseY();
extern float GetTargetPitch();
extern float GetTargetYaw();

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
    shoot.fric_speed_[0].Init(10.f, 1.f, 0.0f, 1680.f * 4.f, 0.f);  // 限定最大值，防止突震，结合调试确定，参考速度闭环输出曲线 最大输出-16384-16384
    shoot.fric_speed_[1].Init(10.f, 1.f, 0.0f, 1680.f * 4.f, 0.f);
    /**
     * 拨弹盘2006电机的PID参数初始化
     */
    shoot.trigger_pos_.Init(10.f, 0.f, 0.f, 1.f * 2160.f, 0.f);   // 依据减速比(36/1*60)=2160*n得n转每秒（最快）
    shoot.trigger_speed_.Init(5.f, 1.f, 0.f, 1000.f * 9.f, 0.f);  // 限定最大值，防止突震，最大输出-10000-10000

    /**
     * Yaw轴3508电机的PID参数初始化
     */
    gimbal.yaw_angle.Init(30.f, 30.f, 2.5f, 1500.f * 1.f, 0.f);  // 输出限幅控制最大速度
    gimbal.yaw_speed.Init(80.f, 0.f, 3.f, 10000.f * 1.f, 0.f);

    // gimbal.yaw_angle.Init(30.f, 30.f, 2.5f, 50.f * 1.f, 0.f);  // 输出限幅控制最大速度
    // gimbal.yaw_speed.Init(80.f, 0.f, 3.f, 3200.f * 1.f, 0.f);

    // gimbal.yaw_angle.Init(66.f, 0.f, 2.05f, 66.f * 1.5f, 0.f);  // 输出限幅控制最大速度
    // gimbal.yaw_speed.Init(66.f, 0.f, 2.05f, 6534.f * 1.f, 0.f);

    // Kp越大，俯仰角抖动越厉害，Kd大了可能导致在稳定后出现自震荡，小了则可能抑制不了比例环节导致震荡。向左转抖动比向右转厉害

    /**
     * Pitch轴DM电机的PID参数初始化 超调——超出目标值距离；震荡频率——在目标值附近来回震荡频率；震荡时间——震荡的持续时间；
     * 往往先用Ki作为主要输出（注意限幅），KpKd作为辅助，如果有静态误差，则双环都给Ki，如果静态误差影响很小，则只位置环给Ki就行；
     * 首先确定速度环输出限幅，这决定所需力矩（往往先给一个小值，调到能稳定在平衡位置时，先不管震荡，超调和灵敏度，看看定位能力），如果不足则继续加大双环Kp；
     * 定位能力——主要由双环Kp乘积以及定位区间决定，如我要定位在1°以内，Kp1=1，Kp2=2，则在误差为1°时输出力矩可达1.2N*m（对达妙MIT纯力矩控制模式而言）
     * 然后如果定位能力满足需求，则调节保持速度环输出限幅不变，增大双环Kp，增大位置环输出限幅——从而增加响应灵敏度；
     * 如果灵敏度达到要求，此时可以解决超调与低频震荡，可以增减速度环Ki，调节双环Kd；
     * Ki的输出是滞后的，这也能在Kp突变时对反向的输出起到抵消作用，从而抑制了震荡；Kp与Ki相辅相成，相互促进又相互抑制；
     * 微分滤波数值越大，微分环节对当前Kd的响应越小，输出越取决于以往Kd的平均值；有利于过滤较大跳变的影响保持稳定的输出，不利于抑制当前的输出震荡，受时间影响较大；
     * 在Kp确定可行后，高频震荡，自震荡往往由Kd引起；超调以及低频震荡，由Kp引起，如果Ki超过稳态误差，也会由Ki引起；
     * 向下静态力矩0.05-0.4N*m，向上0.2-0.4N*m
     *
     * 位置环输出不限幅，速度环限幅，
     */

    gimbal.pitch_angle.Init(0.8f, 2.8f, 0.005f, 5.f * 1.f, 0.0f);  // 无摄像头配重版本
    gimbal.pitch_speed.Init(0.9f, 1.5f, 0.005f, 3.f * 1.f, 0.0f);

    // gimbal.pitch_angle.Init(1.0f, 2.f, 0.007f, 5.f * 1.f, 0.0f);  // 有摄像头配重版本
    // gimbal.pitch_speed.Init(1.0f, 1.5f, 0.01f, 3.f * 1.f, 0.0f);

    // gimbal.pitch_angle.Init(0.6f, 5.f, 0.15f, 5.f * 1.f, 0.0f);   // 微分滤波1（位置环kI作为主要输出，来延缓输出，抑制震荡，同时速度环Kp作为主要输出，快速响应）
    // gimbal.pitch_speed.Init(1.2f, 6.f, 0.06f, 5.f * 1.f, 0.0f);  // 微分滤波1 稳定3，定位能力3 超调0.1 震荡频率0.1 震荡时间0.1

    // gimbal.pitch_angle.Init(1.0f, 3.f, 0.3f, 10.f * 1.f, 0.0f);    // 微分滤波1（位置环kI作为主要输出，来延缓输出，抑制震荡，同时速度环Kp作为主要输出，快速响应）
    // gimbal.pitch_speed.Init(0.9f, 5.f, 0.04f, 10.f * 1.f, 0.0f);  // 微分滤波1 稳定3，定位能力4 超调0.15 震荡频率0.15 震荡时间0.1

    // gimbal.pitch_angle.Init(1.f, 3.0f, 0.2f, 2.5f * 1.f, 0.0f);  // 微分滤波1（位置环kI作为主要输出，来延缓输出，抑制震荡，同时速度环Kp作为主要输出，快速响应）
    // gimbal.pitch_speed.Init(1.f, 6.f, 0.2f, 0.8f * 1.f, 0.0f);  // 微分滤波1 稳定2，定位能力4 超调0.2 震荡频率0.1 震荡时间0.1

    // gimbal.pitch_angle.Init(1.2f, 3.f, 0.003f, 2.5f * 1.f, 0.0f);    // 微分滤波100（位置环kI作为主要输出，来延缓输出，抑制震荡，同时速度环Kp作为主要输出，快速响应）
    // gimbal.pitch_speed.Init(1.2f, 1.5f, 0.003f, 1.2f * 1.f, 0.0f);  // 微分滤波100 稳定2，定位能力4 超调0.2 震荡频率0.1 震荡时间0.1

    // gimbal.pitch_angle.Init(2.f, 0.35f, 0.04f, 3.5f * 1.f, 0.0f);     //微分滤波10
    // gimbal.pitch_speed.Init(0.4f, 0.2f, 0.3f, 1.4f * 1.f, 0.0f);  //微分滤波20 稳定2，定位能力3 超调0.5 震荡频率0.3 震荡时间0.5

    // gimbal.pitch_angle.Init(2.f, 0.5f, 0.04f, 4.f * 1.f, 0.0f);      //微分滤波10
    // gimbal.pitch_speed.Init(0.45f, 0.35f, 0.2f, 1.8f * 1.f, 0.0f);  //微分滤波20 稳定2，定位能力4 超调0.3 震荡频率0.5 震荡时间0.2

    // gimbal.pitch_angle.Init(2.f, 0.5f, 0.04f, 2.f * 1.f, 0.0f);    // 输出限幅控制最大速度  微分滤波10
    // gimbal.pitch_speed.Init(0.35f, 0.3f, 0.25f, 0.7f * 1.f, 0.0f);  // 输出限幅控制最大力矩 微分滤波20 稳定1，定在原位的能力2 超调0.5 震荡0.5

    // gimbal.pitch_angle.Init(2.f, 0.5f, 0.15f, 2.f * 1.f, 0.0f);       // 输出限幅控制最大速度  微分滤波1
    // gimbal.pitch_speed.Init(0.3f, 0.75f, 0.3f, 0.6f * 1.f, 0.0f);  // 输出限幅控制最大力矩 微分滤波10 稳定1，定在原位的能力1 超调1 震荡0.1

    // gimbal.pitch_angle.Init(2.5f, 3.f, 1.5f, 2.5f * 1.f, 0.0f);     // 输出限幅控制最大速度  微分滤波10
    // gimbal.pitch_speed.Init(0.25f, 0.75f, 0.25f, 1.f * 1.f, 0.0f);  // 输出限幅控制最大力矩 微分滤波10 小幅KP震荡，源于微分

    // gimbal.pitch_angle.Init(2.5f, 3.f, 1.f, 2.5f * 1.f, 0.0f);     // 输出限幅控制最大速度  微分滤波50
    // gimbal.pitch_speed.Init(0.25f, 0.75f, 0.15f, 1.f * 1.f, 0.0f);  // 输出限幅控制最大力矩 微分滤波50 大幅KP震荡，源于比例

    // 位置环比例环节主输出并负责抵抗干扰，积分环节辅助输出，微分环节抑制超调，微分滤波防止突变；速度环积分环节主导输出，积分环节补充输出并负责抵抗干扰，微分滤波防止突变
    // 增加微分环节，增加滤波系数，无法消除源于积分的震荡；减少微分环节，减少滤波系数，无法消除突变震荡；增大滤波系数可抑制突变，但无法消除源于积分的震荡，且会削弱微分系数的影响；
    // 当位置环比例系数从2.0增大到2.5后，似乎无法调整，超出了上限。调整速度环比例系数还有上升空间
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
    gimbaltarget.friction_wheel_target = 120.f * 60.f;  // =6900 无减速箱，依据n*60得n转每秒 115.f * 60.f 摩擦轮温度足够高后达到30m/s射速
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    /*35s支援时间，估计25s发弹时间，发弹量500，一转8发，62.5转，则预计比赛时速度需要62.5/25=2.5转/秒*/
    if (remote.GetS1() == 3) {
        gimbaltarget.turn_magazine_target = 2.5f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else {
        gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
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
    //     gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 55.f;
    // }
    // VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右55度
    // gimbal.SetYawPosition(-gimbaltarget.yaw_target);

    /*位置增量式角度控制*/

    // Pitch轴目标值设置
    if (remote.GetCh1() > 600.f || remote.GetCh1() < -600.f) {
        pitch_target = remote.GetCh1() / 660.f * 0.01f;
    } else {
        pitch_target = 0.f;
    }
    gimbaltarget.pitch_target += pitch_target;
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 30.0f);   // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为30度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 陀螺仪向上为正，遥杆前推为正。负号使得遥控器前推为负，低头；符合操作习惯；

    // Yaw轴目标值设置
    if (remote.GetCh2() > 600.f || remote.GetCh2() < -600.f) {
        yaw_target = remote.GetCh2() / 660.f * 0.015f;
    } else {
        yaw_target = 0.f;
    }
    gimbaltarget.yaw_target += yaw_target;
    VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右55度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);    // 陀螺仪向左为正，遥杆向右为正。前取负号，使得遥杆向左为正，左转；符合操作习惯；
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
    if (flag.fric_flag == 1) {
        gimbaltarget.friction_wheel_target = 120.f * 60.f;  // =6000 无减速箱，依据n*60得n转每秒
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    } else if (flag.fric_flag == 0) {
        gimbaltarget.friction_wheel_target = 0.f;
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    } else {
        gimbaltarget.friction_wheel_target = 0.f;
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    }

    // 拨弹盘目标值设置
    if (flag.trig_flag == 0) {
        gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 1) {
        gimbaltarget.turn_magazine_target = 1.5f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 3) {
        gimbaltarget.turn_magazine_target = 2.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 2) {
        gimbaltarget.turn_magazine_target = 2.5f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else {
        gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    }

    /*自瞄和键鼠切换时，因为坐标系正负不同，所以需要调整*/
    if (flag.auto_flag == 0 && flag.last_auto_flag == 1) {
        gimbaltarget.pitch_target = -gimbaltarget.pitch_target;
        gimbaltarget.yaw_target = -gimbaltarget.yaw_target;
        flag.last_auto_flag = flag.auto_flag;
    }

    // Pitch轴目标值设置
    pitch_target = remote.GetMouseY();
    if (pitch_target < 3.f && pitch_target > -3.f) {
        pitch_target = 0.f;
    }  // 死区设置，防止误漂移。
    /* 实测陀螺仪抬头为负，低头为正，第一人称，鼠标前移抬头，后移低头*/
    gimbaltarget.pitch_target += pitch_target * 0.001f;   // 根据鼠标灵敏度结合操作手的操作习惯实际测试后调整数值。测试鼠标DPI为1600。
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 30.0f);   // 抬头最大值角度为10度，低头最大角度为30度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 陀螺仪向上为正，鼠标往下为正。前取负号，使得鼠标向上为正，抬头；符合操作习惯；

    // Yaw轴目标值设置
    yaw_target = remote.GetMouseX();
    if (yaw_target < 3.f && yaw_target > -3.f) {
        yaw_target = 0.f;
    }
    gimbaltarget.yaw_target += yaw_target * 0.001f;
    VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 最大值为向左向右55度
    gimbal.SetYawPosition(-gimbaltarget.yaw_target);    // 陀螺仪向左为正，鼠标往右为正。前取负号，使得鼠标向左为正，左转；符合操作习惯；
}

/**
 * @brief      键鼠自瞄模式，电机目标值设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void AutoAimingTargetSet()
{
    // 摩擦轮目标值设置
    if (flag.fric_flag == 1) {
        gimbaltarget.friction_wheel_target = 120.f * 60.f;  // =6000 无减速箱，依据n*60得n转每秒
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    } else if (flag.fric_flag == 0) {
        gimbaltarget.friction_wheel_target = 0.f;
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    } else {
        gimbaltarget.friction_wheel_target = 0.f;
        shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);
    }

    // 拨弹盘目标值设置
    if (flag.trig_flag == 0) {
        gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 1) {
        gimbaltarget.turn_magazine_target = 1.5f * 60.0f * 36.0f;  // =2430 依据减速比n*60*（36/1）得n转每秒
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 3) {
        gimbaltarget.turn_magazine_target = 2.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else if (flag.trig_flag == 1 && remote.GetS1() == 2) {
        gimbaltarget.turn_magazine_target = 2.5f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    } else {
        gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
        shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);
    }

    // Pitch轴目标值设置
    gimbaltarget.pitch_target = GetTargetPitch();
    VAL_LIMIT(gimbaltarget.pitch_target, -30.f, 10.0f);  // 抬头最大值角度为10度，低头最大角度为30度
    gimbal.SetPitchPosition(gimbaltarget.pitch_target);  // 陀螺仪向上为正，视觉低头给负值，抬头给正值，符合陀螺仪值；
    // Yaw轴目标值设置
    gimbaltarget.yaw_target = GetTargetYaw();
    VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 向左向右55度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);     // 陀螺仪向左为正，视觉左转给正值，右转给负值，符合陀螺仪值；

    flag.last_auto_flag = 1;
}

/**
 * @brief      遥控器自瞄测试，电机目标值设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void AutoAimingTargetTest()
{
    // 摩擦轮目标值设置
    gimbaltarget.friction_wheel_target = 0.f;
    shoot.SetFricSpeed(gimbaltarget.friction_wheel_target);

    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
    shoot.SetTriggerSpeed(-gimbaltarget.turn_magazine_target);

    // Pitch轴目标值设置
    pitch_target = GetTargetPitch();
    gimbaltarget.pitch_target = pitch_target;
    VAL_LIMIT(gimbaltarget.pitch_target, -30.f, 10.0f);  // 抬头最大值角度为10度，低头最大角度为30度
    gimbal.SetPitchPosition(gimbaltarget.pitch_target);  // 陀螺仪向上为正，视觉低头给负值，抬头给正值，符合陀螺仪值；
    // Yaw轴目标值设置
    yaw_target = GetTargetYaw();
    gimbaltarget.yaw_target = yaw_target;
    VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 向左向右55度
    gimbal.SetYawPosition(gimbaltarget.yaw_target);     // 陀螺仪向左为正，视觉左转给正值，右转给负值，符合陀螺仪值；

    // 先拨至发弹急停模式再切换到键鼠或者遥控器模式；
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
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 30.f;
    }  // 实测仰角为正，俯角为负————抬头遥杆向后输出负值，低头遥杆向前输出正值
    else {
        gimbaltarget.pitch_target = remote.GetCh1() / 660.f * 10.f;
    }
    VAL_LIMIT(gimbaltarget.pitch_target, -10.f, 30.0f);   // 遥控器右手柄上下通道控制，抬头最大值角度为10度，低头最大角度为30度
    gimbal.SetPitchPosition(-gimbaltarget.pitch_target);  // 根据实际情况调整正负号

    // Yaw轴目标值设置
    if (remote.GetCh2() < 2.f && remote.GetCh2() > -2.f) {
        gimbaltarget.yaw_target = remote.GetCh2() * 0.f;
    } else {
        gimbaltarget.yaw_target = remote.GetCh2() / 660.f * 55.f;
    }
    VAL_LIMIT(gimbaltarget.yaw_target, -55.0f, 55.0f);  // 遥控器左手柄左右通道控制，最大值为向左向右55度
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

    HaltOutput();
}

/**
 * @brief      将输出计算结果置零，停止输出
 *   @arg       None
 * @retval      None
 * @note        None
 */
void HaltOutput()
{
    gimbal.pitch_output_pos = 0;
    gimbal.pitch_output_speed = 0;
    gimbal.yaw_output_speed = 0;
    shoot.fric_output_[0] = 0;
    shoot.fric_output_[1] = 0;
    shoot.trig_output_ = 0;
}

/**
 * @brief      退弹模式，拨弹盘反转
 *   @arg       None
 * @retval      None
 * @note        None
 */
void TriggerReturnTargetSet()
{
    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 1.f * 60.0f * 36.0f;
    shoot.SetTriggerSpeed(gimbaltarget.turn_magazine_target);
}

/**
 * @brief      退弹前，先让拨弹盘0速
 *   @arg       None
 * @retval      None
 * @note        None
 */
void TriggerZeroTargetSet()
{
    // 拨弹盘目标值设置
    gimbaltarget.turn_magazine_target = 0.f * 60.0f * 36.0f;
    shoot.SetTriggerSpeed(gimbaltarget.turn_magazine_target);
}
