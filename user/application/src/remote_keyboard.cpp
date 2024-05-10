/**
 *******************************************************************************
 * @file      : remote_keyboard.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      RM2024      Jason Li        Victory
 *******************************************************************************
 * @attention :
 * s1:1       s2:1      遥控模式：摩擦轮开，拨弹盘关
 * s1:3       s2:1      遥控模式：摩擦轮开，拨弹盘低速
 * s1:2       s2:1      遥控模式：摩擦轮开，拨弹盘高速
 * s1:1       s2:3      键鼠模式（按F开关摩擦轮，鼠标左键开关拨弹盘，鼠标右键开关自瞄模式，下同）：拨弹盘低速
 * s1:3       s2:3      键鼠模式：拨弹盘中速
 * s1:2       s2:3      键鼠模式：拨弹盘高速（如果开启自瞄模式，拨弹盘默认为高速）
 * s1:1       s2:2      发弹急停模式：摩擦轮速度设为0，拨弹盘速度设为0，可旋转双轴
 * s1:3       s2:2      发弹急停模式：摩擦轮速度设为0，拨弹盘速度设为0，可旋转双轴
 * s1:2       s2:2      急停模式：摩擦轮，拨弹盘，双轴输出都发0
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
/* External variables --------------------------------------------------------*/
extern uint8_t auto_flag;  // 自瞄开关控制
/* Private function prototypes -----------------------------------------------*/
void PidFlagInit();
void PidModeSwitch();
void PidAdjust();
void PidAdjustByError();
void HaltOutput();

void ModeTask()
{
    /*遥控器控制模式选择*/
    // 右拨杆在上，遥控器控制模式
    if (remote.GetS2() == 1) {
        /*左拨杆在上——只转摩擦轮模式，中——拨弹盘低速模式，下——高速模式或（遥控器切换到自瞄模式，需要使用时取消注释）*/
        PidAdjust();
        RemoteAimingTargetSet();
        MotorStart();
    }

    // 右拨杆在中，键鼠模式
    if (remote.GetS2() == 3) {
        // 当启用空中支援且有剩余发弹时间(默认向下取整0.9==0）时才能用键鼠模式控制，否则为全停模式

        referee.aerial_robot_support_data_.airforce_status = 2;  // 测试用
        referee.aerial_robot_support_data_.time_remain = 1;      // 测试用
        if (referee.aerial_robot_support_data_.airforce_status == 2 && referee.aerial_robot_support_data_.time_remain > 0) {
            // 按鼠标右键键切换自瞄模式与手瞄模式
            if (auto_flag == 1) {
                PidAdjust();
                AutoAimingTargetSet();
                MotorStart();
            } else {
                PidAdjust();
                KeymouseAimingTargetSet();
                MotorStart();
            }
        } else {
            PidAdjust();
            GimbalStop2TargetSet();
            MotorStart();
            HaltOutput();
        }
    }

    // 右拨杆在下，急停模式
    if (remote.GetS2() == 2) {
        // 左拨杆在上或中，发弹急停 OR 左拨杆在下，切换到全停模式（拨弹盘，摩擦轮目标速度设为0，双轴目标位置设为0度,输出强制为0,电机CAN信号直接发送0）
        if (remote.GetS1() == 1 || remote.GetS1() == 3) {
            PidAdjust();
            GimbalStop1TargetSet();
            MotorStart();
        } else {
            PidAdjust();
            GimbalStop2TargetSet();
            MotorStart();
            HaltOutput();
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
}

void PidAdjustByError()
{
}

/**
 * @brief       不同模式不同任务下PID设置选择判断,更改controlmode_pidset_flag进入不同PID模式，更改xxx_pid_switchflag进行不同任务下不同设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidModeSwitch()
{
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