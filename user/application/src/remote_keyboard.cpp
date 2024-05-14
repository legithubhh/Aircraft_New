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
 * 键鼠控制按键：F键开关摩擦轮，R键进行退弹，鼠标左键开关拨弹盘，鼠标右键开关自瞄模式
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "remote_keyboard.h"

#include "cmsis_os.h"
#include "gimbal.h"
#include "motor_pidmodify.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Flag flag;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void PidFlagInit();
void PidModeSwitch();
void PidAdjust();
void PidAdjustByError();
void HaltOutput();

void ModeTask()
{
    // 键鼠模式摩擦轮控制，按F键切换摩擦轮状态，按R键切换自瞄状态
    for (uint8_t i = 0; i < 16; i++) {
        if (ref_keymouse.referee_key_press[i] != flag.last_key_press[i]) {
            if (ref_keymouse.referee_key_press[KEY_F] == 1) {
                flag.fric_flag = !flag.fric_flag;
            }
            if (ref_keymouse.referee_key_press[KEY_R] == 1) {
                flag.return_trig_count++;
            }
            flag.last_key_press[i] = ref_keymouse.referee_key_press[i];
        }
        flag.auto_flag = ref_keymouse.comma_data.right_button_down;
        flag.trig_flag = ref_keymouse.comma_data.left_button_down;
    }  // 图传键鼠链路状态切换

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
            if (flag.auto_flag == 1) {
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

    // 退弹模式，拨弹盘反转0.1s，但急停模式优先
    if (flag.return_trig_count != flag.last_return_trig_count) {
        TriggerReturnTargetSet();
        MotorStart();
        flag.last_return_trig_count = flag.return_trig_count;
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.yaw_output_speed, (int16_t)shoot.trig_output_);
        gimbal.pitch_motor.MITSend(&hcan1, 0.f, 0.f, 0.f, 0.f, gimbal.pitch_output_torque);
        osDelay(50);
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