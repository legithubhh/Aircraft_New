/**
 *******************************************************************************
 * @file      : infantry.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      RM2024      Jason Li        Victory
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "aircraft.h"

#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "motor_pidmodify.h"
#include "remote.h"
#include "remote_keyboard.h"
#include "shoot.h"
#include "vision.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CANC620IdSet();
/**
 * @brief Initializes the system.
 */
void InfantrySystemInit()
{
    RemoteControlInit(&huart3);
    referee.Init(&huart6);
    ref_keymouse.Init(&huart1);
    gimbal.MotorInit();
    shoot.MotorInit();
}

/**
 * @brief Initializes the gimbal.
 */
void GimbalInit()
{
    gimbal.PidInit();
    shoot.PidInit();
    /*PID初始化*/
    PidSetInitial();
}

/**
 * @brief This function handles the Gimbal task.
 */
void GimbalTask()
{
    /*与视觉通信*/
    vision.Send();
    vision.Ctrl();
    /*电机控制，集中发送CAN信号，发送给电机的信号必须为整形*/
    // CAN1总线0X1FF对应电机ID，ID号1为拨弹盘2006电机
    // CAN1总线0X200对应电机ID，ID号1-4分别为摩擦轮3508电机1，摩擦轮3508电机2，Yaw轴3508电机，Pitch轴2006电机

    // Pitch轴的DM电机需要先使能再发送数据
    if (remote.GetS1() != 2 || remote.GetS2() != 2 && gimbal.pitch_motor.enanble_flag == 0) {
        do {
            gimbal.pitch_motor.Enable(&hcan1);  // 使能电机
            osDelay(1);
        } while (gimbal.pitch_motor.enanble_flag == 0);
    }

    // 遥控控制与键鼠控制
    if (remote.GetS2() == 1 || remote.GetS2() == 3) {
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.yaw_output_speed, (int16_t)shoot.trig_output_);
        gimbal.pitch_motor.MITSend(&hcan1, 0.f, 0.f, 0.f, 0.f, gimbal.pitch_output_torque);
    }

    // 两种急停控制加自瞄测试
    if (remote.GetS1() != 2 && remote.GetS2() == 2) {
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.yaw_output_speed, (int16_t)shoot.trig_output_);
        gimbal.pitch_motor.MITSend(&hcan1, 0.f, 0.f, 0.f, 0.f, gimbal.pitch_output_torque);
    }

    if (remote.GetS1() == 2 && remote.GetS2() == 2) {
        DjiMotorSend(&hcan1, 0x200, 0, 0, 0, 0);
        // do {
        //  gimbal.pitch_motor.SaveZero(&hcan1);  // 初始化时保存当前位置为零点
        // } while ( gimbal.pitch_motor.zero_flag == 0);
        do {
            gimbal.pitch_motor.Disable(&hcan1);  // 失能电机
            osDelay(1);
        } while (gimbal.pitch_motor.enanble_flag == 1);
    }

    // 软件控制C620电调进入快速设置ID模式 (int16_t)shoot.trig_output_
    // CANC620IdSet();

    // 以下为测试代码专用
    //
}

/**
 * @brief       发送ID为0x700的CAN包,它会设置C620电调进入快速设置ID
 *   @arg       None
 * @retval      None
 * @note        None
 */
void CANC620IdSet()
{
    DjiMotorSend(&hcan1, CAN_MOTOR3508_SET_ID, 0000, 0000, 0000, 0000);
}