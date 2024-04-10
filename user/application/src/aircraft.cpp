/**
 *******************************************************************************
 * @file      : infantry.cpp
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
#include "aircraft.h"

#include "bsp_dwt.h"
#include "gimbal.h"
#include "motor_pidmodify.h"
#include "referee.h"
#include "remote.h"
#include "remote_keyboard.h"
#include "shoot.h"
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
    /*电机控制，集中发送CAN信号，发送给电机的信号必须为整形*/
    // CAN1总线0X1FF对应电机ID，ID号1为拨弹盘2006电机
    // CAN1总线0X200对应电机ID，ID号1-4分别为摩擦轮3508电机1，摩擦轮3508电机2，Yaw轴3508电机，Pitch轴2006电机

    if (remote.GetS1() == 1 && remote.GetS2() == 1) {
        DjiMotorSend(&hcan1, 0x1FF, 0, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.output_speed_[1], (int16_t)gimbal.output_speed_[0]);
    }

    if (remote.GetS1() != 1 && remote.GetS2() == 1) {
        DjiMotorSend(&hcan1, 0x1FF, (int16_t)shoot.trig_output_, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.output_speed_[1], (int16_t)gimbal.output_speed_[0]);
    }

    if (remote.GetS1() == 1 && remote.GetS2() == 3) {
        DjiMotorSend(&hcan1, 0x1FF, 0, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.output_speed_[1], (int16_t)gimbal.output_speed_[0]);
    }

    if (remote.GetS1() != 1 && remote.GetS2() == 3) {
        DjiMotorSend(&hcan1, 0x1FF, (int16_t)shoot.trig_output_, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.output_speed_[1], (int16_t)gimbal.output_speed_[0]);
    }

    if (remote.GetS1() != 2 && remote.GetS2() == 2) {
        DjiMotorSend(&hcan1, 0x1FF, (int16_t)shoot.trig_output_, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)gimbal.output_speed_[1], (int16_t)gimbal.output_speed_[0]);
    }

    if (remote.GetS1() == 2 && remote.GetS2() == 2) {
        DjiMotorSend(&hcan1, 0x1FF, 0, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x200, 0, 0, 0, 0);
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