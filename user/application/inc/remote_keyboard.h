/**
 *******************************************************************************
 * @file      : remote_keyboard.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_KEYBOARD_H_
#define __REMOTE_KEYBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * 创建电机CAN通信ID枚举类型，方便辨别，由使用情况具体定义名称
 */
typedef enum {
    /**
     * 0x200对应CAN1发送控制两个摩擦轮3508，一个Yaw轴3508和一个Pitch轴2006，0x1FF对应CAN1发送控制拨弹盘2006
     */
    CAN_GIMBAL_SEND_ID = 0x200,
    CAN_GIMBAL_SEND_ELSE_ID = 0x1FF,

    /**
     * 分别对应三个3508和两个2006报文ID
     */
    CAN_FRICTION1_3508_ID = 0x201,
    CAN_FRICTION2_3508_ID = 0x202,

    CAN_TRIGGER_2006_ID = 0x205,
    CAN_PITCH_2006_ID = 0x204,

    CAN_YAW_3508_ID = 0x203,

    /**
     * 3508电调ID软件快速设置发送ID
     */
    CAN_MOTOR3508_SET_ID = 0x700

} CanMsgMotorId;

/**
 * 针对遥控器，键鼠与自瞄三种控制模式设置三套相适应枚举类型PID参数，以下为提供判断条件的对应的标志编码
 */
typedef enum {
    /**
     * 遥控模式与急停模式
     */
    remote_pid_flag = 0,

    /**
     * 键鼠模式
     */
    keymouse_pid_flag = 1,

    /**
     * 自瞄模式
     */
    autoaim_pid_flag = 2,

} PidsetModeID;

/**
 * 同一电机在执行不同任务时可能需要用到不同PID参数，进行PID参数的切换，以下为提供判断条件的对应的标志编码
 */
typedef enum {
    /**
     * 基础模式
     */
    base_pid = 0,

    /**
     * Pitch模式1
     */
    pitch1_pid = 1,

    /**
     * Pitch模式2
     */
    pitch2_pid = 2,

    /**
     * Pitch模式3
     */
    pitch3_pid = 3,

    /**
     * Pitch模式4
     */
    pitch4_pid = 4,

    /**
     * Pitch模式5
     */
    pitch5_pid = 5,

    /**
     * Yaw模式1
     */

    yaw1_pid = 6,

    /**
     * Yaw模式2
     */
    yaw2_pid = 7

} PidSwitchMode;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void ModeTask();

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_KEYBOARD_H_ */
