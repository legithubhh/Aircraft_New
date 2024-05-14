/**
 *******************************************************************************
 * @file      : remote_keyboard.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_KEYBOARD_H_
#define __REMOTE_KEYBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
    CAN_PITCH_POS_DM_ID = 0x154,

    /**
     * 分别对应三个3508和两个2006报文ID
     */
    CAN_FRICTION1_3508_ID = 0x201,
    CAN_FRICTION2_3508_ID = 0x202,
    CAN_YAW_3508_ID = 0x203,
    CAN_PITCH_4310_ID = 0x32,
    CAN_TRIGGER_2006_ID = 0x205,

    /**
     * 3508电调ID软件快速设置发送ID
     */
    CAN_MOTOR3508_SET_ID = 0x700

} CanMsgMotorId;

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

} PidSwitchMode;

typedef struct
{
    uint8_t fric_flag;           // 摩擦轮开关控制
    uint8_t trig_flag;           // 拨弹盘开关控制
    uint8_t auto_flag;           // 自瞄开关控制
    uint8_t last_auto_flag;      // 上一次自瞄开关状态
    uint8_t return_trig_count;      // 拨弹盘退弹总次数统计
    uint8_t last_return_trig_count; // 上一次退弹总次数统计
    uint8_t trig_block_flag;             // 卡弹检测
    uint8_t last_key_press[16];  // 上一次按键状态
    uint8_t last_s1;             // 上一次左拨杆状态
} Flag;

/* Exported variables --------------------------------------------------------*/
extern Flag flag;
/* Exported function prototypes ----------------------------------------------*/
void ModeTask();

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_KEYBOARD_H_ */
