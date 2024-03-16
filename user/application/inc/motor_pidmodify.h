/**
 *******************************************************************************
 * @file      : motor_pidmodify.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PIDMODIFY_H_
#define __MOTOR_PIDMODIFY_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * 创建摩擦轮电机与双轴电机的目标值变量，方便修改。
 */
typedef struct
{
    float pitch_target;
    float yaw_target;
    float friction_wheel_target;
    float turn_magazine_target;
} GimbalTargetSylloge;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void PidSetRemote();
void RemotePitchPidDemo1();
void RemotePitchPidDemo2();
void RemotePitchPidDemo3();
void RemotePitchPidDemo4();
void RemotePitchPidDemo5();
void RemoteYawPidDemo1();
void RemoteYawPidDemo2();
void PidSetKeymouse();
void PidSetAutoaim();
void KeymPitchPidDemo1();
void AutoPitchPidDemo1();

void RemoteControlMode();
void KeymouseControlMode();
void AutoControlMode();
void GimbalStop1ControlMode();
void GimbalStop2ControlMode();
#endif

#endif /* __MOTOR_PIDMODIFY_H_ */
