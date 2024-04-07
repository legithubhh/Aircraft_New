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
void PidSetInitial();
void PidAdjust();
void PitchPidDemo1();
void PitchPidDemo2();
void PitchPidDemo3();
void PitchPidDemo4();
void PitchPidDemo5();
void YawPidDemo1();
void YawPidDemo2();

void RemoteAimingTargetSet();
void KeymouseAimingTargetSet();
void AutoAimingTargetSet();
void GimbalStop1TargetSet();
void GimbalStop2TargetSet();
void MotorStart();
#endif

#endif /* __MOTOR_PIDMODIFY_H_ */
