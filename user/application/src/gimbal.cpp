/**
 *******************************************************************************
 * @file      : gimbal.cpp
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
#include "gimbal.h"

#include "bsp_dwt.h"
#include "ins.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Gimbal gimbal;
/* Private function prototypes -----------------------------------------------*/

static void PitchMotorCallback();
static void YawMotorCallback();

/**
 * @brief Initializes the Gimbal.
 */
void Gimbal::PidInit()
{
    yaw_angle.Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 20.f, 0, 0, 0, 0);
    yaw_speed.Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 0, 0, 0, 0, 0);
    pitch_angle.Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER, 2.f, 0, 0, 0, 10);
    pitch_speed.Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER, 0.5f, 0, 0, 0, 20);
}

/**
 * @brief Initializes the motor for the gimbal.
 */
void Gimbal::MotorInit()
{
    yaw_motor.Init(0x203, &hcan1, ABSOLUTE_FLAG);
    yaw_motor.pdji_motor_instance->pCanCallBack = YawMotorCallback;
    pitch_motor.Init(MIT, 0x54, 0x32, &hcan1);
    pitch_motor.pdji_motor_instance->pCanCallBack = PitchMotorCallback;
    do {
        pitch_motor.Enable(&hcan1);  // 使能电机
    } while (pitch_motor.enanble_flag == 0);
    // DWT_Delay(1.f);
    // do {
    //     pitch_motor.SaveZero(&hcan1);  // 初始化时保存当前位置为零点
    // } while (pitch_motor.zero_flag == 0);
}

/**
 * @brief Controls the gimbal.  pitch_angle.Calculate()
 */
void Gimbal::Control()
{
    yaw_angle.SetMeasure(INS.YawTotalAngle);
    yaw_speed.SetRef(yaw_angle.Calculate());
    yaw_speed.SetMeasure(INS.Gyro[YAW_AXIS]);
    yaw_output_speed = yaw_speed.Calculate();

    pitch_angle.SetMeasure(INS.Roll);
    pitch_speed.SetRef(pitch_angle.Calculate());
    pitch_speed.SetMeasure(INS.Gyro[ROLL_AXIS]);
    pitch_output_torque = -pitch_speed.Calculate();  // 可以考虑加前馈，目前不需要；
}

/**
 * @brief Set the pitch position of the gimbal.
 *
 * This function sets the desired pitch position for the gimbal.
 *
 * @param position The desired pitch position in degrees.
 */
void Gimbal::SetPitchPosition(float _ang)
{
    pitch_angle.SetRef(_ang);
}

/**
 * @brief Sets the yaw position of the gimbal.
 *
 * @param _ang The desired yaw angle in degrees.
 */
void Gimbal::SetYawPosition(float _ang)
{
    yaw_angle.SetRef(_ang);
}
/**
 * @brief Callback function for the pitch motor.
 */
static void PitchMotorCallback()
{
    gimbal.pitch_motor.Update();  // 更新电机数据，包括角度、速度、电流等，以及错误码等信息，以便后续处理，如清除错误等操作
}

/**
 * @brief Callback function for the yaw motor.
 */
static void YawMotorCallback()
{
    gimbal.yaw_motor.Update();
}