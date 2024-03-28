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
    angle_[0].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 100, 0, 0, 0, 0);
    speed_[0].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 3000, 0, 0, 0, 0);
    angle_[1].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 0, 0, 0, 0, 0);
    speed_[1].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT | PID_DERIVATIVE_ON_MEASUREMENT, 0, 0, 0, 0, 0);
}

/**
 * @brief Initializes the motor for the gimbal.
 */
void Gimbal::MotorInit()
{
    motor_[0].Init(0x204, &hcan1, ABSOLUTE_FLAG);
    motor_[1].Init(0x205, &hcan1, ABSOLUTE_FLAG);
    motor_[0].pdji_motor_instance->pCanCallBack = PitchMotorCallback;
    motor_[1].pdji_motor_instance->pCanCallBack = YawMotorCallback;
}

/**
 * @brief Controls the gimbal.
 */
void Gimbal::Control()
{
    angle_[0].SetMeasure(INS.Roll);//根据安装位置，数据Roll对应实际Pitch轴
    angle_[1].SetMeasure(INS.YawTotalAngle);

    speed_[0].SetRef(angle_[0].Calculate());
    speed_[1].SetRef(angle_[1].Calculate());

    speed_[0].SetMeasure(INS.Gyro[ROLL_AXIS]);//速度测量值为陀螺仪角速度，而不是电机速度
    speed_[1].SetMeasure(INS.Gyro[YAW_AXIS]);

    output_speed_[0] = speed_[0].Calculate();
    output_speed_[1] = speed_[1].Calculate();
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
    angle_[0].SetRef(Math::AbsLimit(set_pitch_, 25.0f));
}

/**
 * @brief Sets the yaw position of the gimbal.
 *
 * @param _ang The desired yaw angle in degrees.
 */
void Gimbal::SetYawPosition(float _ang)
{
    angle_[1].SetRef(Math::AbsLimit(set_yaw_, 30.0f));
}

/**
 * @brief Callback function for the pitch motor.
 */
static void PitchMotorCallback()
{
    gimbal.motor_[0].Update();
}

/**
 * @brief Callback function for the yaw motor.
 */
static void YawMotorCallback()
{
    gimbal.motor_[1].Update();
}