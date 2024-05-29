/**
 *******************************************************************************
 * @file      : gimbal.h
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
#ifndef __GIMBAL_H_
#define __GIMBAL_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "dji_motor.h"
#include "dm_motor.h"
#include "pid.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief The Gimbal class represents a gimbal system used for controlling the pitch and yaw positions.
 */
class Gimbal
{
   public:
    Pid yaw_angle;          /**< Array of Pid objects for angle control of pitch and yaw. */
    Pid yaw_speed;          /**< Array of Pid objects for speed control of pitch and yaw. */
    Pid pitch_angle;        /**< Pid object for angle control of pitch. */
    Pid pitch_speed;        /**< Pid object for torque control of pitch. */
    DjiMotor yaw_motor;     /**< Array of DjiMotor objects for controlling the motors of the gimbal. */
    DMMotor pitch_motor;    /**< DMMotor object for controlling the pitch motor. */
    float yaw_output_speed; /**< Array of output speeds for the motors. */
    float pitch_output_pos;
    float pitch_output_speed;
    float pitch_output_torque;
    float pitch_offset;  // 因为陀螺仪返回值与达妙电机的返回值不一样，而我们真正需要参考的是陀螺仪的返回值，所以需要一个偏移量来控制达妙电机抵消偏差
    float pitch_insreal;
    float pitch_dmreal;
    float pitch_set_real;
    float pitch_err;
    float yaw_modify;
    float pitch_modify;
    /**
     * @brief Controls the gimbal system.
     */
    void Control();

    /**
     * @brief Sets the pitch position of the gimbal.
     * @param _ang The desired pitch angle in degrees.
     */
    void SetPitchPosition(float _ang);

    /**
     * @brief Sets the yaw position of the gimbal.
     * @param _ang The desired yaw angle in degrees.
     */
    void SetYawPosition(float _ang);

    /**
     * @brief Initializes the PID controllers for angle and speed control.
     */
    void PidInit();

    /**
     * @brief Initializes the motors of the gimbal.
     */
    void MotorInit();

   private:
};
/* Exported variables --------------------------------------------------------*/
extern Gimbal gimbal;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __GIMBAL_H_ */
