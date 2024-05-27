/**
 *******************************************************************************
 * @file      : shoot.cpp
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
#include "shoot.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Shoot shoot;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief This function is a callback for the left friction motor.
 *
 * This function is called when an event related to the left friction motor occurs.
 * It is responsible for handling the event and performing the necessary actions.
 *
 * @return void
 */
static void LeftFricMotorCallback();

/**
 * @brief This function is a callback for the right friction motor.
 *
 * This function is called when a specific event occurs related to the right friction motor.
 * It is responsible for handling the event and performing the necessary actions.
 *
 * @return void
 */
static void RightFricMotorCallback();

/**
 * @brief This function is a callback for triggering the motor.
 */
static void TriggerMotorCallback();

void Shoot::PidInit()
{
    fric_speed_[0].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT, 20.f, 0.0f, 0.0f, 0.0f, 0.0f);//空转i_out为20
    fric_speed_[1].Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT, 20.f, 0.0f, 0.0f, 0.0f, 0.0f);
    trigger_pos_.Inprovement(PID_IMPROVE_NONE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    trigger_speed_.Inprovement(PID_TRAPEZOID_INTEGRAL | PID_INTEGRAL_LIMIT, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f);//空转i_out为400
}

void Shoot::MotorInit()
{
    fric_motor_[0].Init(0x201, &hcan1, ABSOLUTE_FLAG);//左摩擦轮
    fric_motor_[1].Init(0x202, &hcan1, ABSOLUTE_FLAG);//右摩擦轮
    trigger_motor_.Init(0x204, &hcan1, RELATIVE_FLAG);
    fric_motor_[0].pdji_motor_instance->pCanCallBack = LeftFricMotorCallback;
    fric_motor_[1].pdji_motor_instance->pCanCallBack = RightFricMotorCallback;
    trigger_motor_.pdji_motor_instance->pCanCallBack = TriggerMotorCallback;
}

void Shoot::Control()
{
    FricCalc();
    TrigCalc();
}

void Shoot::FricCalc()
{
    for (int i = 0; i < 2; i++) {
        fric_speed_[i].SetMeasure(fric_motor_[i].GetSpd());
        fric_output_[i] = fric_speed_[i].Calculate();
    }
}

void Shoot::TrigCalc()
{
    if (shoot_flag_ == ANGLE_FLAG) {
        trigger_pos_.SetMeasure(trigger_motor_.GetAngle());
        trigger_speed_.SetRef(trigger_pos_.Calculate());
    }

    trigger_speed_.SetMeasure(trigger_motor_.GetSpd());
    trig_output_ = trigger_speed_.Calculate();//根据实际情况调整正负号
}

void Shoot::SetFricSpeed(float _speed)
{
    fric_speed_[0].SetRef(-_speed);
    fric_speed_[1].SetRef(_speed);
}

void Shoot::SetTriggerPos(float _pos)
{
    trigger_pos_.SetRef(trigger_motor_.GetAngle() + _pos);
}

void Shoot::SetTriggerSpeed(float _speed)
{
    trigger_speed_.SetRef(_speed);
}

void Shoot::SetFlag(uint16_t _flag)
{
    shoot_flag_ = _flag;
}

static void LeftFricMotorCallback()
{
    shoot.fric_motor_[0].Update();
}

static void RightFricMotorCallback()
{
    shoot.fric_motor_[1].Update();
}

static void TriggerMotorCallback()
{
    shoot.trigger_motor_.Update();
}