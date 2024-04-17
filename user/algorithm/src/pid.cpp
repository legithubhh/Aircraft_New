/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "pid.h"

#include "bsp_dwt.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Calculates the output of the PID controller.
 *
 * @return The calculated output value.
 */
float Pid::Calculate()
{
    if (improve_ & PID_ERROR_HANDLE) {
        ErrorHandle();
    }

    dt = DWT_GetDeltaT(&DWT_CNT);

    err_ = ref_ - measure_;

    if (abs(err_) > dead_band_) {
        p_out_ = kp_ * err_;
        i_term_ = ki_ * err_ * dt;
        d_out_ = kd_ * (err_ - last_err_) / dt;

        if (improve_ & PID_TRAPEZOID_INTEGRAL) {
            TrapezoidIntergral();
        }
        if (improve_ & PID_CHANGING_INTEGRATION_RATE) {
            ChangingIntegratioRate();
        }
        if (improve_ & PID_DERIVATIVE_ON_MEASUREMENT) {
            DerivativeOnMeasurement();
        }
        if (improve_ & PID_DERIVATIVE_FILTER) {
            DerivativeFilter();
        }
        if (improve_ & PID_INTEGRAL_LIMIT) {
            IntegralLimit();
        }

        i_out_ += i_term_;
        output_ = p_out_ + i_out_ + d_out_;

        if (improve_ & PID_OUTPUT_FILTER) {
            OutputFilter();
        }

        OutputLimit();
    } else {
        output_ = 0.0f;
        i_term_ = 0.0f;
    }

    last_measure_ = measure_;
    last_err_ = err_;
    last_output_ = output_;
    last_d_out_ = d_out_;
    last_i_term_ = i_term_;

    return output_;
}

/**
 * @brief Performs trapezoidal integration for the PID controller.梯形积分
 *
 * This function calculates the integral term of the PID controller using the trapezoidal integration method.
 *
 * @note This function should be called at each iteration of the control loop.
 */
void Pid::TrapezoidIntergral()
{
    i_term_ = ki_ * ((err_ + last_err_) / 2.0f) * dt;
}

/**
 * @brief Changes the integration rate for the PID controller.改变积分速率
 */
void Pid::ChangingIntegratioRate()
{
    if (err_ * i_out_ > 0) {
        if (abs(err_) <= coef_b_) {
            return;
        }
        if (abs(err_) <= (coef_a_ + coef_b_)) {
            i_term_ *= (coef_a_ - abs(err_) + coef_b_) / coef_a_;
        } else {
            i_term_ = 0;
        }
    }
}

/**
 * @brief Limits the integral term of the PID controller.积分限幅
 */
void Pid::IntegralLimit()
{
    static float temp_output, temp_i_out;
    temp_i_out = i_out_ + i_term_;
    temp_output = p_out_ + i_out_ + d_out_;
    if (abs(temp_output) > max_out_) {
        if (err_ * i_out_ > 0) {
            i_term_ = 0;
        }
    }

    if (temp_i_out > integral_limit_) {
        i_term_ = 0;
        i_out_ = integral_limit_;
    }
    if (temp_i_out < -integral_limit_) {
        i_term_ = 0;
        i_out_ = -integral_limit_;
    }
}

/**
 * @brief Applies the derivative on measurement control strategy in the PID algorithm.微分环节
 *
 * This function calculates the derivative term of the PID controller based on the rate of change of the process variable.
 * The derivative term is subtracted from the control output to counteract sudden changes in the process variable.
 *
 * @note This function assumes that the process variable and setpoint have been properly initialized.
 */
void Pid::DerivativeOnMeasurement()
{
    d_out_ = kd_ * (last_measure_ - measure_) / dt;
}

/**
 * @brief Applies a derivative filter to the PID controller.微分滤波
 *
 * This function applies a derivative filter to the PID controller.
 * It is used to calculate the derivative term of the PID controller
 * based on the rate of change of the error signal.
 */
void Pid::DerivativeFilter()
{
    d_out_ = d_out_ * dt / (derivative_lpf_rc_ + dt) + last_d_out_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt);
}

/**
 * @brief Applies the output filter to the PID controller's output.输出滤波
 *在这个表达式中，d_out_（当前输出）和 last_d_out_（上一个输出）的权重由 derivative_lpf_rc_ 和 dt 决定。这就是一阶低通滤波器的核心：通过加权平均当前和过去的值来平滑输出。
 *时间常数 derivative_lpf_rc_ 控制了滤波器的平滑程度。较大的 derivative_lpf_rc_ 值意味着对过去值的权重更大，因此滤波效果更显著，输出更加平滑；而较小的值则更接近于当前值，滤波效果较弱。
 *通过这种方式，一阶低通滤波器能够在保留信号基本特征的同时，减少高频噪声的影响，提高信号的信噪比，使其更适合用于控制系统或数据分析中。
 * @return void
 */
void Pid::OutputFilter()
{
    output_ = output_ * dt / (output_lpf_rc_ + dt) + last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt);
}

/**
 * @brief Sets the output limit for the PID controller.输出限幅
 */
void Pid::OutputLimit()
{
    if (output_ > max_out_) {
        output_ = max_out_;
    }
    if (output_ < -max_out_) {
        output_ = -max_out_;
    }
}

/**
 * @brief Handles the error in the PID controller.
 */
void Pid::ErrorHandle()
{
    if (fabsf(output_) < max_out_ * 0.001f || fabsf(ref_) < 0.0001f)
        return;
    if (fabsf(ref_ - measure_) / fabsf(ref_) > 0.95f) {
        error_handle.ERRORCount++;
    } else {
        error_handle.ERRORCount = 0;
    }

    if (error_handle.ERRORCount > 500) {
        error_handle.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}