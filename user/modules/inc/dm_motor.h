/**
 *******************************************************************************
 * @file      : DM_Motor.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      RM2024      Jason Li        Victory
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DM_MOTOR_H_
#define __DM_MOTOR_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
#define P_MIN -12.5   // 位置最小值
#define P_MAX 12.5    // 位置最大值
#define V_MIN -630     // 速度最小值
#define V_MAX 630      // 速度最大值
#define KP_MIN 0.0    // Kp最小值
#define KP_MAX 500.0  // Kp最大值
#define KD_MIN 0.0    // Kd最小值
#define KD_MAX 5.0    // Kd最大值
#define T_MIN -20     // 转矩最大值
#define T_MAX  20     // 转矩最小值

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum ErrorCode{
        Disable = 0x0,//失能；
        Enable = 0x1,//使能；
        OverVoltage = 0x8,//超压；
        UnderVoltage = 0x9,//欠压；
        OverCurrent = 0xA,//过电流；
        MOS_OverTemperature = 0xB,//MOS 过温
        MotorCoil_OverTemperature = 0x0C,//电机线圈过温；
        LostCommunication = 0x0D,//通讯丢失；
        OverLoad = 0x0E//过载；
    }ErrorCode;

typedef enum CtrlMode{
        MIT = 0,//MIT模式
        POS = 1,//位置模式
        SPEED = 2,//速度模式
    }CtrlMode;

class DMMotor
{
   public:
    uint16_t CAN_id;
    CanInstance *pdji_motor_instance;
    uint8_t enanble_flag;//0为未使能，1为使能
    uint8_t zero_flag;//0为未保存零点，1为保存零点
    void Update();
    void TXJudge();
    void Init(CtrlMode _mode, uint16_t _CAN_id, uint32_t _master_id, CAN_HandleTypeDef* _phcan);
    void Enable(CAN_HandleTypeDef* _phcan);
    void Disable(CAN_HandleTypeDef* _phcan);
    void SaveZero(CAN_HandleTypeDef* _phcan);
    void DeleteError(CAN_HandleTypeDef* _phcan);
    void MITSend(CAN_HandleTypeDef* _phcan, float _pos, float _vel, float _KP, float _KD, float _torq);
    void PosSend(CAN_HandleTypeDef* _phcan, float _pos, float _vel);
    void SpeedSend(CAN_HandleTypeDef* _phcan, float _vel);
    void SetAngle(float _angle);
    void SetSpeed(float _speed);
    void SetTorque(float _torque);
    const ErrorCode GetErrorCode();
    const float GetAngleTarget();
    const float GetSpeedTarget();
    const float GetTorqueTarget();
    const float GetAngle();
    const float GetSpeed();
    const float GetTorque();

   private:
    CtrlMode mode;
    ErrorCode errorcode;
    float set_angle, set_speed, set_torque;//发送给电机的目标值，单位需为rad,rad/s,Nm
    float angle, speed, torque;//单位分别为rad,rad/s,Nm
    uint8_t temperatureMOS;//表示驱动上 MOS 的平均温度，单位℃
    uint8_t temperatureRotor;//表示电机内部线圈的平均温度，单位℃
    CAN_TxHeaderTypeDef tx_conf;

};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DM_MOTOR_H_ */
