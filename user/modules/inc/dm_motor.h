/**
 *******************************************************************************
 * @file      : DM_Motor.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DM_MOTOR_H_
#define __DM_MOTOR_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
#define P_MIN -12.5   // 位置最小值
#define P_MAX 12.5    // 位置最大值
#define V_MIN -45     // 速度最小值
#define V_MAX 45      // 速度最大值
#define KP_MIN 0.0    // Kp最小值
#define KP_MAX 500.0  // Kp最大值
#define KD_MIN 0.0    // Kd最小值
#define KD_MAX 5.0    // Kd最大值
#define T_MIN -18     // 转矩最大值
#define T_MAX 18      // 转矩最小值

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum ErrorCode{
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
    void Update();
    void TXJudge();
    void Init(CtrlMode _mode,uint16_t _CAN_id, uint32_t _idx, CAN_HandleTypeDef* _phcan, uint8_t _init);
    void Enable(CAN_HandleTypeDef* _phcan);
    void Disable(CAN_HandleTypeDef* _phcan);
    void SaveZero(CAN_HandleTypeDef* _phcan);
    void DeleteError(CAN_HandleTypeDef* _phcan);
    void MITSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _pos, float _vel, float _KP, float _KD, float _torq);
    void PosSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _pos, float _vel);
    void SpeedSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _vel);
    const float GetAngle();
    const float GetSpeed();
    const float GetTorque();

   private:
    CtrlMode mode;
    ErrorCode errorcode;
    float angle, speed, torque;//单位分别为rad,rad/s,Nm
    uint8_t temperatureMOS;//表示驱动上 MOS 的平均温度，单位℃
    uint8_t temperatureRotor;//表示电机内部线圈的平均温度，单位℃
    uint8_t init_;
    CAN_TxHeaderTypeDef tx_conf;

};
/* Exported variables --------------------------------------------------------*/
extern DMMotor pitch_motor;
/* Exported function prototypes ----------------------------------------------*/
float RadToDeg(float _rad);
float DegToRad(float _deg);
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
