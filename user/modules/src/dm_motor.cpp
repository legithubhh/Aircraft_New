/**
 *******************************************************************************
 * @file      : DMMotor.cpp
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
#include "dm_motor.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int FloatToUint(float x, float x_min, float x_max, int bits);
static float UintToFloat(int x_int, float x_min, float x_max, int bits);

void DMMotor::Update()
{
    errorcode = static_cast<ErrorCode>(pdji_motor_instance->rx_buff[0] >> 4);
    static int p_int, v_int, t_int;
    p_int = pdji_motor_instance->rx_buff[1] << 8 | pdji_motor_instance->rx_buff[2];
    v_int = pdji_motor_instance->rx_buff[3] << 4 | pdji_motor_instance->rx_buff[4] >> 4;
    t_int = (pdji_motor_instance->rx_buff[4] & 0xF) << 8 | pdji_motor_instance->rx_buff[5];
    angle = UintToFloat(p_int, P_MIN, P_MAX, 16);
    speed = UintToFloat(v_int, V_MIN, V_MAX, 12);
    torque = UintToFloat(t_int, T_MIN, T_MAX, 12);
    temperatureMOS = pdji_motor_instance->rx_buff[6];
    temperatureRotor = pdji_motor_instance->rx_buff[7];
}

/** 
 * @brief  上电自检完毕后需发送“使能”命令，电机LED由红变绿，之后才可以控制电机
 */
void DMMotor::Enable(CAN_HandleTypeDef* _phcan)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFC;
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/** 
 * @brief 失能电机
 */
void DMMotor::Disable(CAN_HandleTypeDef* _phcan)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFD;
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/** 
 * @brief “保存位置零点”
 */
void DMMotor::SaveZero(CAN_HandleTypeDef* _phcan)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFE;
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/** 
 * @brief 电机出现过热等错误时，发送“清除”命令可以清除错误
 */
void DMMotor::DeleteError(CAN_HandleTypeDef* _phcan)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFB;
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

const float DMMotor::GetAngle()
{
    return angle;
}

const float DMMotor::GetSpeed()
{
    return speed;
}

const float DMMotor::GetTorque()
{
    return torque;
}

void DMMotor::Init(CtrlMode _mode, uint16_t _CAN_id, uint32_t _idx, CAN_HandleTypeDef* _phcan, uint8_t _init)
{
    mode = _mode;
    CAN_id = _CAN_id;
    CanInitConf conf;
    init_ = _init;
    conf.hcan = _phcan;
    conf.rx_id = _idx;
    pdji_motor_instance = pCanRegister(&conf);
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 * @note:根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des
 *       即可实现匀速转动;kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出。KD不要给的很小，不然会出现震荡。
 * attention: 对位置进行控制时，kd 不能赋 0，否则会造成震荡甚至失控
 */
void DMMotor::MITSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _pos, float _vel,
                      float _KP, float _KD, float _torq)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = FloatToUint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = FloatToUint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = FloatToUint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = FloatToUint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = FloatToUint(_torq, T_MIN, T_MAX, 12);

    tx_data[0] = (pos_tmp >> 8);
    tx_data[1] = pos_tmp;
    tx_data[2] = (vel_tmp >> 4);
    tx_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    tx_data[4] = kp_tmp;
    tx_data[5] = (kd_tmp >> 4);
    tx_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    tx_data[7] = tor_tmp;

    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/**
 * Function_Name:DM位置级控制
 * Function_description:位置速度双环PID，PID通过达妙官方调参软件调整。ID为CAN_ID加上0x100的偏移
 * Param:_pos 16位 _vel
 * 创建时间:2023/12/13 15:18:52
 * Modify:2024/4/10 11:03 by Jason Li
 * @note:_pos 为控制的目标位置，_vel 是用来限定运动过程中的最大绝对速度值。
 */
void DMMotor::PosSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _pos, float _vel)
{
    TXJudge();
    tx_conf.DLC = 0x08;
    uint8_t tx_data[8];
    float pos, vel;
    pos = _pos;
    vel = _vel;
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;
    tx_data[0] = *pbuf;
    tx_data[1] = *(pbuf + 1);
    tx_data[2] = *(pbuf + 2);
    tx_data[3] = *(pbuf + 3);
    tx_data[4] = *vbuf;
    tx_data[5] = *(vbuf + 1);
    tx_data[6] = *(vbuf + 2);
    tx_data[7] = *(vbuf + 3);
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

/**
 * Function_Name:DM速度模式控制
 * Function_description:ID为CAN_ID加上0x200的偏移，PID通过达妙官方调参软件调整
 * Author:Zhuolin Yang
 * 创建时间:2023/12/13 14:35:51
 * Modify:2024/4/10 11:03 by Jason Li
 */
void DMMotor::SpeedSend(CAN_HandleTypeDef* _phcan, uint16_t _idx, float _vel)  // 速度控制可以直接输出
{
    TXJudge();
    tx_conf.DLC = 0x04;
    uint8_t tx_data[4];
    uint8_t* vbuf;
    float vel;
    vel = _vel;
    vbuf = (uint8_t*)&vel;  // Vbuf为指向地址的指针
    tx_data[0] = *vbuf;
    tx_data[1] = *(vbuf + 1);
    tx_data[2] = *(vbuf + 2);
    tx_data[3] = *(vbuf + 3);
    if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            while (1) {
            }
        }
    }
}

void DMMotor::TXJudge()
{
    if (mode == MIT) {
        tx_conf.StdId = CAN_id;
    } else if (mode == POS) {
        tx_conf.StdId = 0x100 + CAN_id;
    } else if (mode == SPEED) {
        tx_conf.StdId = 0x200 + CAN_id;
    }
    tx_conf.IDE = CAN_ID_STD;
    tx_conf.RTR = CAN_RTR_DATA;
}

float RadToDeg(float _rad)//弧度转角度
{
    return _rad * 180.f / 3.1415926f;
}

float DegToRad(float _deg)//角度转弧度
{
    return _deg * 3.1415926f / 180.f;
}

static int FloatToUint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float UintToFloat(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}