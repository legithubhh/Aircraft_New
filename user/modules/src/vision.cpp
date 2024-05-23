/**
 *******************************************************************************
 * @file      : vision.cpp
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
#include "vision.h"

#include "board_comm.h"
#include "bsp_dwt.h"
#include "crc.h"
#include "ins.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
#define PI 3.1415927f
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables  ---------------------------------------------------------*/
Vision vision;
/* External variables --------------------------------------------------------*/
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
/* Private function prototypes -----------------------------------------------*/

void Vision::AngTrans()
{
    target_pitch_ = Math::RadToDeg((float)rece_.pitch);
    target_yaw_ = Math::RadToDeg((float)rece_.yaw);
}

void Vision::Receive()
{
    if (UserRxBufferFS[0] == 0xA5) {
        if (Verify_CRC16_Check_Sum(UserRxBufferFS, sizeof(rece_packet))) {
            memcpy(&rece_, &UserRxBufferFS, sizeof(rece_packet));
        }
    }
}

void Vision::Encode()
{
    send_.header = 0x5A;
    if (board_comm.GetRobotID() > 100) {
        send_.detect_color = 0;
    } else {
        send_.detect_color = 1;
    }
    send_.reserved = 0;
    send_.mode = 1;
    send_.pitch = Math::DegToRad(INS.Roll);  // 实际安装后Roll为pitch，陀螺仪向左向上为正，视觉向左向上为正
    send_.yaw = Math::DegToRad(INS.Yaw);
    send_.bullet_speed = 27.f;  //referee.shoot_data_.bullet_speed
    send_.checksum = Get_CRC16_Check_Sum((uint8_t *)&send_, sizeof(send_packet) - 2, 0xffff);
}

void Vision::Send()
{
    Encode();
    CDC_Transmit_FS((uint8_t *)&send_, sizeof(send_packet));
}

void Vision::Ctrl()
{
    Receive();
    if (rece_.tracking == 1) {
        AngTrans();
    }
}

float GetTargetPitch()
{
    return vision.target_pitch_;
}

float GetTargetYaw()
{
    return vision.target_yaw_;
}