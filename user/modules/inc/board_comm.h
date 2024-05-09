/**
 *******************************************************************************
 * @file      : board_comm.h
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
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "referee.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    bool s;
} send_pack;

typedef struct {
    bool r;
} rece_pack;

#pragma pack()

class BoardComm
{
   public:
    void Send();
    void Receive();
    void Init(CAN_HandleTypeDef *_phcan, uint16_t _id);
    uint8_t GetRobotID() { return referee.game_robot_state_.robot_id; }

   private:
    CanInstance *p_instance_;
    send_pack send_;
    rece_pack rece_;
};

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __BOARD_COMM_H_ */
