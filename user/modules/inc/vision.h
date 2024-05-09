/**
 *******************************************************************************
 * @file      : vision.h
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
#ifndef __VISION_H_
#define __VISION_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "user_lib.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
// 设置参数


struct rece_packet
{
  uint8_t header = 0xA5;
  bool tracking: 1;
  uint8_t can_shoot:1;
  uint8_t reserved:6;
  float pitch;
  float yaw;
  uint16_t checksum = 0;
} __attribute__((packed));

struct send_packet
{
  uint8_t header = 0x5A;
  uint8_t detect_color: 1;  // 0-red 1-blue
  bool reset_tracker: 1;
  uint8_t mode:3;//1:aim_mode  3:s_buff_mode   4:b_buff_mode
  uint8_t reserved: 3;
  float pitch;
  float yaw;
  float bullet_speed;
  int8_t pitch_bias=0;
  int8_t yaw_bias=0;
  int8_t pre_time_bias=0;
  uint16_t checksum = 0;
} __attribute__((packed));

class Vision
{
 public:
  float target_pitch_, target_yaw_;
  void Ctrl();
  void Send();
  void SetReSetTracker(bool _data)
  {
	send_.reset_tracker = _data;
  };
  bool GetTrack()
  {
	return rece_.tracking;
  };
  bool GetShootSignal()
  {
	return rece_.can_shoot;
  };
  float GetTargetPitch()
  {
	return target_pitch_;
  };
  float GetTargetYaw()
  {
	return target_yaw_;
  };
  send_packet send_;
  rece_packet rece_;
 private:
  void Receive();
  void Encode();
  void AngTrans();
};

/* Exported variables --------------------------------------------------------*/
extern Vision vision;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __VISION_H_ */
