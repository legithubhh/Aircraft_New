/**
 *******************************************************************************
 * @file      : referee.cpp
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
#include "referee.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Referee referee;
RefKeyMouse ref_keymouse;
/* Private function prototypes -----------------------------------------------*/
/* Function definitions ------------------------------------------------------*/

void Referee::Update(uint8_t *_p_data)
{
    uint16_t judge_length;  // 统计一帧数据长度
    uint16_t data_length;   // 统计有效数据长度
    //	CmdID = 0; //数据命令码解析
    // 空数据包，则不作任何处理
    if (_p_data == NULL)
        return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&FrameHeader, _p_data, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
    if (_p_data[SOF] == JUDGE_FRAME_HEADER) {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(_p_data, LEN_HEADER) == TRUE) {
            data_length = _p_data[DATA_LENGTH];
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = data_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(_p_data, judge_length) == TRUE) {
                // 2个8位拼成16位int
                CmdID = (_p_data[6] << 8 | _p_data[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (CmdID) {
                    case ID_game_state:  // 0x0001
                        memcpy(&game_state_, (_p_data + DATA), LEN_game_state);
                        break;

                    case ID_game_result:  // 0x0002
                        memcpy(&game_result_, (_p_data + DATA), LEN_game_result);
                        break;

                    case ID_game_robot_survivors:  // 0x0003
                        memcpy(&game_robot_HP_, (_p_data + DATA), LEN_game_robot_HP);
                        break;

                    case ID_event_data:  // 0x0101
                        memcpy(&event_data_, (_p_data + DATA), LEN_event_data);
                        break;

                    case ID_supply_projectile_action:  // 0x0102
                        memcpy(&supply_projectile_action_, (_p_data + DATA), LEN_supply_projectile_action);
                        break;

                    case ID_game_robot_state:  // 0x0201
                        memcpy(&game_robot_state_, (_p_data + DATA), LEN_game_robot_state);
                        break;

                    case ID_power_heat_data:  // 0x0202
                        memcpy(&power_heat_data_, (_p_data + DATA), LEN_power_heat_data);
                        break;

                    case ID_game_robot_pos:  // 0x0203
                        memcpy(&game_robot_pos_, (_p_data + DATA), LEN_game_robot_pos);
                        break;

                    case ID_buff_musk:  // 0x0204
                        memcpy(&buff_musk_, (_p_data + DATA), LEN_buff_musk);
                        break;

                    case ID_aerial_robot_support_data:  // 0x0205
                        memcpy(&aerial_robot_support_data_, (_p_data + DATA), LEN_aerial_robot_energy);
                        break;

                    case ID_robot_hurt:  // 0x0206
                        memcpy(&robot_hurt_, (_p_data + DATA), LEN_robot_hurt);
                        break;

                    case ID_shoot_data:  // 0x0207
                        memcpy(&shoot_data_, (_p_data + DATA), LEN_shoot_data);
                        // JUDGE_ShootNumCount();//发弹量统计
                        break;
                }
            }
        }
        // // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        // if (*(_p_data + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5) {
        //     // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
        //     Update(_p_data + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
        // }
    }
}

static void RefereeCallback()
{
    referee.Update(referee.preferee_instance_->rx_buffer);
}

void Referee::Init(UART_HandleTypeDef *_phuart)
{
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 256;
    conf.callback_function = RefereeCallback;
    referee.preferee_instance_ = pUartRegister(&conf);
    return;
}

/**
 * @brief      图传链路键鼠解算
 * @param       *_p_data:
 *   @arg       None
 * @retval      None
 * @note        None
 */
void RefKeyMouse::Update(uint8_t *_p_data)
{
    uint16_t judge_length;  // 统计一帧数据长度
    uint16_t data_length;   // 统计有效数据长度
    //	CmdID = 0; //数据命令码解析
    // 空数据包，则不作任何处理
    if (_p_data == NULL)
        return;

    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&FrameHeader, _p_data, LEN_HEADER);

    // 判断帧头数据(0)是否为0xA5
    if (_p_data[SOF] == JUDGE_FRAME_HEADER) {
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(_p_data, LEN_HEADER) == TRUE) {
            data_length = _p_data[DATA_LENGTH];
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = data_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(_p_data, judge_length) == TRUE) {
                // 2个8位拼成16位int
                CmdID = (_p_data[6] << 8 | _p_data[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (CmdID) {
                    case ID_command_data:  // 0x0304
                        memcpy(&comma_data, (_p_data + DATA), LEN_command_data);
                        break;
                }
            }
        }
        // // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        // if (*(_p_data + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5) {
        //     // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
        //     Update(_p_data + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
        // }
    }
    KeyProcessUI();
}

void RefKeyMouse::KeyProcessUI()  // 图传链路键鼠解算代码
{
    for (short i = 0; i < 16; i++) {
        if (ref_keymouse.comma_data.keyboard_value & (0x01 << i)) {
            referee_key_press[i] = 1;
        } else {
            referee_key_press[i] = 0;
        }
    }
}

static void ClientUICallback()
{
    ref_keymouse.Update(ref_keymouse.pkeymouse_instance_->rx_buffer);
}

void RefKeyMouse::Init(UART_HandleTypeDef *_phuart)
{
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 256;
    conf.callback_function = ClientUICallback;
    ref_keymouse.pkeymouse_instance_ = pUartRegister(&conf);
    return;
}

int16_t GetRefMouseX()
{
    return ref_keymouse.comma_data.mouse_x;
}

int16_t GetRefMouseY()
{
    return ref_keymouse.comma_data.mouse_y;
}