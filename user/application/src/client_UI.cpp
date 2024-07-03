/**
 *******************************************************************************
 * @file      : client_UI.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      RM2024      Jason Li        Victory
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, University of Science and Technology Beijing.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "client_UI.h"

#include "crc.h"
#include "ins.h"
#include "remote_keyboard.h"
#include "string.h"
#include "gimbal.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char UI_Seq;  // 包的序号
uint16_t Robot_ID;
uint16_t Cilent_ID;

String_Data CH_Cap, CH_Shoot, CH_AIM, CH_user, CH_XTL_State;
Graph_Data G_aim[16], G1, G2;
Float_Data F1;
char G_aim_mark[8][4] = {"601", "602", "603", "604", "605", "606", "607", "608"};

UI_Graph1_t UI_Graph1[2];
UI_Graph2_t UI_Graph2[2];
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String[5];
UI_Delete_t UI_Delete;

char XTL[2] = "G";
char shotsingle[7] = "single";
char shotfusillade[10] = "fusillade";
char cap_arr[4] = "Cap";
char aim_arr[4] = "Aim";
char farshoot_arr[4] = "Dtc";
char user[7] = "Reborn";
char auto_aim[8] = "AutoAim";
char da_fu[8] = "BIGBUFF";
char xiao_fu[8] = "MINBUFF";

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ID_Judge(void);
void UI_init(void);
void UI_PushUp_Graphs(uint8_t cnt, void *Graphs);
void UI_PushUp_String(UI_String_t *String);
void UI_PushUp_Delete(UI_Delete_t *Delete);
void Line_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void Circle_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 Graph_Radius);
void Rectangle_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y);
void Float_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_Digit, u32 Graph_Width, u32 Start_x, u32 Start_y, float FloatData);
void Char_Draw(String_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_Digit, u32 Graph_Width, u32 Start_x, u32 Start_y, char *Char_Data);
void Arc_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_StartAngle, u32 Graph_EndAngle, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 x_Length, u32 y_Length);

void UITask()
{
    // 为了不过多发送，占用资源，这里UI_PushUp_Counter判断都是质数
    static uint16_t UI_PushUp_Counter = 80;
    UI_PushUp_Counter++;
    // ID判断
    if (UI_PushUp_Counter > 500) {
        ID_Judge();
        vTaskDelay(15);  // 发送一次数据，立马延时，防止堵塞。
        UI_PushUp_Counter = 90;
    }

    if (UI_PushUp_Counter % 101 == 0) {
        // 下横线
        for (int i = 0; i < 7; i++) {
            Line_Draw(&UI_Graph7.imageData[i], G_aim_mark[i], UI_Graph_Add, 6, UI_Color_Yellow, 1, 860 + 10 * i, 540 - 27 * i, 1060 - 10 * i, 540 - 27 * i);
        }
        UI_PushUp_Graphs(7, &UI_Graph7);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 109 == 0) {
        // 竖线
        Line_Draw(&UI_Graph1[0].imageData[0], G_aim_mark[7], UI_Graph_Add, 6, UI_Color_Green, 1, 960, 300, 960, 780);
        UI_PushUp_Graphs(1, &UI_Graph1[0]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 127 == 0) {
        // 自瞄圆
        Circle_Draw(&UI_Graph1[1].imageData[0], (char *)"701", UI_Graph_Add, 7, UI_Color_Orange, 1, 960, 540, 60);
        UI_PushUp_Graphs(1, &UI_Graph1[1]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 137 == 0) {
        // 各种标志位名称：摩擦轮状态（ON为开启，OFF为停止）、自瞄状态（ON为开启，OFF为停止）
        Char_Draw(&UI_String[0].String, (char *)"501", UI_Graph_Add, 5, UI_Color_Green, 18, 20, 3, 285, 682, (char *)"Fric:\tOFF\nAuto:\tOFF");
        UI_PushUp_String(&UI_String[0]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 151 == 0) {
        // P轴Y轴角度；
        Char_Draw(&UI_String[1].String, (char *)"201", UI_Graph_Add, 2, UI_Color_Orange, 18, 15, 3, 1410, 712, (char *)"Pitch:\n\n  Yaw:");
        UI_PushUp_String(&UI_String[1]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 167 == 0) {
        // 卡弹状态，退弹次数计数；
        Char_Draw(&UI_String[2].String, (char *)"202", UI_Graph_Add, 2, UI_Color_Pink, 12, 16, 2, 285, 800, (char *)"Block:\n\nReturn:");
        UI_PushUp_String(&UI_String[2]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 179 == 0) {
        // 卡弹状态，退弹次数计数；
        Float_Draw(&UI_Graph2[0].imageData[0], (char *)"203", UI_Graph_Add, 2, UI_Color_Pink, 12, 0, 2, 385, 800, (float)flag.trig_block_flag);
        Float_Draw(&UI_Graph2[0].imageData[1], (char *)"204", UI_Graph_Add, 2, UI_Color_Pink, 12, 0, 2, 385, 764, (float)flag.return_trig_count);
        UI_PushUp_Graphs(2, &UI_Graph2[0]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 191 == 0) {
        // P轴Y轴角度；
        Float_Draw(&UI_Graph2[1].imageData[0], (char *)"205", UI_Graph_Add, 2, UI_Color_Orange, 18, 2, 3, 1510, 712, INS.Roll + gimbal.pitch_modify);
        Float_Draw(&UI_Graph2[1].imageData[1], (char *)"206", UI_Graph_Add, 2, UI_Color_Orange, 18, 2, 3, 1510, 659, INS.YawTotalAngle + gimbal.yaw_modify);
        UI_PushUp_Graphs(2, &UI_Graph2[1]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 199 == 0) {
        // Life?；
        Char_Draw(&UI_String[3].String, (char *)"401", UI_Graph_Add, 4, UI_Color_Pink, 24, 6, 4, 910, 884, (char *)"Life?");
        UI_PushUp_String(&UI_String[3]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 211 == 0) {
        // Just So So!；
        Char_Draw(&UI_String[4].String, (char *)"402", UI_Graph_Add, 4, UI_Color_Pink, 30, 12, 5, 810, 824, (char *)"Just So So!");
        UI_PushUp_String(&UI_String[4]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 29 == 0) {
        if (flag.auto_flag == 1) {
            Line_Draw(&UI_Graph5.imageData[0], (char *)"702", UI_Graph_Add, 7, UI_Color_Pink, 1, 900, 540, 1020, 540);
            Line_Draw(&UI_Graph5.imageData[1], (char *)"703", UI_Graph_Add, 7, UI_Color_Pink, 1, 960 + 2, 540 + 2, 960 + 15, 540 + 15);
            Line_Draw(&UI_Graph5.imageData[2], (char *)"704", UI_Graph_Add, 7, UI_Color_Pink, 1, 960 + 2, 540 - 2, 960 + 15, 540 - 15);
            Line_Draw(&UI_Graph5.imageData[3], (char *)"705", UI_Graph_Add, 7, UI_Color_Pink, 1, 960 - 2, 540 + 2, 960 - 15, 540 + 15);
            Line_Draw(&UI_Graph5.imageData[4], (char *)"706", UI_Graph_Add, 7, UI_Color_Pink, 1, 960 - 2, 540 - 2, 960 - 15, 540 - 15);
        } else if (flag.auto_flag == 0) {
            Line_Draw(&UI_Graph5.imageData[0], (char *)"702", UI_Graph_Del, 7, UI_Color_Pink, 1, 900, 540, 1020, 540);
            Line_Draw(&UI_Graph5.imageData[1], (char *)"703", UI_Graph_Del, 7, UI_Color_Pink, 1, 960 + 2, 540 + 2, 960 + 15, 540 + 15);
            Line_Draw(&UI_Graph5.imageData[2], (char *)"704", UI_Graph_Del, 7, UI_Color_Pink, 1, 960 + 2, 540 - 2, 960 + 15, 540 - 15);
            Line_Draw(&UI_Graph5.imageData[3], (char *)"705", UI_Graph_Del, 7, UI_Color_Pink, 1, 960 - 2, 540 + 2, 960 - 15, 540 + 15);
            Line_Draw(&UI_Graph5.imageData[4], (char *)"706", UI_Graph_Del, 7, UI_Color_Pink, 1, 960 - 2, 540 - 2, 960 - 15, 540 - 15);
        }
        UI_PushUp_Graphs(5, &UI_Graph5);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 31 == 0) {
        Float_Draw(&UI_Graph2[0].imageData[0], (char *)"203", UI_Graph_Change, 2, UI_Color_Pink, 12, 0, 2, 385, 800, (float)flag.trig_block_flag);
        Float_Draw(&UI_Graph2[0].imageData[1], (char *)"204", UI_Graph_Change, 2, UI_Color_Pink, 12, 0, 2, 385, 764, (float)flag.return_trig_count);
        UI_PushUp_Graphs(2, &UI_Graph2[0]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 37 == 0) {
        Float_Draw(&UI_Graph2[1].imageData[0], (char *)"205", UI_Graph_Change, 2, UI_Color_Orange, 18, 2, 3, 1510, 712, INS.Roll + gimbal.pitch_modify);
        Float_Draw(&UI_Graph2[1].imageData[1], (char *)"206", UI_Graph_Change, 2, UI_Color_Orange, 18, 2, 3, 1510, 659, INS.YawTotalAngle + gimbal.yaw_modify);
        UI_PushUp_Graphs(2, &UI_Graph2[1]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }

    if (UI_PushUp_Counter % 41 == 0) {
        if (flag.auto_flag == 0 && flag.fric_flag == 1) {
            Char_Draw(&UI_String[0].String, (char *)"501", UI_Graph_Change, 5, UI_Color_Green, 18, 20, 3, 285, 682, (char *)"Fric:\tON \nAuto:\tOFF");
        } else if (flag.auto_flag == 1 && flag.fric_flag == 1) {
            Char_Draw(&UI_String[0].String, (char *)"501", UI_Graph_Change, 5, UI_Color_Green, 18, 20, 3, 285, 682, (char *)"Fric:\tON \nAuto:\tON ");
        } else if (flag.auto_flag == 1 && flag.fric_flag == 0) {
            Char_Draw(&UI_String[0].String, (char *)"501", UI_Graph_Change, 5, UI_Color_Green, 18, 20, 3, 285, 682, (char *)"Fric:\tOFF\nAuto:\tON ");
        } else if (flag.auto_flag == 0 && flag.fric_flag == 0) {
            Char_Draw(&UI_String[0].String, (char *)"501", UI_Graph_Change, 5, UI_Color_Green, 18, 20, 3, 285, 682, (char *)"Fric:\tOFF\nAuto:\tOFF");
        }
        UI_PushUp_String(&UI_String[0]);
        vTaskDelay(15);
        UI_PushUp_Counter++;
    }
}

/**
 * @brief 机器人、客户端ID判断
 * @return {*}
 */
void ID_Judge(void)
{
    // 机器人角色确定
    if (referee.game_robot_state_.robot_id == 6)  // 红6
    {
        Robot_ID = UI_Data_RobotID_RAerial;
        Cilent_ID = UI_Data_CilentID_RAerial;
    } else if (referee.game_robot_state_.robot_id == 106)  // 蓝6
    {
        Robot_ID = UI_Data_RobotID_BAerial;
        Cilent_ID = UI_Data_CilentID_BAerial;
    }
}

/**
 * @brief UI初始化
 * @return {*}
 */
void UI_init(void)
{
    memset(&UI_Graph1, 0, sizeof(UI_Graph1));
    memset(&UI_Graph2, 0, sizeof(UI_Graph2));
    memset(&UI_Graph5, 0, sizeof(UI_Graph5));
    memset(&UI_Graph7, 0, sizeof(UI_Graph7));
    for (uint8_t i = 0; i < 5; i++) {
        memset(&UI_String[i], 0, sizeof(UI_String[0]));
    }
    memset(&UI_Delete, 0, sizeof(UI_Delete));
    memset(&CH_XTL_State, 0, sizeof(CH_XTL_State));
    memset(&CH_Shoot, 0, sizeof(CH_Shoot));
}

/*绘图坐标系

    屏幕：长 * 宽 = 1920*1280

    坐标原点：屏幕左下角为 (0,0)

    坐标轴：坐标原点水平向右，x增加；坐标原点竖直向上，y增加
*/

/**
 * @brief 绘制直线
 * @param *image           存放图形数据
 * @param imagename[3]     图形名称，用于表示更改
 * @param Graph_Operate    图形操作（增加、改变、删除）
 * @param Graph_Layer      图层（0-9）
 * @param Graph_Color      图形颜色
 * @param Graph_Width      图形线宽
 * @param Start_x,Start_y  直线起始坐标
 * @param End_x,End_y      直线结束坐标
 * @return {*}
 */
void Line_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y)
{
    int i;
    for (i = 0; i < 3; i++)
        image->graphic_name[i] = imagename[i];
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/**
 * @brief 绘制矩形
 * @param *image           存放图形数据
 * @param imagename[3]        图形名称，用于表示更改
 * @param Graph_Operate    图形操作（增加、改变、删除）
 * @param Graph_Layer      图层（0-9）
 * @param Graph_Color      图形颜色
 * @param Graph_Width      图形线宽
 * @param Start_x,Start_y  矩形左上角坐标
 * @param End_x,End_y      矩形右下角坐标
 * @return {*}
 */
void Rectangle_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 End_x, u32 End_y)
{
    int i;
    for (i = 0; i < 3; i++)
        image->graphic_name[i] = imagename[i];
    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/**
 * @brief 绘制整圆
 * @param *image           存放图形数据
 * @param imagename[3]     图形名称，用于表示更改
 * @param Graph_Operate    图形操作（增加、改变、删除）
 * @param Graph_Layer      图层（0-9）
 * @param Graph_Color      图形颜色
 * @param Graph_Width      图形线宽
 * @param Start_x,Start_y  圆心坐标
 * @param Graph_Radius     圆形半径
 * @return {*}
 */
void Circle_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 Graph_Radius)
{
    int i;
    for (i = 0; i < 3; i++)
        image->graphic_name[i] = imagename[i];
    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/**
 * @brief 绘制圆弧
 * @param *image            存放图形数据
 * @param imagename[3]      图形名称，用于表示更改
 * @param Graph_Operate     图形操作（增加、改变、删除）
 * @param Graph_Layer       图层（0-9）
 * @param Graph_Color       图形颜色
 * @param Graph_Width       图形线宽
 * @param Graph_StartAngle,Graph_EndAngle 圆弧开始、终止角度
 * @param Start_x,Start_y   圆心坐标
 * @param x_Length,y_Length x,y方向上轴长（类似椭圆）
 * @return {*}
 */
void Arc_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_StartAngle, u32 Graph_EndAngle, u32 Graph_Width, u32 Start_x, u32 Start_y, u32 x_Length, u32 y_Length)
{
    int i;

    for (i = 0; i < 3; i++)
        image->graphic_name[i] = imagename[i];
    image->graphic_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/**
 * @brief 绘制浮点型数据
 * @param *image           存放图形数据
 * @param imagename[3]     图形名称，用于表示更改
 * @param Graph_Operate    图形操作（增加、改变、删除）
 * @param Graph_Layer      图层（0-9）
 * @param Graph_Color      图形颜色
 * @param Graph_Width      图形线宽
 * @param Graph_Size       字号
 * @param Graph_Digit      小数位数
 * @param Start_x,Start_y  数据开始坐标
 * @param Graph_Float      需要显示的浮点数
 * @return {*}
 */
void Float_Draw(Graph_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_Digit, u32 Graph_Width, u32 Start_x, u32 Start_y, float FloatData)
{
    int i;

    for (i = 0; i < 3; i++)
        image->graphic_name[i] = imagename[i];
    image->graphic_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;
    int32_t IntData = FloatData * 1000;
    image->radius = (IntData & 0x000003ff) >> 0;
    image->end_x = (IntData & 0x001ffc00) >> 10;
    image->end_y = (IntData & 0xffe00000) >> 21;
}

/**
 * @brief 绘制字符串数据
 * @param *image           存放图形数据
 * @param imagename[3]     图形名称，用于表示更改
 * @param Graph_Operate    图形操作（增加、改变、删除）
 * @param Graph_Layer      图层（0-9）
 * @param Graph_Color      图形颜色
 * @param Graph_Size       字号
 * @param Graph_Digit      字符位数
 * @param Graph_Width      图形线宽
 * @param Start_x,Start_y  数据开始坐标
 * @param *Char_Data      需要显示的字符串的指针
 * @return {*}
 */
void Char_Draw(String_Data *image, char imagename[3], u32 Graph_Operate, u32 Graph_Layer, u32 Graph_Color, u32 Graph_Size, u32 Graph_Digit, u32 Graph_Width, u32 Start_x, u32 Start_y, char *Char_Data)
{
    int i;

    for (i = 0; i < 3; i++)
        image->Graph_Control.graphic_name[i] = imagename[i];
    image->Graph_Control.graphic_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = Graph_Digit;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;

    for (i = 0; i < Graph_Digit; i++) {
        image->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/**
 * @brief 更新UI函数（非字符串数据）
 * @param cnt        图形个数（目前只支持1、2、5、7，若输出数不满足，需要做数学题）
 * @param *Graphs    需要绘制的图形数据
 * @return 无
 */
void UI_PushUp_Graphs(uint8_t cnt, void *Graphs)
{
    UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs;

    Graph->framehead.SOF = UI_SOF;
    Graph->framehead.Data_Length = 6 + cnt * 15;
    Graph->framehead.Seq = UI_Seq;
    Graph->framehead.CRC8 = Get_CRC8_Check_Sum_UI((uint8_t *)Graph, 4, 0xFF);
    Graph->framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    switch (cnt) {
        case 1:
            Graph->datahead.Data_ID = UI_Data_ID_Draw1;
            break;
        case 2:
            Graph->datahead.Data_ID = UI_Data_ID_Draw2;
            break;
        case 5:
            Graph->datahead.Data_ID = UI_Data_ID_Draw5;
            break;
        case 7:
            Graph->datahead.Data_ID = UI_Data_ID_Draw7;
            break;
        default:
            break;
    }
    Graph->datahead.Sender_ID = Robot_ID;
    Graph->datahead.Receiver_ID = Cilent_ID;  // 填充操作数据

    if (cnt == 1) {
        UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
        Graph1->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2, 0xFFFF);
    } else if (cnt == 2) {
        UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
        Graph2->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2, 0xFFFF);
    } else if (cnt == 5) {
        UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
        Graph5->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2, 0xFFFF);
    } else if (cnt == 7) {
        UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
        Graph7->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2, 0xFFFF);
    }

    if (cnt == 1)
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Graph, sizeof(UI_Graph1_t));
    else if (cnt == 2)
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Graph, sizeof(UI_Graph2_t));
    else if (cnt == 5)
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Graph, sizeof(UI_Graph5_t));
    else if (cnt == 7)
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Graph, sizeof(UI_Graph7_t));

    UI_Seq++;  // 包序号+1
}

/**
 * @brief 更新UI函数（字符串）
 * @param *String    存放字符串数据的变量（一次只能更新一个字符串数据）
 * @return 无
 */
void UI_PushUp_String(UI_String_t *String)
{
    String->framehead.SOF = UI_SOF;
    String->framehead.Data_Length = 6 + 45;
    String->framehead.Seq = UI_Seq;
    String->framehead.CRC8 = Get_CRC8_Check_Sum_UI((uint8_t *)String, 4, 0xFF);
    String->framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    String->datahead.Data_ID = UI_Data_ID_DrawChar;
    String->datahead.Sender_ID = Robot_ID;
    String->datahead.Receiver_ID = Cilent_ID;  // 填充操作数据

    String->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)String, sizeof(UI_String_t) - 2, 0xFFFF);

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)String, sizeof(UI_String_t));

    UI_Seq++;  // 包序号+1
}

/**
 * @brief 删除操作
 * @param *Delete    存放删除操作的变量
 * @return 无
 */
void UI_PushUp_Delete(UI_Delete_t *Delete)
{
    Delete->framehead.SOF = UI_SOF;
    Delete->framehead.Data_Length = 6 + 2;
    Delete->framehead.Seq = UI_Seq;
    Delete->framehead.CRC8 = Get_CRC8_Check_Sum_UI((uint8_t *)Delete, 4, 0xFF);
    Delete->framehead.CMD_ID = UI_CMD_Robo_Exchange;  // 填充包头数据

    Delete->datahead.Data_ID = UI_Data_ID_Del;
    Delete->datahead.Sender_ID = Robot_ID;
    Delete->datahead.Receiver_ID = Cilent_ID;  // 填充操作数据

    Delete->CRC16 = Get_CRC16_Check_Sum_UI((uint8_t *)Delete, sizeof(Delete) - 2, 0xFFFF);

    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Delete, sizeof(UI_Delete_t));

    UI_Seq++;  // 包序号+1
}

/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff;

const unsigned char CRC8_TAB_UI[256] =
    {
        0x00,
        0x5e,
        0xbc,
        0xe2,
        0x61,
        0x3f,
        0xdd,
        0x83,
        0xc2,
        0x9c,
        0x7e,
        0x20,
        0xa3,
        0xfd,
        0x1f,
        0x41,
        0x9d,
        0xc3,
        0x21,
        0x7f,
        0xfc,
        0xa2,
        0x40,
        0x1e,
        0x5f,
        0x01,
        0xe3,
        0xbd,
        0x3e,
        0x60,
        0x82,
        0xdc,
        0x23,
        0x7d,
        0x9f,
        0xc1,
        0x42,
        0x1c,
        0xfe,
        0xa0,
        0xe1,
        0xbf,
        0x5d,
        0x03,
        0x80,
        0xde,
        0x3c,
        0x62,
        0xbe,
        0xe0,
        0x02,
        0x5c,
        0xdf,
        0x81,
        0x63,
        0x3d,
        0x7c,
        0x22,
        0xc0,
        0x9e,
        0x1d,
        0x43,
        0xa1,
        0xff,
        0x46,
        0x18,
        0xfa,
        0xa4,
        0x27,
        0x79,
        0x9b,
        0xc5,
        0x84,
        0xda,
        0x38,
        0x66,
        0xe5,
        0xbb,
        0x59,
        0x07,
        0xdb,
        0x85,
        0x67,
        0x39,
        0xba,
        0xe4,
        0x06,
        0x58,
        0x19,
        0x47,
        0xa5,
        0xfb,
        0x78,
        0x26,
        0xc4,
        0x9a,
        0x65,
        0x3b,
        0xd9,
        0x87,
        0x04,
        0x5a,
        0xb8,
        0xe6,
        0xa7,
        0xf9,
        0x1b,
        0x45,
        0xc6,
        0x98,
        0x7a,
        0x24,
        0xf8,
        0xa6,
        0x44,
        0x1a,
        0x99,
        0xc7,
        0x25,
        0x7b,
        0x3a,
        0x64,
        0x86,
        0xd8,
        0x5b,
        0x05,
        0xe7,
        0xb9,
        0x8c,
        0xd2,
        0x30,
        0x6e,
        0xed,
        0xb3,
        0x51,
        0x0f,
        0x4e,
        0x10,
        0xf2,
        0xac,
        0x2f,
        0x71,
        0x93,
        0xcd,
        0x11,
        0x4f,
        0xad,
        0xf3,
        0x70,
        0x2e,
        0xcc,
        0x92,
        0xd3,
        0x8d,
        0x6f,
        0x31,
        0xb2,
        0xec,
        0x0e,
        0x50,
        0xaf,
        0xf1,
        0x13,
        0x4d,
        0xce,
        0x90,
        0x72,
        0x2c,
        0x6d,
        0x33,
        0xd1,
        0x8f,
        0x0c,
        0x52,
        0xb0,
        0xee,
        0x32,
        0x6c,
        0x8e,
        0xd0,
        0x53,
        0x0d,
        0xef,
        0xb1,
        0xf0,
        0xae,
        0x4c,
        0x12,
        0x91,
        0xcf,
        0x2d,
        0x73,
        0xca,
        0x94,
        0x76,
        0x28,
        0xab,
        0xf5,
        0x17,
        0x49,
        0x08,
        0x56,
        0xb4,
        0xea,
        0x69,
        0x37,
        0xd5,
        0x8b,
        0x57,
        0x09,
        0xeb,
        0xb5,
        0x36,
        0x68,
        0x8a,
        0xd4,
        0x95,
        0xcb,
        0x29,
        0x77,
        0xf4,
        0xaa,
        0x48,
        0x16,
        0xe9,
        0xb7,
        0x55,
        0x0b,
        0x88,
        0xd6,
        0x34,
        0x6a,
        0x2b,
        0x75,
        0x97,
        0xc9,
        0x4a,
        0x14,
        0xf6,
        0xa8,
        0x74,
        0x2a,
        0xc8,
        0x96,
        0x15,
        0x4b,
        0xa9,
        0xf7,
        0xb6,
        0xe8,
        0x0a,
        0x54,
        0xd7,
        0x89,
        0x6b,
        0x35,
};

unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB_UI[ucIndex];
    }
    return (ucCRC8);
}

uint16_t CRC_INIT_UI = 0xffff;

const uint16_t wCRC_Table_UI[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
                                                         0x00ff];
    }
    return wCRC;
}
