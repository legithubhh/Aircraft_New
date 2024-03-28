/**
 *******************************************************************************
 * @file      : remote_keyboard.cpp
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
#include "remote_keyboard.h"

#include "gimbal.h"
#include "motor_pidmodify.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PidsetModeID controlmode_pidset_flag = remote_pid_flag;  // 针对遥控器，键鼠与自瞄三种控制模式设置三套相适应枚举类型PID参数

PidSwitchMode pitchpid_switchflag = base_pid, yawpid_switchflag = base_pid;  // 针对不同控制模式，在同一电机执行不同任务时，设置不同的PID参数
/* External variables --------------------------------------------------------*/
extern uint8_t remote_key_press[16];
extern uint8_t referee_key_press[16];
/* Private function prototypes -----------------------------------------------*/
void MotorOffset();
void PidFlagInit(PidsetModeID pidsetmode);
void PidSetSwitch();
void RemoteDisconShootCtrl();

void ModeTask()
{
    /*集中修改双轴电机的初始位置偏移量*/
    // MotorOffset();

    /*遥控器控制模式选择*/
    // 右拨杆在上，遥控器控制模式
    if (remote.GetS2() == 1) {
        /*不连续射击控制,需要使用时取消注释，并注释其它代码该if指令其它代码*/
        // RemoteDisconShootCtrl();
        // PidFlagInit(remote_pid_flag);
        // // 当Pitch轴误差达到一定范围时，更改PID参数进行自适应调节，分俯仰角调节以适应重心后偏
        // if (gimbal.angle_[0].GetMeasure() < 0.5) {
        //     if (gimbal.angle_[0].GetError() > 2.5f || gimbal.angle_[0].GetError() < -2.5f) {
        //         pitchpid_switchflag = base_pid;
        //     } else if (gimbal.angle_[0].GetError() > 1.f || gimbal.angle_[0].GetError() < -1.f) {
        //         pitchpid_switchflag = pitch1_pid;
        //     } else {
        //         pitchpid_switchflag = pitch2_pid;
        //     }
        // } else {
        //     if (gimbal.angle_[0].GetError() > 3.5f || gimbal.angle_[0].GetError() < -3.5f) {
        //         pitchpid_switchflag = pitch3_pid;
        //     } else if (gimbal.angle_[0].GetError() > 0.5f || gimbal.angle_[0].GetError() < -0.5f) {
        //         pitchpid_switchflag = pitch4_pid;
        //     } else {
        //         pitchpid_switchflag = pitch5_pid;
        //     }
        // }

        // // 当Yaw轴误差达到一定范围时，更改PID参数进行自适应调节
        // if (gimbal.angle_[1].GetError() > 15.f || gimbal.angle_[1].GetError() < -15.f) {
        //     yawpid_switchflag = base_pid;
        // } else if (gimbal.angle_[1].GetError() > 2.5f || gimbal.angle_[1].GetError() < -2.5f) {
        //     yawpid_switchflag = yaw1_pid;
        // } else {
        //     yawpid_switchflag = yaw2_pid;
        // }
        // PidSetSwitch();
        // RemoteControlMode();

        // 左拨杆在上或中，遥控器手控模式 OR 左拨杆在下，遥控器切换到自瞄模式
        if (remote.GetS1() == 1 || remote.GetS1() == 3) {
            PidFlagInit(remote_pid_flag);
            // 当Pitch轴误差达到一定范围时，更改PID参数进行自适应调节，分俯仰角调节以适应重心后偏
            if (gimbal.angle_[0].GetMeasure() < 0.5) {
                if (gimbal.angle_[0].GetError() > 2.5f || gimbal.angle_[0].GetError() < -2.5f) {
                    pitchpid_switchflag = base_pid;
                } else if (gimbal.angle_[0].GetError() > 1.f || gimbal.angle_[0].GetError() < -1.f) {
                    pitchpid_switchflag = pitch1_pid;
                } else {
                    pitchpid_switchflag = pitch2_pid;
                }
            } else {
                if (gimbal.angle_[0].GetError() > 3.5f || gimbal.angle_[0].GetError() < -3.5f) {
                    pitchpid_switchflag = pitch3_pid;
                } else if (gimbal.angle_[0].GetError() > 0.5f || gimbal.angle_[0].GetError() < -0.5f) {
                    pitchpid_switchflag = pitch4_pid;
                } else {
                    pitchpid_switchflag = pitch5_pid;
                }
            }

            // 当Yaw轴误差达到一定范围时，更改PID参数进行自适应调节
            if (gimbal.angle_[1].GetError() > 15.f || gimbal.angle_[1].GetError() < -15.f) {
                yawpid_switchflag = base_pid;
            } else if (gimbal.angle_[1].GetError() > 2.5f || gimbal.angle_[1].GetError() < -2.5f) {
                yawpid_switchflag = yaw1_pid;
            } else {
                yawpid_switchflag = yaw2_pid;
            }
            PidSetSwitch();
            RemoteControlMode();
        } else {
            PidFlagInit(autoaim_pid_flag);
            PidSetSwitch();
            AutoControlMode();
        }
    }

    // 右拨杆在中，键鼠模式
    if (remote.GetS2() == 3) {
        // 当启用空中支援且有剩余发弹时间(默认向下取整0.9==0）时才能用键鼠模式控制，否则为全停模式
        if (referee.aerial_robot_support_data_.airforce_status == 2 && referee.aerial_robot_support_data_.time_remain > 0) {
            // 当键盘R键按下期间，键鼠模式切换到自瞄模式，松开换回键鼠手控模式
            if (remote_key_press[KEY_R] || referee_key_press[KEY_R] == 1) {
                PidFlagInit(autoaim_pid_flag);
                PidSetSwitch();
                AutoControlMode();
            } else {
                PidFlagInit(keymouse_pid_flag);
                if (gimbal.angle_[0].GetMeasure() < 0.5) {
                    if (gimbal.angle_[0].GetError() > 2.5f || gimbal.angle_[0].GetError() < -2.5f) {
                        pitchpid_switchflag = base_pid;
                    } else if (gimbal.angle_[0].GetError() > 1.f || gimbal.angle_[0].GetError() < -1.f) {
                        pitchpid_switchflag = pitch1_pid;
                    } else {
                        pitchpid_switchflag = pitch2_pid;
                    }
                } else {
                    if (gimbal.angle_[0].GetError() > 3.5f || gimbal.angle_[0].GetError() < -3.5f) {
                        pitchpid_switchflag = pitch3_pid;
                    } else if (gimbal.angle_[0].GetError() > 0.5f || gimbal.angle_[0].GetError() < -0.5f) {
                        pitchpid_switchflag = pitch4_pid;
                    } else {
                        pitchpid_switchflag = pitch5_pid;
                    }
                }

                // 当Yaw轴误差达到一定范围时，更改PID参数进行自适应调节
                if (gimbal.angle_[1].GetError() > 15.f || gimbal.angle_[1].GetError() < -15.f) {
                    yawpid_switchflag = base_pid;
                } else if (gimbal.angle_[1].GetError() > 1.5f || gimbal.angle_[1].GetError() < -1.5f) {
                    yawpid_switchflag = yaw1_pid;
                } else {
                    yawpid_switchflag = yaw2_pid;
                }
                PidSetSwitch();
                KeymouseControlMode();
            }
        } else {
            PidFlagInit(remote_pid_flag);
            PidSetSwitch();
            GimbalStop2ControlMode();
        }
    }

    // 右拨杆在下，急停模式
    if (remote.GetS2() == 2) {
        // 左拨杆在上或中，发弹急停 OR 左拨杆在下，切换到全停模式（拨弹盘，摩擦轮目标速度设为0，双轴目标位置设为0度）
        if (remote.GetS1() == 1 || remote.GetS1() == 3) {
            PidFlagInit(remote_pid_flag);
            // 当Pitch轴误差达到一定范围时，更改PID参数进行自适应调节，分俯仰角调节以适应重心后偏
            if (gimbal.angle_[0].GetMeasure() < 0.5) {
                if (gimbal.angle_[0].GetError() > 2.5f || gimbal.angle_[0].GetError() < -2.5f) {
                    pitchpid_switchflag = base_pid;
                } else if (gimbal.angle_[0].GetError() > 1.f || gimbal.angle_[0].GetError() < -1.f) {
                    pitchpid_switchflag = pitch1_pid;
                } else {
                    pitchpid_switchflag = pitch2_pid;
                }
            } else {
                if (gimbal.angle_[0].GetError() > 3.5f || gimbal.angle_[0].GetError() < -3.5f) {
                    pitchpid_switchflag = pitch3_pid;
                } else if (gimbal.angle_[0].GetError() > 0.5f || gimbal.angle_[0].GetError() < -0.5f) {
                    pitchpid_switchflag = pitch4_pid;
                } else {
                    pitchpid_switchflag = pitch5_pid;
                }
            }

            // 当Yaw轴误差达到一定范围时，更改PID参数进行自适应调节
            if (gimbal.angle_[1].GetError() > 3.5f || gimbal.angle_[1].GetError() < -3.5f) {
                yawpid_switchflag = base_pid;
            } else if (gimbal.angle_[1].GetError() > 2.5f || gimbal.angle_[1].GetError() < -2.5f) {
                yawpid_switchflag = yaw1_pid;
            } else {
                yawpid_switchflag = yaw2_pid;
            }

            PidSetSwitch();
            GimbalStop1ControlMode();
        } else {
            PidFlagInit(remote_pid_flag);
            PidSetSwitch();
            GimbalStop2ControlMode();
        }
    }
}

/**
 * @brief       集中修改双轴电机的初始位置偏移量
 *   @arg       None
 * @retval      None
 * @note        None
 */
void MotorOffset()
{
}

/**
 * @brief       PID参数标志初始化
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidFlagInit(PidsetModeID pidsetmode)
{
    controlmode_pidset_flag = pidsetmode;
    pitchpid_switchflag = base_pid;
    yawpid_switchflag = base_pid;
}

/**
 * @brief       不同模式不同任务下PID设置选择判断,更改controlmode_pidset_flag进入不同PID模式，更改xxx_pid_switchflag进行不同任务下不同设置
 *   @arg       None
 * @retval      None
 * @note        None
 */
void PidSetSwitch()
{
    // 遥控器模式参数
    if (controlmode_pidset_flag == remote_pid_flag) {
        PidSetRemote();
        // 遥控模式Pitch轴PID调制
        switch (pitchpid_switchflag) {
            case base_pid:
                break;
            case pitch1_pid:
                RemotePitchPidDemo1();
                break;
            case pitch2_pid:
                RemotePitchPidDemo2();
                break;
            case pitch3_pid:
                RemotePitchPidDemo3();
                break;
            case pitch4_pid:
                RemotePitchPidDemo4();
                break;
            case pitch5_pid:
                RemotePitchPidDemo5();
                break;
        }
        // 遥控模式Yaw轴PID调制
        switch (yawpid_switchflag) {
            case base_pid:
                break;
            case yaw1_pid:
                RemoteYawPidDemo1();
                break;
            case yaw2_pid:
                RemoteYawPidDemo2();
                break;
        }
    }
    // 键鼠模式参数
    else if (controlmode_pidset_flag == keymouse_pid_flag) {
        PidSetRemote();
        // 键鼠模式Pitch轴PID调制
        switch (pitchpid_switchflag) {
            case base_pid:
                break;
            case pitch1_pid:
                RemotePitchPidDemo1();
                break;
            case pitch2_pid:
                RemotePitchPidDemo2();
                break;
            case pitch3_pid:
                RemotePitchPidDemo3();
                break;
            case pitch4_pid:
                RemotePitchPidDemo4();
                break;
            case pitch5_pid:
                RemotePitchPidDemo5();
                break;
        }
        // 键鼠模式Yaw轴PID调制
        switch (yawpid_switchflag) {
            case base_pid:
                break;
            case yaw1_pid:
                RemoteYawPidDemo1();
                break;
            case yaw2_pid:
                RemoteYawPidDemo2();
                break;
        }
    }
    // 自瞄模式参数
    else if (controlmode_pidset_flag == autoaim_pid_flag) {
        PidSetAutoaim();
        switch (pitchpid_switchflag) {
            case base_pid:
                break;
            case pitch1_pid:
                AutoPitchPidDemo1();
                break;
            default:
                break;
        }
    }
}

/**
 * @brief      不连续射击模式控制
 *   @arg       None
 * @retval      None
 * @note        S1由上拨至中（顺序固定），不连续射击一次；拨至下时，连续射击
 */
void RemoteDisconShootCtrl()
{
    static uint8_t flag = 0;
    if (remote.GetS1() == 1) {
        flag = 1;
        shoot.SetFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(0.0f);
    }

    if (remote.GetS1() == 3 && flag == 1) {
        flag = 1;
        shoot.SetFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(45 * 36.0f);
        flag = 0;
    }
    if (remote.GetS1() == 2) {
        shoot.SetFlag(SPEED_FLAG);
    }
}