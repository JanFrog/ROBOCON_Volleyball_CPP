#pragma once

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <system_error>
#include <signal.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libsocketcan.h>





struct CAN_PARAMETERS   // CAN参数包
{
    std::string interface_name = "can0";    // can接口 系统注册名
    int bitrate = 500000;                   // 基础波特率

    int enable_canfd = 0;                   // ?启用 CAN FD
    int data_bitrate = 2000000;              // 数据段波特率 (若启用FD)
};





class CAN
{
public:

    // 构造
    CAN(const std::string interface_name = "can0", int bitrate = 500000, bool enable_canfd = false, int data_bitrate = 500000);
    CAN(const CAN_PARAMETERS& params);
    CAN();

    // 析构
    ~CAN();

    // 使能
    bool activate(std::stringstream& logger);
    // 失能
    bool deactivate(std::stringstream& logger);


    // 重设 CAN 参数
    bool set_parameters(const CAN_PARAMETERS& params, std::stringstream& logger);


    // 发送 CAN 帧
    bool send(uint32_t id, uint8_t data_len, void* data, std::stringstream& logger);
    // 接收 CAN 帧
    bool receive(can_frame& frame, std::stringstream& logger);
    bool receive(canfd_frame& frame, std::stringstream& logger);
    

private:

    CAN_PARAMETERS config;  // 默认配置

    can_frame Frame;        // 标准 CAN 帧
    canfd_frame Frame_FD;   // CANFD 帧

    bool ACTIVATED;             // 接口状态
    can_frame received_frame;   // 接收的 CAN 帧

    int sock;   // 套接字标识
};