#pragma once

#include "CAN.h"
#include <thread>
#include <shared_mutex>
#include <atomic>
#include <deque>



struct Control_msg
{
    uint32_t ID = 0x310;
    int16_t vx=0, vy=0, omega=0;
};


class Messenger: public CAN
{
public:

    Messenger(CAN_PARAMETERS can_params, int frequency=100, int Rx_buffer_size=100);
    ~Messenger();


    bool activate(std::stringstream& logger);
    bool deactivate(std::stringstream& logger);

    bool set_control_state(uint16_t vx, uint16_t vy, uint16_t omega);

    bool any_received(can_frame& frame);


private:

    double Period;              // 单位：秒

    Control_msg control_msg;    // 控制消息

    std::thread msg_update_thread;  // 消息更新线程
    std::shared_mutex Tx_mutex;     // Tx消息锁
    std::shared_mutex Rx_mutex;     // Rx消息锁

    std::atomic<bool> running;      // 线程运行标志
    
    std::deque<can_frame> msg_queue;
    int Rx_buffer_size;

    void keep_updating();           // 消息更新线程函数


};