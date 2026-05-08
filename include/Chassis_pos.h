#pragma once
#include "IMU_I2C.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <shared_mutex>
#include <Eigen/Dense>

// socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

// debug
#include <sstream>





// 初始状态包
struct Zero_state
{
    Eigen::Vector2d acc = {0.0, 0.0};
    double yaw = 0.0;
};


struct Chassis_state
{
    Eigen::Vector2d vel = {0.0, 0.0};     // 速度[vx, vy] (m/s)
    Eigen::Vector2d pos = {0.0, 0.0};     // 位置[x, y]
    double yaw = 0.0;               // yaw (rad)
    uint64_t timestamp_us = 0;      // 时间戳 (微秒)
};



// 底盘位姿类
class Chassis_pos
{
public:
    Chassis_pos(uint8_t I2C_port, bool debug = false, double acc_threshold=0, double g=9.8, std::string server_ip = "192.168.137.1", int server_port = 12345);
    ~Chassis_pos();

    bool Initialize_state(Chassis_state& initial_state);

    void activate();
    void deactivate();

    

private:

    IMU_I2C imu;
    bool initialized;


    // 状态锁
    std::shared_mutex state_lock;
    Zero_state Zs;  // 初始状态，后续状态在此基础上迭代
    Chassis_state state;    // 当前状态
    std::thread imu_thread;
    std::thread sender_thread;

    double acc_threshold; // 加速度死区 (m/s^2)
    double g;



    // socket 相关
    int socket_fd = -1;
    struct sockaddr_in server_addr{};
    


    // 是否激活
    std::atomic<bool> active;

    // 稳定数据序列 (假定IMU静止)
    bool stablize_data(double* data, double* timestamp, int length, double& result, short method=0);

    void syncing_imu();

    void sender();
};