#include "Chassis_pos.h"



Chassis_pos::Chassis_pos(uint8_t I2C_port, bool debug, double acc_threshold, double g, std::string server_ip, int server_port):imu(I2C_port)
{
    active.store(false);

    if(!imu.activate())
        throw std::runtime_error("IMU activation failed");
    
    imu.set_algorithm(9);

    this->acc_threshold = acc_threshold;
    this->g = g;
    initialized = false;

    if(debug)
    {
        // 初始化 socket
        this->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(this->socket_fd < -1)
            throw std::runtime_error("socket creation failed");
        
        this->server_addr.sin_family = AF_INET;
        this->server_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
        this->server_addr.sin_port = htons(server_port);
        
    }
}



Chassis_pos::~Chassis_pos()
{
    deactivate();
    if(this->socket_fd > 0)
        close(this->socket_fd);
}




bool Chassis_pos::stablize_data(double* data, double* timestamp, int length, double& result, short method)
{
    switch(method)
    {
    case 0: // 平均值
        result = 0.0;
        for(int i = 0; i < length; i++)
            result += data[i];

        result /= length;

        return true;

    case 1: // KF (待实现)
        result = 0.0;
        return true;

    default:
        return false;
    }

    return false;
}





// 初始化零状态，须保持IMU静止状态
bool Chassis_pos::Initialize_state(Chassis_state& initial_state)
{
    // log
    std::cout << "开始设置零状态，请保持IMU静止" << std::endl;

    std::vector<double> acc;
    std::vector<double> euler;
    double acc_seq[2][100];
    double yaw_seq[100];

    for(int i = 0; i < 100; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(!imu.get_raw_acc(acc) || !imu.get_euler(euler))
        {
            std::cerr << "获取IMU数据失败" << std::endl;
            return false;
        }

        acc_seq[0][i] = acc[0];
        acc_seq[1][i] = acc[1];

        yaw_seq[i] = euler[2];
    }


    // 稳定数据
    double acc_stable[2];
    this->stablize_data(acc_seq[0], nullptr, 100, acc_stable[0], 0);
    this->stablize_data(acc_seq[1], nullptr, 100, acc_stable[1], 0);

    double euler_stable;
    this->stablize_data(yaw_seq, nullptr, 100, euler_stable, 0);


    // 拷贝
    this->Zs.acc << acc_stable[0], acc_stable[1];
    this->Zs.yaw = euler_stable;

    // 设置标志位
    this->initialized = true;

    std::unique_lock<std::shared_mutex> lock_1(this->state_lock);
    this->state = initial_state;
    lock_1.unlock();


    // log
    std::cout << "零状态设置完成" << std::endl;
    std::cout << "加速度零点: " << acc_stable[0] << ", " << acc_stable[1] << std::endl;
    std::cout << "yaw角零点: " << euler_stable << std::endl;
    
    return true;
}






// 同步IMU数据
void Chassis_pos::syncing_imu()
{
    if(!initialized)
    {
        this->active.store(false);
        std::cerr << "未初始化零状态，无法同步IMU数据" << std::endl;
        return;
    }

    std::vector<double> acc;
    std::vector<double> euler;
    Eigen::Vector2d delta_acc;

    Eigen::Vector2d last_vel, last_pos;

    Eigen::Vector2d new_vel, new_pos;

    double yaw;

    // 旋转矩阵
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

    std::chrono::high_resolution_clock::time_point timestamp;
    std::chrono::high_resolution_clock::time_point last_timestamp = std::chrono::high_resolution_clock::now();
    double dt;
    static const int Period_us = 10000;

    while(this->active.load())
    {

        // 获取数据
        if(!imu.get_raw_acc(acc) || !imu.get_euler(euler)){
            std::cerr << "获取IMU数据失败" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(Period_us));
            continue;
        }

        // 打时间戳
        timestamp = std::chrono::high_resolution_clock::now();
        dt = std::chrono::duration_cast<std::chrono::microseconds>(timestamp - last_timestamp).count() / 1e6;
        last_timestamp = timestamp;


        // 加速度变化量
        delta_acc << (acc[0] - Zs.acc[0]), (acc[1] - Zs.acc[1]);

        // 欧拉角变化量
        yaw = euler[2] - Zs.yaw;

        // 欧拉角变化值
        if(yaw > M_PI)
            yaw -= 2 * M_PI;
        else if(yaw< -M_PI)
            yaw += 2 * M_PI;


        // 加速度变化量判断
        if(std::abs(delta_acc(0)) < acc_threshold && std::abs(delta_acc(1)) < acc_threshold){
            std::this_thread::sleep_for(std::chrono::microseconds(Period_us));
            continue;
        }


        // 获取前状态速度
        std::shared_lock<std::shared_mutex> lock_1(this->state_lock);
        last_vel = this->state.vel;
        last_pos = this->state.pos;
        lock_1.unlock();

        R << cos(yaw), -sin(yaw),
             sin(yaw), cos(yaw) ;

        delta_acc = R * delta_acc;
 

        // 速度变化
        new_vel = last_vel + (delta_acc * this->g) * dt;

        // 位置变化 (dx = a_0 * dt^2 / 2 + v_0 * dt)
        new_pos = last_pos + (delta_acc * (this->g * dt * dt * 0.5)) + (last_vel * dt);

        // 更新当前状态位置
        std::unique_lock<std::shared_mutex> lock_2(this->state_lock);
        this->state.yaw = yaw;
        this->state.vel = new_vel;
        this->state.pos = new_pos;
        this->state.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
        lock_2.unlock();



        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - timestamp);

        // 等待剩余时间
        if(dt.count() < Period_us)
            std::this_thread::sleep_for(std::chrono::microseconds(Period_us) - dt);

    }
}



void Chassis_pos::sender()
{
    Eigen::Vector2d pos;
    double yaw;
    std::stringstream ss;

    while(this->active.load())
    {
        if(this->socket_fd > 0)
        {
            
            std::shared_lock<std::shared_mutex> lock_1(this->state_lock);
            pos = this->state.pos;
            yaw = this->state.yaw;
            lock_1.unlock();

            ss << "1 " << pos[0] << " " << pos[1] << " 0 " << pos[0] + 0.2 * cos(yaw) << " " << pos[1] + 0.2 * sin(yaw) << " 0 y_ 0.01" << std::endl;
            std::string msg = ss.str();
            ssize_t sentBytes_1 = sendto(this->socket_fd, msg.c_str(), msg.length(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
            
            if(sentBytes_1 != msg.length())
                std::cerr << "Error: sendto failed" << std::endl;

            ss.str("");
            ss.clear();

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}




void Chassis_pos::activate()
{
    if(this->active.load())
        return;
    
    this->active.store(true);

    this->imu_thread = std::thread(&Chassis_pos::syncing_imu, this);
    this->sender_thread = std::thread(&Chassis_pos::sender, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if(!this->active.load())
    {
        this->imu_thread.join();
        this->sender_thread.join();
        std::cout << "未初始化，已退出" << std::endl;
    }
    
    this->imu_thread.detach();
    this->sender_thread.detach();
}


void Chassis_pos::deactivate()
{
    if(!this->active.load())
        return;

    this->active.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if(this->imu_thread.joinable())
        this->imu_thread.join();
    if(this->sender_thread.joinable())
        this->sender_thread.join();
}
