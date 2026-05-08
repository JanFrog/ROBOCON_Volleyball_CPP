#pragma once
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <cerrno>
#include <string>
#include <cstring>
#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>


enum IMU_DATA_TYPE
{
    version,        // 版本号
    raw_acc,        // 原始加速度数据
    raw_gyro,       // 原始陀螺仪数据
    raw_mag,        // 原始磁力计数据
    quaternion,     // 四元数数据
    euler,          // 欧拉角数据
    baro,           // 压力计数据
    algorithm,      // 算法选择
    calib_IMU,      // IMU 校准
    calib_MAG,      // 磁力计校准
    calib_TEMP,     // 温度校准 
    reset_flash,    // 重置闪存
};



class IMU_I2C
{
public:

    IMU_I2C(uint8_t port);
    ~IMU_I2C();

    bool set_port(uint8_t port);

    bool activate();
    bool deactivate();


    // Read
    // *******************
    bool get_version(std::string& version);                             // 版本号
    bool get_raw_acc(std::vector<double>& acc);                         // 加速度
    bool get_raw_gyro(std::vector<double>& gyro);                       // 陀螺仪
    bool get_raw_mag(std::vector<double>& mag);                         // 磁力计
    bool get_quaternion(std::vector<double>& quaternion);               // 四元数
    bool get_euler(std::vector<double>& euler, bool ToAngle = false);   // 欧拉角
    // *******************


    // Write
    // *******************
    bool set_algorithm(uint8_t algorithm);
    bool calibration_imu();
    bool calibration_mag();
    bool calibration_temperature(int16_t temperature);
    bool reset_user_data();



private:
    
    uint8_t port;                              // I2C 端口号
    int fd = -1;
    static const uint8_t device_addr = 0x23;   // 设备地址 (七位二进制)

    int file_desc;                             // 文件描述符


    // 寄存器地址信息
    static const uint8_t
        FUNC_version    = 0x01,  // 版本号
        FUNC_raw_acc    = 0x04,  // 原始加速度数据
        FUNC_raw_gyro   = 0x0A,  // 原始陀螺仪数据
        FUNC_raw_mag    = 0x10,  // 原始磁力计数据
        FUNC_quaternion = 0x16,  // 四元数数据
        FUNC_euler      = 0x26,  // 欧拉角数据
        FUNC_baro       = 0x32,  // 压力计数据

        FUNC_algorithm  = 0x61,  // 算法选择

        FUNC_calib_IMU  = 0x70,  // IMU 校准
        FUNC_calib_MAG  = 0x71,  // 磁力计校准
        FUNC_calib_TEMP = 0x73,  // 温度校准

        FUNC_reset_flash = 0xA0;  // 重置闪存


    static std::map<IMU_DATA_TYPE, uint8_t> addr_map;   // 寄存器地址字典
    static std::map<IMU_DATA_TYPE, bool> Writable_map;  // 是否可写字典
    static std::map<IMU_DATA_TYPE, uint8_t> Byte_map;   // 数据字节数字典


    bool Write_imu(IMU_DATA_TYPE data_type, uint8_t* data);     // 写入数据
    bool Read_imu(IMU_DATA_TYPE data_type, uint8_t* data);      // 读取数据


    bool get_file_description_num(int port, int& file_desc);
    bool block_for_calibration(IMU_DATA_TYPE data_type, int32_t timeout_ms = -1);


};