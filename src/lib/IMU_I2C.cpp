#include "IMU_I2C.h"



std::map<IMU_DATA_TYPE, uint8_t> IMU_I2C::addr_map = {
    {version,       FUNC_version},
    {raw_acc,       FUNC_raw_acc},
    {raw_gyro,      FUNC_raw_gyro},
    {raw_mag,       FUNC_raw_mag},
    {quaternion,    FUNC_quaternion},
    {euler,         FUNC_euler},
    {baro,          FUNC_baro},
    {algorithm,     FUNC_algorithm},
    {calib_IMU,     FUNC_calib_IMU},
    {calib_MAG,     FUNC_calib_MAG},
    {calib_TEMP,    FUNC_calib_TEMP},
    {reset_flash,   FUNC_reset_flash},
};


std::map<IMU_DATA_TYPE, uint8_t> IMU_I2C::Byte_map = {
    {version,       3},
    {raw_acc,       6},
    {raw_gyro,      6},
    {raw_mag,       6},
    {quaternion,    16},
    {euler,         12},
    {baro,          16},
    {algorithm,     1},
    {calib_IMU,     1},
    {calib_MAG,     1},
    {calib_TEMP,    2},
    {reset_flash,   1},
};


std::map<IMU_DATA_TYPE, bool> IMU_I2C::Writable_map = {
    {version,       false},
    {raw_acc,       false},
    {raw_gyro,      false},
    {raw_mag,       false},
    {quaternion,    false},
    {euler,         false},
    {baro,          false},
    {algorithm,     true},
    {calib_IMU,     true},
    {calib_MAG,     true},
    {calib_TEMP,    true},
    {reset_flash,   true},
};






IMU_I2C::IMU_I2C(uint8_t port)
{
    this->port = port;
    this->get_file_description_num(port, this->fd);
}


IMU_I2C::~IMU_I2C()
{
    this->deactivate();
}





bool IMU_I2C::activate()
{
    if(this->fd < 0)
    {
        std::cerr << "I2C 总线尚未设置" << std::endl;
        return false;
    }

    if (ioctl(this->fd, I2C_SLAVE, this->device_addr) < 0) {
        std::cerr << "无法设置从机地址 0x" << std::hex << (int)this->device_addr << std::endl;
        close(this->fd);
        this->fd = -1;
        return false;
    }

    return true;
}




bool IMU_I2C::deactivate()
{
    if(this->fd < 0)
        return true;
    
    close(this->fd);
    this->fd = -1;
    return true;
}




bool IMU_I2C::block_for_calibration(IMU_DATA_TYPE data_type, int32_t timeout_ms)
{
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    uint8_t result = 0;

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        i2c_smbus_read_i2c_block_data(this->fd, this->addr_map[data_type], 1, &result);

        if(result != 0)
        {
            return true;
        }

        if(timeout_ms > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() >= timeout_ms)
        {
            return false;
        }

    }
    return false;
}




bool IMU_I2C::Read_imu(IMU_DATA_TYPE data_type, uint8_t* data)
{
    int ret = i2c_smbus_read_i2c_block_data(this->fd, this->addr_map[data_type], this->Byte_map[data_type], data);

    if(ret != this->Byte_map[data_type])
        return false;

    return true;
}





bool IMU_I2C::Write_imu(IMU_DATA_TYPE data_type, uint8_t* data)
{
    if(!Writable_map[data_type])
        return false;

    int ret = i2c_smbus_write_i2c_block_data(this->fd, this->addr_map[data_type], this->Byte_map[data_type], data);
    if(ret != 0)
        return false;

    return true;
}





bool IMU_I2C::get_file_description_num(int port, int& file_desc)
{
    std::string device_name = "/dev/i2c-" + std::to_string(port);

    file_desc = open(device_name.c_str(), O_RDWR);

    if (file_desc < 0) {
        std::cerr << "文件描述符失败: " << device_name << std::endl;
        return false;
    }

    return true;
}





bool IMU_I2C::set_port(uint8_t port)
{
    this->port = port;
    return this->get_file_description_num(port, this->fd);
}





bool IMU_I2C::get_version(std::string& version)
{
    int8_t version_data[3];

    bool ret = this->Read_imu(IMU_DATA_TYPE::version, (uint8_t*)version_data);
    if(!ret)
        return false;
        
    std::stringstream ss;
    ss << (int)version_data[0] << "." << (int)version_data[1] << "." << (int)version_data[2];
    version = ss.str();

    return true;
}




bool IMU_I2C::get_raw_acc(std::vector<double>& acc)
{
    static const double scale = 16.00 / 32767.0;
    acc.resize(3);

    int16_t acc_data[3];

    bool ret = this->Read_imu(IMU_DATA_TYPE::raw_acc, (uint8_t*)acc_data);

    if(!ret)
        return false;
    
    for(int i = 0; i < 3; i++)
    {
        acc[i] = acc_data[i] * scale;
    }

    return true;

}




bool IMU_I2C::get_raw_gyro(std::vector<double>& gyro)
{
    static const double scale = M_PI / 180.0;
    int16_t gyro_data[3];

    gyro.resize(3);
    bool ret = this->Read_imu(IMU_DATA_TYPE::raw_gyro, (uint8_t*)gyro_data);
    if(!ret)
        return false;
    
    for(int i = 0; i < 3; i++)
    {
        gyro[i] = gyro_data[i] * scale;
    }
    return true;
}



bool IMU_I2C::get_raw_mag(std::vector<double>& mag)
{
    static const double scale = 800.0 / 32767.0;
    int16_t mag_data[3];

    mag.resize(3);
    bool ret = this->Read_imu(IMU_DATA_TYPE::raw_mag, (uint8_t*)mag_data);
    if(!ret)
        return false;
    
    for(int i = 0; i < 3; i++)
    {
        mag[i] = mag_data[i] * scale;
    }
    return true;
}



bool IMU_I2C::get_quaternion(std::vector<double>& quaternion)
{
    quaternion.resize(4);
    _Float32 quaternion_data[4];

    bool ret = this->Read_imu(IMU_DATA_TYPE::quaternion, (uint8_t*)quaternion_data);
    
    if(!ret)
        return false;
    
    for(int i = 0; i < 4; i++)
    {
        quaternion[i] = quaternion_data[i];
    }
    return true;
}





bool IMU_I2C::get_euler(std::vector<double>& euler, bool ToAngle)
{
    euler.resize(3);
    double scale = ToAngle ?  (180.0 / M_PI) : 1.0;
    _Float32 euler_data[3];

    bool ret = this->Read_imu(IMU_DATA_TYPE::euler, (uint8_t*)euler_data);
    if(!ret)
        return false;
    
    for(int i = 0; i < 3; i++)
    {
        euler[i] = euler_data[i] * scale;
    }
    return true;

}





bool IMU_I2C::set_algorithm(uint8_t algorithm)
{
    if(!(algorithm == 6 || algorithm == 9))
        return false;

    return this->Write_imu(IMU_DATA_TYPE::algorithm, (uint8_t*)&algorithm);
}




bool IMU_I2C::calibration_imu()
{
    int8_t command = 0x01;

    if(!this->Write_imu(IMU_DATA_TYPE::calib_IMU, (uint8_t*)&command))
        return false;

    if(!this->block_for_calibration(IMU_DATA_TYPE::calib_IMU, 7000))
        return false;
        
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return true;

}





bool IMU_I2C::calibration_mag()
{
    int8_t command = 0x01;

    if(!this->Write_imu(IMU_DATA_TYPE::calib_MAG, (uint8_t*)&command))
        return false;

    if(!this->block_for_calibration(IMU_DATA_TYPE::calib_MAG, 7000))
        return false;
        
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return true;

}






bool IMU_I2C::calibration_temperature(int16_t temperature)
{
    if(temperature < -50 || temperature > 50)
    {
        std::cerr << "Invalid temperature" << std::endl;
        return false;
    }

    temperature *= 100;

    if(!this->Write_imu(IMU_DATA_TYPE::calib_TEMP, (uint8_t*)&temperature))
    {
        return false;
    }
        
    if(!this->block_for_calibration(IMU_DATA_TYPE::calib_TEMP, 2000))
    {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return true;
}



bool IMU_I2C::reset_user_data()
{
    uint8_t command = 0x01;

    if(!this->Write_imu(IMU_DATA_TYPE::reset_flash, &command))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}