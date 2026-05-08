#include "points_generator.h"
#include "predictor.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <string>
#include <sstream>
#include <bitset>

// socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>



// int main()
// {

//     int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sockfd < 0) {
//         std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
//         return -1;
//     }

//     struct sockaddr_in destAddr{};
//     destAddr.sin_family = AF_INET;          // IPv4
//     destAddr.sin_port = htons(12345);        // port
//     destAddr.sin_addr.s_addr = inet_addr("192.168.137.1");
//     std::stringstream ss_1, ss_2;
//     std::string msg_1, msg_2;





//     PointsGenerator pg(0.014, 0.284, 9.8);
//     // UKF ukf(0.014, 9.8, 0.284, 0.1, 0.3, 0.9, 3.0, 1.7);

//     // 配置Predictor参数
//     Predictor_Params predictor_params;

//     Eigen::Matrix3d MTX;

//     MTX << 1.31527123e+03,  0.00000000e+00,  5.82870287e+02,
//             0.00000000e+00, 1.31458950e+03,  5.74049534e+02,
//             0.00000000e+00, 0.00000000e+00,  1.00000000e+00;

//     Eigen::Vector3d locate_result;

//     predictor_params.locator_params.mtx = MTX;
//     predictor_params.locator_params.radius = 0.106;
//     predictor_params.locator_params.img_width = 1280;
//     predictor_params.locator_params.img_height = 1024;

//     Predictor pd(predictor_params);

//     float dt = 0.00625;
//     auto dt_duration = std::chrono::duration<float>(dt);


//     Eigen::VectorXd state = pg.set_state(-3,-3,0,4,4,8);
//     Eigen::VectorXd state_noise = pg.get_state(true, 0.05, true);
//     Eigen::VectorXd state_filtered(6);
//     Eigen::VectorXd result(2);
    


//     while (true)
//     {

//         auto t1 = std::chrono::high_resolution_clock::now();
        
//         state = pg.update(dt);
//         state_noise = pg.get_state(true, 0.05, true);
//         // std::cout<< std::endl << state_noise << std::endl;
        
        

//         if(pd.push_get_legacy(state.head<3>(), dt, result, &state_filtered)){

//             // ss_1 << "0 " << state_filtered[0] << ' ' << state_filtered[1] << ' ' << state_filtered[2] << " b_";
//             ss_1 << "0 " << state[0] << ' ' << state[1] << ' ' << state[2] << " b_";
//             ss_2 << "0 " << result[0] << ' ' << result[1] << " 0 y_";
//             msg_1 = ss_1.str();
//             msg_2 = ss_2.str();

            

//             ssize_t sentBytes_1 = sendto(sockfd, msg_1.c_str(), msg_1.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
//             ssize_t sentBytes_2 = sendto(sockfd, msg_2.c_str(), msg_2.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));

//             ss_1.str("");
//             ss_2.str("");
//             ss_1.clear();
//             ss_2.clear();


//             if(sentBytes_1 < 0 || sentBytes_2 <0){
//                 std::cerr << "Did not sent successfully\nmsg 1: " << sentBytes_1 << "\nmsg 2: " << sentBytes_2;
//             }

//         }






//         // std::cout << "\nstate:\n" << state << std::endl;
//         auto t2 = std::chrono::high_resolution_clock::now();
//         std::cout << "time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " μs" << std::endl;
        
//         // std::this_thread::sleep_for(dt_duration - (t2 - t1));
//         // std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
//         if(state(2) < 0){
//             break;
//         }
//     }

//     close(sockfd);
//     return 0;
// }






// int main()
// {

//     // 初始化CAN接口
//     CAN can("can0", 1000000, true, 1000000);
//     std::stringstream can_logger;

//     if(!can.activate(can_logger))
//     {
//         std::cerr << can_logger.str() << std::endl;
//         return -1;
//     }
//     can_logger.str("");
//     can_logger.clear();







//     // 准备CAN帧
    
//     uint64_t msg = 0x0123456789ABCDEF;

//     uint8_t* msg_ptr = new uint8_t[8];
//     uint16_t V_x = 0xcdef, V_y = 0x89ab, Omega = 0x4567, Other = 0x0123;


//     memcpy(msg_ptr, &V_x, sizeof(uint16_t));
//     memcpy(msg_ptr + 2, &V_y, sizeof(uint16_t));
//     memcpy(msg_ptr + 4, &Omega, sizeof(uint16_t));
//     memcpy(msg_ptr + 6, &Other, sizeof(uint16_t));


//     uint8_t msg_len = 8;
//     // for(int i = 0; i < msg_len; i++){
//     //     msg_ptr[i] = i;
//     // }
//     uint32_t msg_id = 0x01;







//     // 发送CAN帧
//     int try_count = 0;
//     int success_count = 0;
//     //msg = *((uint64_t*)msg_ptr);
//     std::chrono::time_point t1 = std::chrono::high_resolution_clock::now();


//     for(int i = 0; i < 2000; i++)
//     {
//         // msg = i;



//         // std::cout << "msg: " << std::hex << msg << std::endl;
//         // std::cout << "msg_ptr: " << std::hex << *(uint64_t*)msg_ptr << std::endl;



//         if(try_count >= 100){


//             std::chrono::time_point t2 = std::chrono::high_resolution_clock::now();

//             std::cout << "attempt_FPS: " << (float)try_count * 1000000 / std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " Hz" << std::endl;
//             std::cout << "success_rate: " << (float)success_count * 100.0 / try_count << "%" << std::endl;

//             std::cout << "try_count: " << try_count << std::endl;
//             std::cout << "success_count: " << success_count << std::endl << std::endl;


//             try_count = 0;
//             success_count = 0;
//             t1 = t2;
//         }


//         can_logger.str("");
//         can_logger.clear();
        
//         if(can.send(msg_id, msg_len, msg_ptr, can_logger))
//         {
//             success_count++;
//         }
//         else 
//         {
//             std::cerr << can_logger.str() << std::endl;
//             return -1;
//         }
//         try_count++;

//         std::this_thread::sleep_for(std::chrono::duration<float>(0.01));
//     } 



//     delete[] msg_ptr;
//     return 0;
// }





// int main()
// {



//     // 初始化CAN接口
//     CAN can("can0", 1000000, false, 10000000);

//     std::stringstream can_logger;

//     if(!can.activate(can_logger))
//     {
//         std::cerr << can_logger.str() << std::endl;
//         return -1;
//     }
//     can_logger.str("");
//     can_logger.clear();








//     can_frame receive_frame;
//     int data_len = 0;

//     while(true)
//     {
//         std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
//         std::cout << can_logger.str() << std::endl;
        
//         can_logger.str("");
//         can_logger.clear();
        
//         if(can.receive(receive_frame, can_logger))
//         {
//             data_len = receive_frame.can_dlc;

//             std::vector<uint8_t> msg_ptr(data_len);



//             // 反转数据字节序
//             for(int i = 0; i < data_len; i++)
//             {
//                 // msg_ptr[i] = (uint8_t)receive_frame.data[data_len - i - 1];
//                 // msg_ptr[i] = (uint8_t)receive_frame.data[i];

//             }


//             std::cout << "receive_frame.can_id: " << std::hex << receive_frame.can_id << std::endl;
//             std::cout << "receive_frame.can_dlc: " << std::dec << data_len << std::endl;
//             std::cout << "receive_frame.data: ";

//             // std::cout << *static_cast<uint64_t*>(&msg_ptr);

//             for(int i = 0; i < data_len; i++)
//             {
//                 std::cout << "0x" << std::hex << static_cast<int>(msg_ptr[i]) << ' ';
//             }

//             std::cout << std::endl;
//         }

//     }


    
//     return 0;
// }







// #include "CAN.h"






// int main()
// {

//     // 初始化CAN接口
//     std::stringstream can_logger;
//     CAN can("can0", 1000000, false, 10000000);

//     if(!can.activate(can_logger))
//     {
//         std::cerr << can_logger.str() << std::endl;
//         return -1;
//     }



//     // 开始发送CAN帧
//     std::chrono::time_point start_time = std::chrono::high_resolution_clock::now();
//     uint64_t msg = 0;
//     uint64_t cmd_1 = 4096;
//     const int Period = 5000;    // ms

//     std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    

//     std::cout << "phase 1" << std::endl;

//     msg = (cmd_1) << (6*8);


//     while(true)
//     {
//         std::cout<< "msg: " << std::bitset<64>(msg) << std::endl;

//         if(!can.send(0x310, 8, &msg, can_logger))
//         {
//             std::cerr << can_logger.str() << std::endl;
//             break;
//         }

//         std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now(); 
//         if(std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() >= Period)
//         {
//             t1 = t2;
//             break;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }




//     std::cout << "phase 2" << std::endl;

//     msg = (cmd_1) << (4*8);

//     while(true)
//     {
//         std::cout<< "msg: " << std::bitset<64>(msg) << std::endl;

//         if(!can.send(0x310, 8, &msg, can_logger))
//         {
//             std::cerr << can_logger.str() << std::endl;
//             break;
//         }

//         std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now(); 
//         if(std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() >= Period)
//         {
//             t1 = t2;
//             break;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }


    

//     std::cout << "phase 3" << std::endl;

//     msg = (cmd_1) << (2*8);

//     while(true)
//     {
//         std::cout<< "msg: " << std::bitset<64>(msg) << std::endl;

//         if(!can.send(0x310, 8, &msg, can_logger))
//         {
//             std::cerr << can_logger.str() << std::endl;
//             break;
//         }

//         std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now(); 
//         if(std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() >= Period)
//         {
//             t1 = t2;
//             break;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }




//     return 0;
// }






// #include "Chassis_pos.h"
// #include <iomanip>

// int main() {


    

//     IMU_I2C imu(1);
//     if(!imu.activate())
//     {
//         std::cerr << "IMU activation failed" << std::endl;
//         return -1;
//     }

//     // if(!imu.calibration_imu())
//     //     std::cerr << "IMU calibration failed" << std::endl;


//     // std::cout << "IMU calibration done" << std::endl;
//     // std::this_thread::sleep_for(std::chrono::milliseconds(2000));


//     // if(!imu.calibration_mag())
//     //     std::cerr << "mag calibration failed" << std::endl;


//     std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
//     int count = 0;

//     std::vector<double> acc, euler, gyro, mag;
//     std::string version;

//     if(!imu.set_algorithm(9)){
//         std::cerr << "set algorithm failed" << std::endl;
//         return -1;
//     }
    
//     while(true)
//     {

//         if(!imu.get_version(version))
//         {
//             std::cerr << "get version failed" << std::endl;
//             return -1;
//         }

//         if(!imu.get_raw_acc(acc))
//         {
//             std::cerr << "get raw acc failed" << std::endl;
//             return -1;
//         }

//         if(!imu.get_euler(euler, true))
//         {
//             std::cerr << "get euler failed" << std::endl;
//             return -1;
//         }
        
//         if(!imu.get_raw_gyro(gyro))
//         {
//             std::cerr << "get gyro failed" << std::endl;
//             return -1;
//         }

//         if(!imu.get_raw_mag(mag))
//         {
//             std::cerr << "get raw mag failed" << std::endl;
//             return -1;
//         }


//         std::this_thread::sleep_for(std::chrono::milliseconds(10));

        
//         count++;

//         if(count % 100 == 0)
//         {
//             std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//             std::cout << "freq: " << 100.0 * 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << "Hz" << std::endl;
//             t1 = t2;
//         }

//         std::cout << "\033c";
//         std::cout << "version: " << version << std::endl;

//         std::cout << "\nacc:\n";
//         std::cout << std::showpos << "x:" << std::left << std::setw(10) << std::setprecision(6) << (double)acc[0];
//         std::cout << "  y:" << (double)acc[1];
//         std::cout << "  z:" << (double)acc[2] << std::endl;

//         std::cout << "\nmag:\n";
//         std::cout << std::showpos << "x:" << std::left << std::setw(10) << std::setprecision(6) << (double)mag[0];
//         std::cout << "  y:" << (double)mag[1];
//         std::cout << "  z:" << (double)mag[2] << std::endl;
        
//         std::cout << "\ngyro:\n";
//         std::cout << std::showpos << "x:" << std::left << std::setw(10) << std::setprecision(6) << (double)gyro[0];
//         std::cout << "  y:" << (double)gyro[1];
//         std::cout << "  z:" << (double)gyro[2] << std::endl;

//         std::cout << "\neuler:\n";
//         std::cout << std::showpos << "roll:" << std::left << std::setw(10) << std::setprecision(6) << (double)euler[0];
//         std::cout << "  pitch:" << (double)euler[1];
//         std::cout << "  yaw:" << (double)euler[2] << std::endl;

//         if(count == 1000000)
//             break;
//         // break;
//     }

//     return 0;
// }








#include "Message_syncer.h"


int main()
{
    CAN_PARAMETERS can_params;
    can_params.interface_name = "can0";
    can_params.bitrate = 1000000;
    can_params.enable_canfd = false;

    Messenger host(can_params, 95);

    std::stringstream can_logger;
    if(!host.activate(can_logger))
    {
        std::cerr << can_logger.str() << std::endl;
        return -1;
    }

    host.set_control_state(4096, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    host.deactivate(can_logger);

    return 0;

}