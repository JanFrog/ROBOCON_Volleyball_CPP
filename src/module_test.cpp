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

#include "CAN.h"


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
//     Predictor pd;

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
        
//         if(pd.push_get_legacy(state.head<3>(), dt, result, state_filtered, true)){

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
//         // std::cout << "time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " μs" << std::endl;
        
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
//     CAN can("can0", 500000, true, 2000000);
//     std::stringstream can_logger;

//     if(!can.activate(can_logger))
//     {
//         std::cerr << can_logger.str() << std::endl;
//         return -1;
//     }
//     can_logger.str("");
//     can_logger.clear();







//     // 准备CAN帧
//     uint8_t* msg_ptr = new uint8_t[64];
//     uint64_t msg;

//     uint8_t msg_len = 64;
//     for(int i = 0; i < msg_len; i++){
//         msg_ptr[i] = i;
//     }
//     uint32_t msg_id = 0x310;







//     // 发送CAN帧
//     int try_count = 0;
//     int success_count = 0;

//     std::chrono::time_point t1 = std::chrono::high_resolution_clock::now();
//     for(int i = 0; i < 2000; i++)
//     {
//         msg = i;

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

//         std::this_thread::sleep_for(std::chrono::duration<float>(0.005));
//     } 



//     delete[] msg_ptr;
//     return 0;
// }





int main()
{



    // 初始化CAN接口
    CAN can("can0", 500000, false, 2000000);

    std::stringstream can_logger;

    if(!can.activate(can_logger))
    {
        std::cerr << can_logger.str() << std::endl;
        return -1;
    }
    can_logger.str("");
    can_logger.clear();








    canfd_frame receive_frame;
    int data_len = 0;

    while(!can.receive(receive_frame, can_logger))
    {
        std::this_thread::sleep_for(std::chrono::duration<float>(0.1));
        std::cout << can_logger.str() << std::endl;
        
        can_logger.str("");
        can_logger.clear();
    }

    data_len = receive_frame.len;

    std::vector<uint8_t> msg_ptr(data_len);



    // 反转数据字节序
    for(int i = 0; i < data_len; i++)
    {
        msg_ptr[i] = (uint8_t)receive_frame.data[data_len - i - 1];
    }


    std::cout << "receive_frame.can_id: " << std::hex << receive_frame.can_id << std::endl;
    std::cout << "receive_frame.can_dlc: " << std::dec << data_len << std::endl;
    std::cout << "receive_frame.data: ";

    for(int i = 0; i < data_len; i++)
    {
        std::cout << "0x" << std::hex << static_cast<int>(msg_ptr[i]) << ' ';
    }

    std::cout << std::endl;


    return 0;
}
