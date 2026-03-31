#include "points_generator.h"
#include "predictor.h"
#include <chrono>
#include <iostream>
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



int main()
{
    CAN can("can0", 500000, false, 2000000);
    std::stringstream can_logger;

    if(!can.activate(can_logger))
    {
        std::cerr << can_logger.str() << std::endl;
    }

    can_logger.str("");
    can_logger.clear();

    uint8_t msg_len = 8;
    uint64_t msg = 0x1234567890123456;
    uint32_t msg_id = 0x123;
    if(!can.send(msg_id, msg_len, &msg, can_logger))
    {
        std::cerr << can_logger.str() << std::endl;
    }
    else{
        std::cout << "msg sent successfully" << std::endl;
    }


    return 0;
}