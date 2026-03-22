#include <iostream>
#include <cstring>
// #include <libsocketcan.h>
// #include <linux/can/raw.h> 
// #include <sys/socket.h>        // 套接字
// #include <unistd.h>            // read/write/close
// #include <net/if.h>
// #include <sys/ioctl.h> 

#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close

#include "locator_2d.h"
#include <chrono>
#include <thread>




int main(){
    Eigen::Matrix3d MTX; 
    MTX << 1.31527123e+03,  0.00000000e+00,  5.82870287e+02,
            0.00000000e+00, 1.31458950e+03,  5.74049534e+02,
            0.00000000e+00, 0.00000000e+00,  1.00000000e+00;
    Locator locator(1280, 1024, 0.106, MTX);

    std::cout << locator.locate(590, 512, 100, 100).transpose();
}