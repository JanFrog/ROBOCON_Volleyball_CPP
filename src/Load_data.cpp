// socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


#include "dataLoader.h"
#include "predictor.h"
#include "RLSCell.h"





int main()
{
    data_Loader bin_loader(50, 5);
    bin_loader.load_file("/home/nvidia/share/Realtime_data/2026-05-02_12-22-03/data_editted.bin", 1024, 1280, 3);

    



    // ===================== Socket =====================
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in destAddr{};
    if (sockfd < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
    }
    else{
        destAddr.sin_family = AF_INET;          // IPv4
        destAddr.sin_port = htons(12345);        // port
        destAddr.sin_addr.s_addr = inet_addr("192.168.137.1");
    }
    std::stringstream ss_1, ss_2;
    std::string msg_1, msg_2;








    Eigen::Matrix3d pos_mtx, MTX;  

    // pitch = 18度
    pos_mtx << cos(M_PI / 10), 0.0, -sin(M_PI / 10),
                0.0, 1.0, 0.0,
                sin(M_PI / 10), 0.0, cos(M_PI / 10);

    MTX << 1.31527123e+03,  0.00000000e+00,  5.82870287e+02,
            0.00000000e+00, 1.31458950e+03,  5.74049534e+02,
            0.00000000e+00, 0.00000000e+00,  1.00000000e+00;
    
    // 配置Predictor参数
    Predictor_Params predictor_params;

    predictor_params.ukf_params.mass = 0.284;
    predictor_params.ukf_params.drag_coefficient = 0.0104;
    predictor_params.ukf_params.sigma_Q = 0.3;
    predictor_params.ukf_params.sigma_R = 0.5;
    predictor_params.ukf_params.alpha = 0.9;
    predictor_params.ukf_params.beta = 3;
    predictor_params.ukf_params.kappa = 1.7;
    predictor_params.ukf_params.g = 9.81;

    Predictor predictor(predictor_params);
    
    


    RLSLine rls(1, 1e6, 10);

    
    
    
    frame_data_mono frame;
    cv::Mat frame_image;

    cv::namedWindow("frame", cv::WINDOW_NORMAL);
    cv::moveWindow("frame", 0, 0);


    // bbox
    double left, top, width, height;
    
    // 初始化临时变量
    Eigen::Vector3d observed_point(3);
    Eigen::VectorXd state(6);
    state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(6, 6) * 10000000;
    uint64_t last_timestamp;
    double dt;
    bool is_first = true;
    double rls_a, rls_b;

    std::vector<Eigen::Vector3d> trajectory;
    
    
    while(bin_loader.get_single_frame(frame))
    {
        if(is_first)
        {
            last_timestamp = frame.timestamp_ns - 0.01;
            is_first = false;
        }



        cv::resize(frame.image, frame_image, cv::Size(640, 512));


        // 有检测目标
        if(frame.detected == true)
        {

            std::tie(left, top, width, height, std::ignore) = frame.bboxes;
            cv::rectangle(frame_image, cv::Point(left / 2, top / 2), cv::Point((left + width) / 2, (top + height) / 2), cv::Scalar(0, 255, 0), 1);

            // // 定位目标
            // if(predictor.locate(left, top, width, height, observed_point))
            // {
            //     if(observed_point(2) > 0.0)
            //     {
            //         dt = (double)(frame.timestamp_ns - last_timestamp) / 1e9;
            //         // std::cout << "dt = " << dt << std::endl;
                    
            //         if(predictor.update(state, P, observed_point, dt))
            //         {
            //             // 发送原始观测消息
            //             ss_1.str("");
            //             ss_1.clear();
            //             ss_1 << "0 " << state(0) << " " << state(1) << " " << state(2) << " y_ 20";
            //             // ss_1 << "0 " << observed_point(0) << " " << observed_point(1) << " " << observed_point(2) << " y_ 0.01";
            //             msg_1 = ss_1.str();
            //             ssize_t sentBytes_1 = sendto(sockfd, msg_1.c_str(), msg_1.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
                        
            //             if (sentBytes_1 < 0) {
            //                 std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
            //             }



            //             // RLS回归
            //             rls.UpdateLineParams(state(0), state(1));
            //             if(rls.GetLineParams(rls_a, rls_b))
            //             {
            //                 ss_1.str("");
            //                 ss_1.clear();
            //                 ss_1 << "1 " << 0 << " " << rls_b << " 0 " << -(rls_b / rls_a) << " " << 0 << " 0 pu 0.050";

            //                 msg_1 = ss_1.str();
            //                 ssize_t sentBytes_1 = sendto(sockfd, msg_1.c_str(), msg_1.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
                            
            //                 if (sentBytes_1 < 0) {
            //                     std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
            //                 }
            //             }



            //             // 发送消息
            //             ss_1.str("");
            //             ss_1.clear();
            //             ss_1 << "0 " << observed_point(0) << " " << observed_point(1) << " " << observed_point(2) << " g_ 20";
            //             msg_1 = ss_1.str();

            //             sentBytes_1 = sendto(sockfd, msg_1.c_str(), msg_1.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
                        
            //             if (sentBytes_1 < 0) {
            //                 std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
            //             } 



            //             // 生成轨迹
            //             if(predictor.trajectory_generate(state, 0.01, 0, trajectory))
            //             {
            //                 // 绘制轨迹
            //                 for(int i = 0; i < trajectory.size(); i++)
            //                 {
            //                     ss_2.str("");
            //                     ss_2.clear();
            //                     ss_2 << "0 " << trajectory[i](0) << " " << trajectory[i](1) << " " << trajectory[i](2) << " or 0.050";
            //                     msg_2 = ss_2.str();
            //                     ssize_t sentBytes_2 = sendto(sockfd, msg_2.c_str(), msg_2.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
            //                     if (sentBytes_2 < 0) {
            //                         std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
            //                     }
            //                 }
            //             }
            //             else
            //             {
            //                 std::cout << "Failed to generate trajectory" << std::endl;
            //             }
            //         }
            //     }


                
            //     last_timestamp = frame.timestamp_ns;
            // }
        }

        cv::imshow("frame", frame_image);
        cv::waitKey(120);
    }

    cv::destroyAllWindows();
}