#pragma once

#include <algorithm>
#include <deque>
#include "filter.h"
#include "frame_data.h"
#include "locator_2d.h"

class Predictor: public UKF, public Locator
{ 
public:
    Predictor(
        int img_width = 1280,                           //帧宽(pixels)
        int img_height = 1024,                          //帧高(pixels)
        float radius = 0.106,                           //目标半径(m)
        Eigen::Matrix3d mtx = Eigen::Matrix3d::Zero(),  //相机内参矩阵
        float angel_width = M_PI/3,                     //弧度制(rad)
        float angel_height = M_PI/3,                    //弧度制(rad)
        Eigen::Matrix3d trans_mtx = Eigen::Matrix3d::Identity(),


        double drag_coefficient = 0.014,    //阻力系数(由 f = k * v² 定义)
        double g = 9.8,                     //重力加速度(m/s²)
        double mass = 0.284,                //质量(kg)
        double sigma_R = 0.1,               //
        double sigma_Q = 0.3,

        double alpha = 0.9,
        double beta = 3,
        double kappa = 1.7,


        int que_size = 10,
        double target_height = 0,
        double threshold_height = 0.4,
        int timeout_count = 10);

    bool push_get_legacy(   const frame_data& detected_info,
                            Eigen::Vector3d& car_displacement,
                            Eigen::VectorXd& result,
                            Eigen::VectorXd& filtered_point,
                            bool get_filtered_point);

    bool push_get_legacy(   const Eigen::Vector3d& obs_point,
                            const double dt,
                            Eigen::VectorXd& result,
                            Eigen::VectorXd& filtered_point,
                            bool get_filtered_point);

    // bool push_get_tmp();

private:
    int que_size_;
    double target_height_;
    double threshold_height_;
    int sequence_count_;
    int timeout_count_;
    double last_tick;
    std::deque<Eigen::VectorXd> result_que;

    Eigen::MatrixXd P_;
    Eigen::VectorXd state_;
    Eigen::Vector3d obs_;
    Eigen::VectorXd target_tmp_;


    bool __target_count_legacy(const Eigen::VectorXd& state, Eigen::VectorXd& result);
};