#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include "filter.h"
#include "frame_data.h"
#include "locator_2d.h"
#include "RLSCell.h"



struct Predictor_Params{
    
    UKF_Params ukf_params;  // 滤波参数
    RLS_Params rls_params;  // 递归最小二乘参数

    // 滤波三维求解
    double target_height = 0;       // 落点所在相对高度
    double threshold_height = 0.4;  // 开始预测阈值高度

    // RLS求近似解
    double target_dist = 0.0;

    // 加权参数
    int iterating_threshold = 0;
    double switch_speed = 0.5;
};




class Predictor: public UKF, public RLSLine
{ 
public:
    Predictor(const Predictor_Params& params);

    // 预测落点
    bool push_get_legacy(const Eigen::Vector3d& obs_point,
                         const double dt,
                         Eigen::Vector2d& result);

    
    void reset();
    void reset(double target_dist);
    // bool push_get_tmp();



private:
    
    // for filter
    Eigen::MatrixXd P_;
    Eigen::VectorXd state_;
    Eigen::Vector2d target_filter_tmp_;
    Eigen::Vector2d target_rls_tmp_;

    // for legacy predictor
    double target_height_;
    double threshold_height_;
    int sequence_count_;

    // for RLS predictor
    double target_distance_;
    double rls_a_, rls_b_;

    // weighting (加权)
    int iterating_threshold_;
    double switch_speed_;
    
    
    double __diy_sigmoid(int iterate_seq);
    void __Weighting(const Eigen::Vector2d& from_legacy, const Eigen::Vector2d& from_rls, int iterate_seq, Eigen::Vector2d& result);
    bool __target_count_legacy(const Eigen::VectorXd& state, Eigen::Vector2d& result);
};