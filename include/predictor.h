#pragma once

#include <algorithm>
#include <deque>
#include "filter.h"
#include "frame_data.h"
#include "locator_2d.h"
#include <vector>





struct Predictor_Params{
    UKF_Params ukf_params;
    Locator_Params locator_params;
    int que_size = 10;
    double target_height = 0;
    double threshold_height = 0.4;
    int timeout_count = 10;
};






class Predictor: public UKF, public Locator
{ 
public:
    Predictor(const Predictor_Params& params);

    bool push_get_legacy(   const frame_data_mono& detected_info,
                            Eigen::Vector3d& car_displacement,
                            Eigen::VectorXd& result,
                            Eigen::VectorXd* filtered_point = nullptr);

    bool push_get_legacy(   const Eigen::Vector3d& obs_point,
                            const double dt,
                            Eigen::VectorXd& result,
                            Eigen::VectorXd* filtered_point = nullptr);

    
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

    bool __make_trajectory(std::vector<Eigen::Vector3d>* trajectory);
    bool __target_count_legacy(const Eigen::VectorXd& state, Eigen::VectorXd& result);
};