#pragma once

#include <Eigen/Dense>
#include <tuple>
#include <vector>
#include <cmath>
#include <iostream>




struct UKF_Params{
    float drag_coefficient = 0.014;
    float g = 9.8;
    float mass = 0.284;
    float sigma_R = 0.1;
    float sigma_Q = 0.01;
    float alpha = 0.9;
    float beta = 3;
    float kappa = 1.7;
};




class UKF {
public:

    UKF(const UKF_Params& params);
    bool update(Eigen::VectorXd& state_prev, Eigen::MatrixXd& P_prev, Eigen::VectorXd observed_point, double dt);
    bool trajectory_generate(const Eigen::VectorXd& state, double Period, double target_height, std::vector<Eigen::Vector3d>& trajectory);


protected:

    float K_;
    float mass_;
    float g_;
    const short n_ = 6; // 状态维度
    const short m_ = 3; // 观测维度
    
    double character_vel_;
    double traffic_;


    double _count_vel_posi_vel(double vel, double dt);
    double _count_vel_nega_vel(double vel, double dt);
    double _count_pos_posi_vel(double vel, double pos, double dt);
    double _count_pos_nega_vel(double vel, double pos, double dt);
    
    std::tuple<double, double> _count_transition_with_acc(double vel, double pos, double dt);
    std::tuple<double, double> _count_transition(double vel, double pos, double dt);




private:

    float sigma_Q_;
    Eigen::MatrixXd R_;

    double lambda_;
    Eigen::VectorXd Weight_M;
    Eigen::VectorXd Weight_C;

    Eigen::MatrixXd __sigma_point_generate(const Eigen::VectorXd& point, const Eigen::MatrixXd& S);
    Eigen::MatrixXd __make_Q(double dt);
    Eigen::MatrixXd __observe(const Eigen::MatrixXd& x);

    Eigen::MatrixXd __transition(const Eigen::MatrixXd& previous_state, double dt);
    // Eigen::MatrixXd __cov(const Eigen::MatrixXd& X);



};