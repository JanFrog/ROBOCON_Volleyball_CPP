#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <tuple>


class PointsGenerator {
public:
    PointsGenerator(double K, double mass, double g);

    Eigen::VectorXd set_state(double x, double y, double z, double vx, double vy, double vz);
    Eigen::VectorXd set_state_randomly();
    Eigen::VectorXd get_state(bool noise, float sigma_noise, bool obs_style) const;
    Eigen::VectorXd update(double dt);

private:
    double K_;
    double mass_;
    double g_;
    Eigen::VectorXd state_;

    double character_vel_;
    double traffic_;

    const short n_ = 6;
    const short m_ = 3;
    


    double __count_vel_posi_vel(double vel, double dt);
    double __count_vel_nega_vel(double vel, double dt);
    double __count_pos_posi_vel(double vel, double pos, double dt);
    double __count_pos_nega_vel(double vel, double pos, double dt);
    
    std::tuple<double, double> __count_transition_with_acc(double vel, double pos, double dt);
    std::tuple<double, double> __count_transition(double vel, double pos, double dt);

    Eigen::MatrixXd __transition(const Eigen::MatrixXd& previous_state, double dt);
};