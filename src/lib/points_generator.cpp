#include "points_generator.h"






PointsGenerator::PointsGenerator(double K = 0.014, double mass = 0.284, double g = 9.8){
    K_ = K;
    mass_ = mass;
    g_ = g;
    state_ = Eigen::VectorXd::Zero(6);

    // dist_a      = std::uniform_real_distribution<double>(-4, 4);
    // dist_b      = std::uniform_real_distribution<double>(-4, 4);
    // dist_c      = std::normal_distribution<double>(4, 2);
    // dist_noise  = std::normal_distribution<double>(0, noise_sigma);


    character_vel_ = sqrt(mass * g / K);
    traffic_ = sqrt(g * K / mass);
}



double PointsGenerator::__count_vel_posi_vel(double vel, double dt){

    return character_vel_ * tan(atan(vel / character_vel_) - traffic_ * dt);
}



double PointsGenerator::__count_vel_nega_vel(double vel, double dt){

    if ((vel + character_vel_) != 0){
        return character_vel_ * ((2 / (1 + ((2 * character_vel_ / (vel + character_vel_) - 1) * exp(2 * dt * traffic_)))) - 1);
    }
    else{
        return vel;
    }
}



double PointsGenerator::__count_pos_posi_vel(double vel, double pos, double dt){

    return (mass_ / K_) * log(cos(atan(vel / character_vel_) - (traffic_ * dt)) / cos(atan(vel / character_vel_))) + pos;
}



double PointsGenerator::__count_pos_nega_vel(double vel, double pos, double dt){

    if ((vel + character_vel_) != 0){
        return (mass_ / K_) * log((2 * character_vel_ / (vel + character_vel_)) / (1 + (2 * character_vel_ / (vel + character_vel_) - 1) * exp(2 * dt * traffic_))) + pos + (dt * character_vel_);
    }
    else{
        return character_vel_ * dt + pos;
    }
}



std::tuple<double, double> PointsGenerator::__count_transition_with_acc(double vel, double pos, double dt){

    double new_vel, new_pos;

    if (vel > 0){

        double threshold = atan(vel / character_vel_) / traffic_;

        if(dt <= threshold){
            new_vel = __count_vel_posi_vel(vel, dt);
            new_pos = __count_pos_posi_vel(vel, pos, dt);
        }
        else{
            new_vel = __count_vel_nega_vel(0, dt - threshold);
            new_pos = __count_pos_nega_vel(0, __count_pos_posi_vel(vel, pos, threshold), dt - threshold);
        }
    }
    else{
        new_vel = __count_vel_nega_vel(vel, dt);
        new_pos = __count_pos_nega_vel(vel, pos, dt);
    }

    return std::make_tuple(new_pos, new_vel);
}



std::tuple<double, double> PointsGenerator::__count_transition(double vel, double pos, double dt){
    
    if (vel != 0){

        float k = K_ * (vel > 0 ? 1 : -1);
        double new_pos = pos + (mass_ / k) * log((k * dt * vel / mass_) + 1);
        double new_vel = vel / ((k * dt * vel) / mass_ + 1);
        return std::make_tuple(new_pos, new_vel);
    }
    else{
        return std::make_tuple(pos, vel);
    }
}



Eigen::MatrixXd PointsGenerator::__transition(const Eigen::MatrixXd& previous_state, double dt){  //状态转移函数

    Eigen::MatrixXd result(n_, previous_state.cols());

    for(Eigen::Index i = 0; i < previous_state.cols(); i++){
        const double pos_x = previous_state(0, i);
        const double pos_y = previous_state(1, i);
        const double pos_z = previous_state(2, i);
        const double vel_x = previous_state(3, i);
        const double vel_y = previous_state(4, i);
        const double vel_z = previous_state(5, i);

        auto [new_pos_x, new_vel_x] = __count_transition(vel_x, pos_x, dt);
        auto [new_pos_y, new_vel_y] = __count_transition(vel_y, pos_y, dt);
        auto [new_pos_z, new_vel_z] = __count_transition_with_acc(vel_z, pos_z, dt);

        result.col(i) << new_pos_x, new_pos_y, new_pos_z, new_vel_x, new_vel_y, new_vel_z;
    }

    return result;
}



Eigen::VectorXd PointsGenerator::set_state(double x, double y, double z, double vx, double vy, double vz){ 
    
    state_ << x, y, z, vx, vy, vz;
    return state_;
}



Eigen::VectorXd PointsGenerator::set_state_randomly(){
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> dist_a(-4, 4);
    std::uniform_real_distribution<double> dist_b(-4, 4);
    std::normal_distribution<double> dist_c(4, 2);

    state_ << dist_a(gen), dist_a(gen), dist_b(gen), dist_a(gen), dist_a(gen), dist_c(gen);
    return state_;
}



Eigen::VectorXd PointsGenerator::get_state(bool noise = false, float sigma_noise = 0.1, bool obs_style = false) const{

    if(obs_style){
        if(noise){
            
            std::random_device rd;
            std::mt19937_64 gen(rd());
            std::normal_distribution<double> dist_noise(0, sigma_noise);
            Eigen::VectorXd noise(3);
            
            noise << dist_noise(gen), dist_noise(gen), dist_noise(gen);

            return state_.head(3) + noise;
        }
        return state_.head(3);
    }
    else{
        if(noise){
            
            std::random_device rd;
            std::mt19937_64 gen(rd());
            std::normal_distribution<double> dist_noise(0, sigma_noise);
            Eigen::VectorXd noise(6);
            
            noise << dist_noise(gen), dist_noise(gen), dist_noise(gen),
                    dist_noise(gen), dist_noise(gen), dist_noise(gen);
                    
            return state_ + noise;
        }
        return state_;
    }
}



Eigen::VectorXd PointsGenerator::update(double dt){
    state_ = __transition(state_, dt);
    return state_;
}
