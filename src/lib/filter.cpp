#include "filter.h"




UKF::UKF(float drag_coefficient, float g, float mass, float sigma_R, float sigma_Q, float alpha, float beta, float kappa)
{
    K_              = drag_coefficient;
    g_              = g;
    mass_           = mass;
    sigma_Q_        = sigma_Q;
    R_              = Eigen::MatrixXd::Identity(m_, m_) * (sigma_R * sigma_R);
    character_vel_  = sqrt(mass_ * g_ / K_);
    traffic_        = sqrt(g_ * K_ / mass_);

    lambda_         = alpha * alpha * (n_ + kappa) - n_;
    Weight_M        = Eigen::VectorXd::Constant(2 * n_ + 1, 0.5 / (n_ + lambda_));
    Weight_M(0)     = lambda_ / (n_ + lambda_);
    Weight_C        = Weight_M;
    Weight_C(0)    += (1 - alpha * alpha + beta);
}


// 计算速度(初速度为正)
double UKF::_count_vel_posi_vel(double vel, double dt){

    return character_vel_ * tan(atan(vel / character_vel_) - traffic_ * dt);
}

// 计算速度(初速度为负)
double UKF::_count_vel_nega_vel(double vel, double dt){

    if ((vel + character_vel_) != 0){
        return character_vel_ * ((2 / (1 + ((2 * character_vel_ / (vel + character_vel_) - 1) * exp(2 * dt * traffic_)))) - 1);
    }
    else{
        return vel;
    }
}


// 计算位置(初速度为正)
double UKF::_count_pos_posi_vel(double vel, double pos, double dt){

    return (mass_ / K_) * log(cos(atan(vel / character_vel_) - (traffic_ * dt)) / cos(atan(vel / character_vel_))) + pos;
}


// 计算位置(初速度为负)
double UKF::_count_pos_nega_vel(double vel, double pos, double dt){

    if ((vel + character_vel_) != 0){
        return (mass_ / K_) * log((2 * character_vel_ / (vel + character_vel_)) / (1 + (2 * character_vel_ / (vel + character_vel_) - 1) * exp(2 * dt * traffic_))) + pos + (dt * character_vel_);
    }
    else{
        return character_vel_ * dt + pos;
    }
}



// 单维度状态转移(带加速度)
std::tuple<double, double> UKF::_count_transition_with_acc(double vel, double pos, double dt){

    double new_vel, new_pos;

    if (vel > 0){

        double threshold = atan(vel / character_vel_) / traffic_;

        if(dt <= threshold){
            new_vel = _count_vel_posi_vel(vel, dt);
            new_pos = _count_pos_posi_vel(vel, pos, dt);
        }
        else{
            new_vel = _count_vel_nega_vel(0, dt - threshold);
            new_pos = _count_pos_nega_vel(0, _count_pos_posi_vel(vel, pos, threshold), dt - threshold);
        }
    }
    else{
        new_vel = _count_vel_nega_vel(vel, dt);
        new_pos = _count_pos_nega_vel(vel, pos, dt);
    }

    return std::make_tuple(new_pos, new_vel);
}



// 单维度状态转移(不带加速度)
std::tuple<double, double> UKF::_count_transition(double vel, double pos, double dt){
    
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



// 状态转移函数
Eigen::MatrixXd UKF::__transition(const Eigen::MatrixXd& previous_state, double dt){  //状态转移函数

    Eigen::MatrixXd result(n_, previous_state.cols());

    for(Eigen::Index i = 0; i < previous_state.cols(); i++){
        const double pos_x = previous_state(0, i);
        const double pos_y = previous_state(1, i);
        const double pos_z = previous_state(2, i);
        const double vel_x = previous_state(3, i);
        const double vel_y = previous_state(4, i);
        const double vel_z = previous_state(5, i);

        auto [new_pos_x, new_vel_x] = _count_transition(vel_x, pos_x, dt);
        auto [new_pos_y, new_vel_y] = _count_transition(vel_y, pos_y, dt);
        auto [new_pos_z, new_vel_z] = _count_transition_with_acc(vel_z, pos_z, dt);

        result.col(i) << new_pos_x, new_pos_y, new_pos_z, new_vel_x, new_vel_y, new_vel_z;
    }

    return result;
}



// 生成sigma点
Eigen::MatrixXd UKF::__sigma_point_generate(const Eigen::VectorXd& point, const Eigen::MatrixXd& S){
    
    Eigen::MatrixXd result(n_, 2 * n_ + 1);
    result.col(0) = point;

    for(Eigen::Index i = 0; i < n_; i++){
        result.col(i + 1) = point + S.col(i);
        result.col(n_ + i + 1) = point - S.col(i);
    }

    return result;
}




// 生成Q矩阵
Eigen::MatrixXd UKF::__make_Q(double dt){

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n_, n_);
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    Q << 
        dt3 / 3.0,      0.0,            0.0,            dt2 / 2.0,      0.0,            0.0,
        0.0,            dt3 / 3.0,      0.0,            0.0,            dt2 / 2.0,      0.0,
        0.0,            0.0,            dt3 / 3.0,      0.0,            0.0,            dt2 / 2.0,
        dt2 / 2.0,      0.0,            0.0,            dt,             0.0,            0.0,
        0.0,            dt2 / 2.0,      0.0,            0.0,            dt,             0.0,
        0.0,            0.0,            dt2 / 2.0,      0.0,            0.0,            dt;

    Q *= (sigma_Q_ * sigma_Q_);

    return Q;
}




// 观测函数
Eigen::MatrixXd UKF::__observe(const Eigen::MatrixXd& points){
    Eigen::MatrixXd points_obs = points.topRows(m_);
    return points_obs;
}



// Eigen::MatrixXd UKF::__cov(const Eigen::MatrixXd& X){       //np.cov(X, rowvar=True)
//     Eigen::MatrixXd Xc = X.rowwise() - X.rowwise().mean();  // 中心化
//     return (Xc * Xc.transpose()) / (X.cols() - 1.0);        // 无偏估计
// }




// 迭代更新
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> UKF::forward(const Eigen::VectorXd& state_prev, const Eigen::MatrixXd& P_prev, Eigen::VectorXd observed_point, double dt)
{ 

    Eigen::MatrixXd scaled_P = (n_+lambda_) * P_prev;

    Eigen::LLT<Eigen::MatrixXd> llt(scaled_P.selfadjointView<Eigen::Lower>());   //Cholesky 分解
    

    // 检查分解是否成功（非正定矩阵会失败）
    if (llt.info() != Eigen::Success) {
        throw std::runtime_error("Cholesky分解失败: 矩阵非正定");
    }
    
    Eigen::MatrixXd S = llt.matrixL();



    Eigen::MatrixXd sigma_predict = __transition(__sigma_point_generate(state_prev, S), dt);
    Eigen::MatrixXd sigma_predict_obs = __observe(sigma_predict);

    Eigen::MatrixXd predict_point = sigma_predict * Weight_M;
    Eigen::MatrixXd predict_point_obs = __observe(predict_point);

    Eigen::VectorXd Res = observed_point - predict_point_obs;

    Eigen::MatrixXd predict_P = Eigen::MatrixXd::Zero(n_, n_);
    Eigen::MatrixXd Cov_zz = Eigen::MatrixXd::Zero(m_, m_);
    Eigen::MatrixXd Cov_xz = Eigen::MatrixXd::Zero(n_, m_);


    for(short i = 0; i < 2 * n_ + 1; i++){

        Eigen::VectorXd dx = sigma_predict.col(i) - predict_point;

        predict_P.noalias() += Weight_C(i) * (dx * dx.transpose());

        Cov_zz.noalias() += Weight_C(i) * ((sigma_predict_obs.col(i) - predict_point_obs) * (sigma_predict_obs.col(i) - predict_point_obs).transpose());

        Cov_xz.noalias() += Weight_C(i) * (dx * (sigma_predict_obs.col(i) - predict_point_obs).transpose());
    }

    
    predict_P.noalias() += __make_Q(dt);
    Cov_zz.noalias() += R_;


    // Eigen::MatrixXd K = Cov_xz * (Cov_zz.inverse());
    Eigen::MatrixXd K = Cov_xz * Cov_zz.ldlt().solve(Eigen::MatrixXd::Identity(m_, m_));

    
    Eigen::VectorXd state_updated = predict_point + (K * Res);
    Eigen::MatrixXd P_updated = predict_P - (K * Cov_zz * K.transpose());

    return std::make_tuple(state_updated, P_updated);
}