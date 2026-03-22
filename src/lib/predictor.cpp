#include "predictor.h"

Predictor::Predictor(
    int img_width,              //帧宽(pixels)
    int img_height,             //帧高(pixels)
    float radius,               //目标半径(m)
    Eigen::Matrix3d mtx,        //相机内参矩阵
    float angel_width,          //弧度制(rad)
    float angel_height,         //弧度制(rad)
    Eigen::Matrix3d trans_mtx,


    double drag_coefficient,    //阻力系数(由 f = k * v² 定义)
    double g,                   //重力加速度(m/s²)
    double mass,                //质量(kg)
    double sigma_R,             //
    double sigma_Q,

    double alpha,
    double beta,
    double kappa,


    int que_size,
    double target_height,
    double threshold_height,
    int timeout_count):

        UKF(drag_coefficient, g, mass, sigma_R, sigma_Q, alpha, beta, kappa),
        Locator(img_width, img_height, radius, mtx, angel_width, angel_height, trans_mtx){



    que_size_ = que_size;
    target_height_ = target_height;
    threshold_height_ = threshold_height;
    sequence_count_ = 0;
    timeout_count_ = timeout_count;

    P_ = Eigen::MatrixXd::Zero(6, 6);
    state_ = Eigen::VectorXd::Zero(6);
    obs_ = Eigen::Vector3d::Zero();
    target_tmp_ = Eigen::Vector2d::Zero();
}



bool Predictor::__target_count_legacy(const Eigen::VectorXd& state, Eigen::VectorXd& result){

    if(state(5) > 0){   //速度向上:

        double threshold = atan(state(5) / character_vel_) / traffic_;
        double top_pos = _count_pos_posi_vel(state(5), state(2), threshold);

        
        if(top_pos >= target_height_){  //最高点大于目标高度:
            double B = 2 * exp((K_ / mass_) * (top_pos - target_height_));
            double dt = threshold + (1 / traffic_) * log((B + sqrt(B * B - 4)) / 2);    //dt = 当前状态到目标高度所需要的时间

            double x, y;
            std::tie(x, std::ignore) = _count_transition(state(3), state(0), dt);
            std::tie(y, std::ignore) = _count_transition(state(4), state(1), dt);

            result << x,y;
            return true;
        }
        else{   //最高点小于目标高度:
            return false;
        }
    }

    else{   //速度向下:
        if(state(2) >= target_height_){

            double A = 2 * character_vel_ / (state(5) + character_vel_) - 1;
            double B = (A + 1) * exp((K_ / mass_) * (state(2) - target_height_));
            double dt = (1 / traffic_) * log((B + sqrt((B * B) - (4 * A))) / (2 * A));  //dt = (同上)

            double x, y;
            std::tie(x, std::ignore) = _count_transition(state(3), state(0), dt);
            std::tie(y, std::ignore) = _count_transition(state(4), state(1), dt);

            result << x,y;
            return true;
        }
        else{   //速度向下且当前高度小于目标高度:
            return false;
        }
    }
}



bool Predictor::push_get_legacy(
    const frame_data& detected_info,        //帧信息
    Eigen::Vector3d& car_displacement,      //车位移
    Eigen::VectorXd& result,                //输出
    Eigen::VectorXd& filtered_point,        //滤波后的点(如果要的话)
    bool get_filtered_point = false     ){  //获取滤波点开关
        

    if (!detected_info.detected){
        if (sequence_count_ > 0) sequence_count_--;
        return false;
    }
    


    float left, right, top, bottom;
    std::tie(left, right, top, bottom, std::ignore) = detected_info.bboxes;
    Eigen::Vector3d obs_ = (locate(left, right, top, bottom) + car_displacement);




    if(obs_(2) < target_height_){
        if (sequence_count_ > 0) sequence_count_--;
        return false;
    }


    if (sequence_count_ == 0){

        Eigen::VectorXd v(6);
        v << 1, 1, 1, 5, 5, 5;
        P_ = v.cwiseProduct(v).asDiagonal(); 
        

        state_<<obs_(0), obs_(1), obs_(2), 0, 0, 3;
        last_tick = (double)detected_info.timestamp_ns / 1e9 - 0.1;

        result_que.clear();
    }

    sequence_count_ = timeout_count_;   // 目标符合要求后重置计数

    //更新状态
    try
    {
        std::tie(state_, P_) = forward(state_, P_, obs_, ((double)detected_info.timestamp_ns / 1e9 -last_tick));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    

    last_tick = (double)detected_info.timestamp_ns / 1e9;

    if (get_filtered_point){
        filtered_point = state_;
    }

    if (__target_count_legacy(state_, target_tmp_)){
        result_que.push_back(target_tmp_);
        if (result_que.size() > que_size_){
            result_que.pop_front();
        }
    }
    else if(!result_que.empty()){
        result_que.pop_front();
    }
    
    if (!result_que.empty()){

        Eigen::VectorXd result_tmp = Eigen::VectorXd::Zero(2);
        double n = result_que.size();

        for(short i = 0; i < n; i++){
            result_tmp.noalias() += result_que[i];
        }

        result_tmp /= n;
        result = result_tmp;
        return true;
    }

    return false;
}





// 重载
bool Predictor::push_get_legacy(   
    const Eigen::Vector3d& obs_point,   //观测点
    const double dt,                    //delta t
    Eigen::VectorXd& result,            //结果引用
    Eigen::VectorXd& filtered_point,    //滤波点引用
    bool get_filtered_point){

    if(obs_point(2) < target_height_){
        if(sequence_count_ > 0) sequence_count_--;
        return false;
    }

    if(sequence_count_ == 0){

        Eigen::VectorXd v(6);
        v << 1, 1, 1, 5, 5, 5;
        P_ = v.cwiseProduct(v).asDiagonal();

        state_<<obs_point(0), obs_point(1), obs_point(2), 0, 0, 3;
        result_que.clear();
    }


    sequence_count_ = timeout_count_;

    // forward
    try
    {
        std::tie(state_, P_) = forward(state_, P_, obs_point, dt);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    if (get_filtered_point){
        filtered_point = state_;
    }

    if (__target_count_legacy(state_, target_tmp_)){
        result_que.push_back(target_tmp_);
        if (result_que.size() > que_size_){
            result_que.pop_front();
        }
    }
    else if(!result_que.empty()){
        result_que.pop_front();
    }

    if(!result_que.empty()){

        Eigen::VectorXd result_tmp = Eigen::VectorXd::Zero(2);
        double n = result_que.size();

        for(short i = 0; i < n; i++){
            result_tmp.noalias() += result_que[i];
        }

        result_tmp /= n;
        result = result_tmp;
        return true;
    }

    return false;
    
}