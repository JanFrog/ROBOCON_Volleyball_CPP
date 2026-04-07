#include "predictor.h"

Predictor::Predictor(const Predictor_Params& params): UKF(params.ukf_params), Locator(params.locator_params)
{
    que_size_ = params.que_size;
    target_height_ = params.target_height;
    threshold_height_ = params.threshold_height;
    sequence_count_ = 0;
    timeout_count_ = params.timeout_count;

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



bool Predictor::push_get_legacy(const frame_data_mono& detected_info,
                                Eigen::Vector3d& car_displacement,
                                Eigen::VectorXd& result,
                                Eigen::VectorXd* filtered_point)

{
        

    if (!detected_info.detected){
        if (sequence_count_ > 0) sequence_count_--;
        return false;
    }
    


    float left, right, top, bottom;
    std::tie(left, right, top, bottom, std::ignore) = detected_info.bboxes;
    Eigen::Vector3d obs_;

    if(!locate(left, right, top, bottom, obs_)){
        return false;
    }




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

    if (filtered_point){
        *filtered_point = state_;
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
bool Predictor::push_get_legacy(const Eigen::Vector3d& obs_point,
                                const double dt,
                                Eigen::VectorXd& result,
                                Eigen::VectorXd* filtered_point)
{

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

    if (filtered_point){
        *filtered_point = state_;
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