#include "predictor.h"




Predictor::Predictor(const Predictor_Params& params):
UKF(params.ukf_params), RLSLine(params.rls_params)
{
    this->target_height_ = params.target_height;
    this->threshold_height_ = params.threshold_height;
    this->sequence_count_ = 0;

    // 滤波参数
    Eigen::VectorXd v_tmp(6);
    v_tmp << 1, 1, 1, 5, 5, 5;
    this->P_ = v_tmp.cwiseProduct(v_tmp).asDiagonal();
    this->state_ = Eigen::VectorXd::Zero(6);

    // RLS参数
    this->target_distance_ = params.target_dist;
    this->rls_a_ = 0.0;
    this->rls_b_ = 0.0;

    // 初始化滤波结果
    this->target_filter_tmp_ = Eigen::Vector2d::Zero();
    this->target_rls_tmp_ = Eigen::Vector2d::Zero();
}




// 计算落点
bool Predictor::__target_count_legacy(const Eigen::VectorXd& state, Eigen::Vector2d& result){

    if(state(5) > 0){   //速度向上:

        double threshold = atan(state(5) / this->character_vel_) / this->traffic_;
        double top_pos = _count_pos_posi_vel(state(5), state(2), threshold);

        
        if(top_pos >= this->target_height_){  //最高点大于目标高度:
            double B = 2 * exp((this->K_ / this->mass_) * (top_pos - this->target_height_));
            double dt = threshold + (1 / this->traffic_) * log((B + sqrt(B * B - 4)) / 2);    //dt = 当前状态到目标高度所需要的时间

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
        if(state(2) >= this->target_height_){

            double A = 2 * this->character_vel_ / (state(5) + this->character_vel_) - 1;
            double B = (A + 1) * exp((this->K_ / this->mass_) * (state(2) - this->target_height_));
            double dt = (1 / this->traffic_) * log((B + sqrt((B * B) - (4 * A))) / (2 * A));  //dt = (同上)

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





// sigmoid
double Predictor::__diy_sigmoid(int iterate_seq)
{
    return (1 / (1 + exp(this->switch_speed_ * (double)(this->iterating_threshold_ - iterate_seq))));
}




// 加权求和
void Predictor::__Weighting(const Eigen::Vector2d& from_legacy, const Eigen::Vector2d& from_rls, int iterate_seq, Eigen::Vector2d& result)
{
    double scale_for_legacy_result = this->__diy_sigmoid(iterate_seq);
    result = (scale_for_legacy_result * from_legacy) + ((1 - scale_for_legacy_result) * from_rls);
}











// 预测接口
bool Predictor::push_get_legacy(const Eigen::Vector3d& obs_point,
                                const double dt,
                                Eigen::Vector2d& result)
{

    if(sequence_count_ == 0){
        this->state_<<obs_point(0), obs_point(1), obs_point(2), 0, 0, 3;
    }


    // update
    if(!UKF::update(this->state_, this->P_, obs_point, dt)){
        return false;
    }

    // 六维状态落点计算
    if (!this->__target_count_legacy(this->state_, this->target_filter_tmp_)){
        return false;
    }

    // RLS更新
    this->RLSLine::UpdateLineParams(this->state_(0), this->state_(1));
    if(this->RLSLine::GetLineParams(this->rls_a_, this->rls_b_)){
        this->target_rls_tmp_ << (this->target_distance_ - this->rls_b_) / (this->rls_a_), this->target_distance_;
        // << x, y;

        this->__Weighting(this->target_filter_tmp_, this->target_rls_tmp_, this->sequence_count_, result);
    }

    this->sequence_count_++;
    return false;
}







void Predictor::reset()
{
    this->sequence_count_ = 0;

    // 滤波参数
    Eigen::VectorXd v_tmp(6);
    v_tmp << 1, 1, 1, 5, 5, 5;
    this->P_ = v_tmp.cwiseProduct(v_tmp).asDiagonal();
    this->state_ = Eigen::VectorXd::Zero(6);

    // 初始化滤波结果
    this->target_filter_tmp_ = Eigen::Vector2d::Zero();
    this->target_rls_tmp_ = Eigen::Vector2d::Zero();

    RLSLine::Reset();
}



void Predictor::reset(double target_dist)
{
    this->target_distance_ = target_dist;
    this->reset();
    this->RLSLine::Reset();
}






// bool Predictor::push_get_legacy(const frame_data_mono& detected_info,
//                                 Eigen::Vector3d& car_displacement,
//                                 Eigen::VectorXd& result,
//                                 Eigen::VectorXd* filtered_point)

// {
        
//     // 是否有检测目标
//     if (!detected_info.detected){
//         return false;
//     }
    

//     // 提取三维坐标
//     float left, top, width, height;
//     std::tie(left, top, width, height, std::ignore) = detected_info.bboxes;
//     Eigen::Vector3d obs_;

//     if(!locate(left, top, width, height, obs_)){
//         return false;
//     }



//     // 
//     if(obs_(2) < target_height_){
//         if (sequence_count_ > 0) sequence_count_--;
//         return false;
//     }


//     if (sequence_count_ == 0){

//         Eigen::VectorXd v(6);
//         v << 1, 1, 1, 5, 5, 5;
//         P_ = v.cwiseProduct(v).asDiagonal(); 
        

//         state_<<obs_(0), obs_(1), obs_(2), 0, 0, 3;
//         last_tick = (double)detected_info.timestamp_ns / 1e9 - 0.1;

//         result_que.clear();
//     }

//     sequence_count_ = timeout_count_;   // 目标符合要求后重置计数

//     //更新状态
//     if(!UKF::update(state_, P_, obs_, ((double)detected_info.timestamp_ns / 1e9 -last_tick))){
//         return false;
//     }

    

//     last_tick = (double)detected_info.timestamp_ns / 1e9;

//     if (filtered_point){
//         *filtered_point = state_;
//     }

//     if (__target_count_legacy(state_, target_tmp_)){
//         result_que.push_back(target_tmp_);
//         if (result_que.size() > que_size_){
//             result_que.pop_front();
//         }
//     }
//     else if(!result_que.empty()){
//         result_que.pop_front();
//     }
    
//     if (!result_que.empty()){

//         Eigen::VectorXd result_tmp = Eigen::VectorXd::Zero(2);
//         double n = result_que.size();

//         for(short i = 0; i < n; i++){
//             result_tmp.noalias() += result_que[i];
//         }

//         result_tmp /= n;
//         result = result_tmp;
//         return true;
//     }

//     return false;
// }