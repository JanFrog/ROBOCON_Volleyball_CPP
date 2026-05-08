#pragma once
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <deque>
#include <string>
#include <Eigen/Dense>
#include <tuple>

#include "frame_data.h"



template<typename T>

class ExposureRecorder {


    public:
        // 获取单例实例
        static ExposureRecorder<T>& instance();
        
        // 记录曝光开始时间
        void recordExposureStart();


        void push2History(T frame);
        
        // 获取最近一次曝光开始时间
        uint64_t getLastExposureStart() const;
        
        // 获取曝光历史记录
        std::deque<T> getExposureHistory() const;

        bool getLatestExposureData(T& latest) const;


    private:
        // 最近一次的曝光开始时间（纳秒）
        std::atomic<uint64_t> last_exposure_start_ns_{0};
        
        // 曝光时间队列（保存最近100次曝光）
        std::deque<T> exposure_history_ ; // 使用指针以便存储图像数据
        mutable std::shared_mutex history_mutex_;
        const size_t max_history_size_ = 100;
        
        // 私有构造函数（单例）
        ExposureRecorder() = default;
};













// 获取单例实例
template<typename T>
ExposureRecorder<T>& ExposureRecorder<T>::instance() {
    static ExposureRecorder<T> recorder;
    return recorder;
}



// 记录曝光开始时间
template<typename T>
void ExposureRecorder<T>::recordExposureStart() {
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    
    // 更新最新曝光时间
    last_exposure_start_ns_.store(time_ns);
}






// 添加曝光数据到历史记录
template<typename T>
void ExposureRecorder<T>::push2History(T frame) {
        
    std::unique_lock<std::shared_mutex> lock(history_mutex_);

    exposure_history_.push_back(frame);
    
    // 保持历史记录大小不超过max_history_size_
    if (exposure_history_.size() > max_history_size_) {
        exposure_history_.pop_front();
    }
}



// 获取最近一次曝光开始时间
template<typename T>
uint64_t ExposureRecorder<T>::getLastExposureStart() const {
    return last_exposure_start_ns_.load();
}




// 获取曝光历史记录
template<typename T>
std::deque<T> ExposureRecorder<T>::getExposureHistory() const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    return exposure_history_;
}




// 获取最新曝光数据
template<typename T>
bool ExposureRecorder<T>::getLatestExposureData(T& latest) const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    if (!exposure_history_.empty()) {

        if(latest.timestamp_ns == exposure_history_.back().timestamp_ns)
        {
            return true;
        }

        latest = exposure_history_.back();
        return true;
    }
    return false; // 没有最新数据
}
