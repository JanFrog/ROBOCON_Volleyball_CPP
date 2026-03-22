#include "exposure_recorder.h"

// 获取单例实例
ExposureRecorder& ExposureRecorder::instance() {
    static ExposureRecorder recorder;
    return recorder;
}

// 记录曝光开始时间
void ExposureRecorder::recordExposureStart() {
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    
    // 更新最新曝光时间
    last_exposure_start_ns_.store(time_ns);
}



void ExposureRecorder::push2History(frame_data frame) {
        
    std::unique_lock<std::shared_mutex> lock(history_mutex_);

        exposure_history_.push_back(frame);
        
        // 保持历史记录大小不超过max_history_size_
        if (exposure_history_.size() > max_history_size_) {
            exposure_history_.pop_front();
        }
    }




// 获取最近一次曝光开始时间
uint64_t ExposureRecorder::getLastExposureStart() const {
    return last_exposure_start_ns_.load();
}

// 获取曝光历史记录
std::deque<frame_data> ExposureRecorder::getExposureHistory() const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    return exposure_history_;
}

frame_data ExposureRecorder::getLatestExposureData() const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    if (!exposure_history_.empty()) {
        return exposure_history_.back();
    }
    return frame_data{}; // 返回默认构造的frame_data
}