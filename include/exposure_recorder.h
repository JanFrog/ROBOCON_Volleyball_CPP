#pragma once
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <deque>
#include <string>
#include "opencv2/opencv.hpp"
#include <tuple>

#include "frame_data.h"




class ExposureRecorder {
    private:
        // 最近一次的曝光开始时间（纳秒）
        std::atomic<uint64_t> last_exposure_start_ns_{0};
        
        // 曝光时间队列（保存最近100次曝光）
        std::deque<frame_data> exposure_history_ ; // 使用指针以便存储图像数据
        mutable std::shared_mutex history_mutex_;
        const size_t max_history_size_ = 100;
        
        // 私有构造函数（单例）
        ExposureRecorder() = default;
        
    public:
        // 获取单例实例
        static ExposureRecorder& instance();
        
        // 记录曝光开始时间
        void recordExposureStart();


        void push2History(frame_data frame);
        
        // 获取最近一次曝光开始时间
        uint64_t getLastExposureStart() const;
        
        // 获取曝光历史记录
        std::deque<frame_data> getExposureHistory() const;

        frame_data getLatestExposureData() const;
};