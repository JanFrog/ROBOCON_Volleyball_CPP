#pragma once

#include <opencv2/opencv.hpp>
#include <deque>


struct frame_data{
    uint64_t timestamp_ns; // 帧的时间戳（纳秒）
    cv::Mat image;                      // 帧的图像数据
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down
    bool detected = false;                                                                                              
};