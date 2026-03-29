#pragma once

#include <opencv2/opencv.hpp>
#include <deque>




enum frame_type{
    frame_type_0 = 0,   // mono cam
    frame_type_1 = 1    // zed
};


struct frame_data{
    frame_type type;
    uint64_t timestamp_ns; // 帧的时间戳（纳秒）
    cv::Mat image;                      // 帧的图像数据
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down
    bool detected = false;                                                                                              
};