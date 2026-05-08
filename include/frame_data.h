#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Dense>





struct frame_data_mono{
    uint64_t timestamp_ns;      // 帧的时间戳（纳秒）
    cv::Mat image;              // 帧的图像数据
    bool detected = false;      // 是否检测到目标
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down                                                                                           
};



struct frame_data_mono_4_saving{
    uint64_t timestamp_ns;          // 帧的时间戳（纳秒）
    std::vector<uchar> bin_data;  // 帧的二进制图像数据
    bool detected = false;          // 是否检测到目标
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down                                                                                           
};




struct frame_data_stereo{
    uint64_t timestamp_ns;  // 帧的时间戳（纳秒）
    cv::Mat left_image;     // 帧的左图像数据
    bool detected = false;  // 是否检测到目标
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down
    Eigen::Vector3d coordinate; // 目标坐标（3d）
};