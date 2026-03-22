#pragma once
#include "exposure_recorder.h"


// class Coordinator {
// public:

//     Coordinator(ExposureRecorder& recorder) : recorder_(recorder) {};
//     std::tuple<cv::Mat, std::tuple<float, float, float, float, float>> get(bool MAT = false, bool BBOX = true) {
//         auto data = recorder_.getLatestExposureData();
//         if (MAT && BBOX) {
//             return std::make_tuple(data.image, data.bboxes);
//         } else if (MAT) {
//             return std::make_tuple(data.image, std::tuple<float, float, float, float, float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
//         } else if (BBOX) {
//             return std::make_tuple(cv::Mat(), data.bboxes);
//         } else {
//             return std::make_tuple(cv::Mat(), std::tuple<float, float, float, float, float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
//         }
//     };

// private:
//     ExposureRecorder& recorder_;
//     frame_data latest_frame_data_;
//     short mode_ = 0; // 0: 返回最新帧数据，1: 返回指定时间戳的帧数据（待实现）
// };