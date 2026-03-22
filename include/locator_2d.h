#pragma once
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include <cmath>


class Locator{
public:
    Locator(
        int img_width = 1280,
        int img_height = 1024,
        float radius = 0.1f,
        Eigen::Matrix3d mtx = Eigen::Matrix3d::Zero(),
        float angel_width = 0.0f,
        float angel_height = 0.0f,
        Eigen::Matrix3d trans_mtx = Eigen::Matrix3d::Identity());

    ~Locator() = default;
    
    Eigen::Vector3d locate(float left, float top, float width, float height);

private:
    int img_width_;
    int img_height_;
    float radius_;
    float f1;
    float f2;
    Eigen::Matrix3d trans_mtx_;

    std::tuple<float, float> __count_1d(float pixel_a, float pixel_b, float f);    

};