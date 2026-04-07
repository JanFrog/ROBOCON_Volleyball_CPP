#pragma once
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <tuple>
#include <iostream>
#include <cmath>




struct Locator_Params{
    
    int img_width = 1280;
    int img_height = 1024;
    
    double radius = 0.1;    // 排球半径
    
    double box_noise = 20.0; // 可接受的最大目标框噪声

    Eigen::Matrix3d mtx = Eigen::Matrix3d::Zero();

    double angel_width = 0.0;
    double angel_height = 0.0;
    
    Eigen::Matrix3d trans_mtx = Eigen::Matrix3d::Identity();

};











class Locator{
public:
    Locator(const Locator_Params& params);

    ~Locator() = default;
    
    bool locate(double left, double top, double width, double height, Eigen::Vector3d& result);



private:
    
    double box_noise_;       // 可接受的最大目标框噪声

    int img_width_;         // 图像宽度
    int img_height_;        // 图像高度

    double radius_;          // 排球半径

    float f1;              // 焦距 (纵向像素为单位描述)
    float f2;              // 焦距 (横向像素为单位描述)



    Eigen::Matrix3d trans_mtx_;     // 相机姿态

    std::tuple<double, double, double> __count_1d(double pixel_a, double pixel_b, double f);

};