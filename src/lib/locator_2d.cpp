#include "locator_2d.h"




Locator::Locator(int img_width, int img_height, float radius, Eigen::Matrix3d mtx, float angel_width, float angel_height, Eigen::Matrix3d trans_mtx):
    img_width_(img_width), img_height_(img_height), radius_(radius)
{
    trans_mtx_ = trans_mtx;

    if (mtx != Eigen::Matrix3d::Zero()) {
        f1 = mtx(0, 0);
        f2 = mtx(1, 1);
    }
    else if (angel_width > 0 && angel_height > 0) {
        f1 = (img_width_ / 2) / tan(angel_width / 2);
        f2 = (img_height_ / 2) / tan(angel_height / 2);
    }
    else {
        std::cerr << "初始化失败：请提供有效的相机内参矩阵或视场角参数" << std::endl;
        throw std::invalid_argument("Invalid camera parameters");
    }

    std::cout<<"Locator initialized ! ( f1: " << f1 << ", f2: " << f2 << " )\n";

}



std::tuple<float, float> Locator::__count_1d(float pixel_a, float pixel_b, float f){

    float pixel_delta = pixel_b - pixel_a;

    float theta_1 = atan(pixel_a / f);
    float theta_2 = atan(pixel_b / f);

    // float beta = (pixel_a * sin(theta_2)) / (pixel_b * sin(theta_1));
    
    float depth = ((radius_ / cos(theta_1)) + (radius_ / cos(theta_2))) * f / pixel_delta;
    float offset = depth * (pixel_a + (pixel_delta * (1 / (1 + 1 / (pixel_a * sin(theta_2)) / (pixel_b * sin(theta_1)))))) / f;

    return std::make_tuple(offset, depth);
}


/*
输入坐标系：
    原点:   左上角
    width:  正方向→
    height: 正方向↓   */
Eigen::Vector3d Locator::locate(float left, float top, float width, float height){

    /*
    坐标系转换:
        原点:   正中心
        width:  正方向←
        height: 正方向↑   */
    float pixel_w_b = (img_width_ / 2) - left;
    float pixel_w_a = pixel_w_b - width;
    float pixel_h_b = (img_height_ / 2) - top;
    float pixel_h_a = pixel_h_b - height;
    

    // 分别计算水平和垂直方向的偏移和深度
    auto [offset_y, depth_1] = __count_1d(pixel_w_a, pixel_w_b, f1);
    auto [offset_z, depth_2] = __count_1d(pixel_h_a, pixel_h_b, f2);



    Eigen::Vector3d result(std::min(depth_1, depth_2), offset_y, offset_z);

    return (trans_mtx_ * result);

}