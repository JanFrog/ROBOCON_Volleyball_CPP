#include "locator_2d.h"






Locator::Locator(const Locator_Params& params):
    img_width_(params.img_width), img_height_(params.img_height), radius_(params.radius), box_noise_(params.box_noise)
{

    trans_mtx_ = params.trans_mtx;
    
    if (params.mtx != Eigen::Matrix3d::Zero()) {
        f1 = params.mtx(0, 0);
        f2 = params.mtx(1, 1);
    }
    else if (params.angel_width > 0 && params.angel_height > 0) {
        f1 = (img_width_ / 2) / tan(params.angel_width / 2);
        f2 = (img_height_ / 2) / tan(params.angel_height / 2);  
    }
    else {
        std::cerr << "初始化失败：请提供有效的相机内参矩阵或视场角参数" << std::endl;
        throw std::invalid_argument("Invalid camera parameters");
    }

    std::cout<<"Locator initialized ! ( f1: " << f1 << ", f2: " << f2 << " )\n";

}



std::tuple<double, double, double> Locator::__count_1d(double pixel_a, double pixel_b, double f)
{
    
    // // 老流程 (更直观)

    // float pixel_delta = pixel_b - pixel_a;

    // float theta_1 = atan(pixel_a / f);
    // float theta_2 = atan(pixel_b / f);

    // // float beta = (pixel_a * sin(theta_2)) / (pixel_b * sin(theta_1));
    
    // float depth = ((radius_ / cos(theta_1)) + (radius_ / cos(theta_2))) * f / pixel_delta;
    // // float depth = (sqrt((pixel_a * pixel_a) + (f * f)) + sqrt((pixel_b * pixel_b) + (f * f))) * radius_ / pixel_delta;

    // float offset = depth * (pixel_a + (pixel_delta * (1 / ((pixel_b * sin(theta_1)) / (pixel_a * sin(theta_2)) + 1)))) / f;
    // double depth_tolerance = 10000;




    // 新流程 (性能更好, 优化掉了超越函数)

    double pixel_delta = pixel_b - pixel_a;

    double sqrt_af = sqrt((pixel_a * pixel_a) + (f * f));
    double sqrt_bf = sqrt((pixel_b * pixel_b) + (f * f));


    double depth = (sqrt_af + sqrt_bf) * radius_ / pixel_delta;

    double offset = ((pixel_b * sqrt_af) + (pixel_a * sqrt_bf)) * radius_ / pixel_delta / f;

    double depth_tolerance = box_noise_ * (radius_ / pixel_delta) * \
        (std::abs((((sqrt_af + sqrt_bf) / pixel_delta)) + (pixel_a / sqrt_af)) + std::abs(-((sqrt_af + sqrt_bf) / pixel_delta) + (pixel_b / sqrt_bf)));

    return std::make_tuple(offset, depth, depth_tolerance);
}





/*
输入坐标系：
    原点:   左上角
    width:  正方向→
    height: 正方向↓   */
bool Locator::locate(double left, double top, double width, double height, Eigen::Vector3d& result){

    /*
    坐标系转换:
        原点:   正中心
        width:  正方向←
        height: 正方向↑   */
    double pixel_w_b = (img_width_ / 2) - left;
    double pixel_w_a = pixel_w_b - width;
    double pixel_h_b = (img_height_ / 2) - top;
    double pixel_h_a = pixel_h_b - height;
    

    // 分别计算水平和垂直方向的偏移和深度
    auto [offset_y, depth_1, tolerance_1] = __count_1d(pixel_w_a, pixel_w_b, f1);
    auto [offset_z, depth_2, tolerance_2] = __count_1d(pixel_h_a, pixel_h_b, f2);


    
    if((img_width_ / 2) - std::max(std::abs(pixel_w_a), std::abs(pixel_w_b)) <= box_noise_ || (img_height_ / 2) - std::max(std::abs(pixel_h_a), std::abs(pixel_h_b)) <= box_noise_)
    {
        if(std::abs((depth_1 - depth_2)) > (tolerance_1 > tolerance_2 ? tolerance_2 : tolerance_1))
        {
            std::cout<< "\nLocator: depth tolerance not met, tolerance: " << (tolerance_1 > tolerance_2 ? tolerance_2 : tolerance_1) << "\ndepth_difference: " << std::abs(depth_1 - depth_2) << std::endl;
            return false;
        }
    }


    result.x() = std::min(depth_1, depth_2);
    result.y() = offset_y;
    result.z() = offset_z;

    return true;

}