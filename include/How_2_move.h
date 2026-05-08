#pragma once

#include <stdint.h>
#include <cmath>

#include "locator_2d.h"



struct COMMANDER_Params
{
    double KP_4_pos=0.0, KI_4_pos=0.0, KD_4_pos=0.0;
    double KP_4_aim=0.0, KI_4_aim=0.0, KD_4_aim=0.0;
    int16_t max_vel=0;
    int16_t max_omega=0;
};

struct Command
{
    int16_t vx=0, vy=0, omega=0;
};

struct Bias
{
    double deviation = 0.0;
    double integral = 0.0;
    double derivative = 0.0;

    int64_t timestamp_ns = -1;
};



class Commander
{
public:

    Commander(COMMANDER_Params params, Locator_Params locator_params);
    // Commander(COMMANDER_Params params, int img_width, int img_height, );
    ~Commander() = default;

    void pos_reset();
    void aim_reset();

    void aim_iterate(double left, double top, double width, double height, uint64_t timestamp_ns, Command& cmd);

private:

    COMMANDER_Params config;

    int img_width_, img_height_;    // 图像尺寸
    Eigen::Matrix3d trans_mtx_;     // 相机角度修正
    Eigen::Matrix3d cam_mtx_;       // 相机内参
    

    Bias pos_x, pos_y, pos_omega;
    Bias aim_x, aim_y;

    void restriction(Command& cmd, short method=0);
};