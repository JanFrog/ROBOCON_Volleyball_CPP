#include "How_2_move.h"




Commander::Commander(COMMANDER_Params params, Locator_Params locator_params): config(params)
{
    pos_reset();
    aim_reset();

    this->img_width_ = locator_params.img_width;    // 图像宽度
    this->img_height_ = locator_params.img_height;  // 图像高度
    this->trans_mtx_ = locator_params.trans_mtx;    // 相机角度修正
    this->cam_mtx_ = locator_params.mtx;            // 相机内参
}



void Commander::pos_reset()
{
    pos_x = {0.0, 0.0, 0.0, -1};
    pos_y = {0.0, 0.0, 0.0, -1};
    pos_omega = {0.0, 0.0, 0.0, -1};
}


void Commander::aim_reset()
{
    aim_x = {0.0, 0.0, 0.0, -1};
    aim_y = {0.0, 0.0, 0.0, -1};
}



void Commander::restriction(Command& cmd, short method)
{
    int16_t vel = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy);

    switch(method)
    {
        case 0:
            if(vel > this->config.max_vel)
            {
                double scale = (double)vel / (double)this->config.max_vel;
                cmd.vx = (int16_t)(cmd.vx * scale);
                cmd.vy = (int16_t)(cmd.vy * scale);
            }

            if(cmd.omega > this->config.max_omega)
            {
                cmd.omega = this->config.max_omega;
            }
            return;

        default:
            return;
    }
}





void Commander::aim_iterate(double left, double top, double width, double height, uint64_t timestamp_ns, Command& cmd)
{
    // 误差方向与应该运动的方向相同
    double pos_x_deviation = (top + (height / 2)) - (this->img_height_ / 2);
    double pos_y_deviation = (this->img_width_ / 2) - (left + (width / 2));

    std::cout << "pos_x_deviation: " << pos_x_deviation << " pos_y_deviation: " << pos_y_deviation << std::endl;

    // 相机仰角pitch修正
    if(this->trans_mtx_(2, 0) != 0.0)
    {
        pos_x_deviation /= trans_mtx_(2, 0);     // 除以sin(pitch)
    }
    else
    {
        pos_x_deviation = 0.0;
    }

    // 更新积分项和导数项
    if(aim_x.timestamp_ns != -1)
    {
        aim_x.integral += pos_x_deviation * (double)(timestamp_ns - aim_x.timestamp_ns) / 1000000000.0;
        aim_y.integral += pos_y_deviation * (double)(timestamp_ns - aim_y.timestamp_ns) / 1000000000.0;

        aim_x.derivative = (pos_x_deviation - aim_x.deviation) / (double)(timestamp_ns - aim_x.timestamp_ns) * 1000000000.0;
        aim_y.derivative = (pos_y_deviation - aim_y.deviation) / (double)(timestamp_ns - aim_y.timestamp_ns) * 1000000000.0;
    }


    // 更新当前误差
    aim_x.deviation = pos_x_deviation;
    aim_y.deviation = pos_y_deviation;

    aim_x.timestamp_ns = timestamp_ns;
    aim_y.timestamp_ns = timestamp_ns;


    // 计算目标速度
    cmd.vx = (this->config.KP_4_aim * pos_x_deviation) + (this->config.KI_4_aim * aim_x.integral) + (this->config.KD_4_aim * aim_x.derivative);
    cmd.vy = (this->config.KP_4_aim * pos_y_deviation) + (this->config.KI_4_aim * aim_y.integral) + (this->config.KD_4_aim * aim_y.derivative);

    // std::cout<< "vx: "<< cmd.vx << " vy: " << cmd.vy << " omega: " << cmd.omega << std::endl;
    // 限制速度
    restriction(cmd, 0);

    return;

}