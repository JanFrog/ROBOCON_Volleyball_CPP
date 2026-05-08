#pragma once
#include <string>
#include <vector>
#include <deque>
#include <fstream>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "frame_data.h"




class data_Loader
{
public:

    data_Loader(uint32_t batch_size = 20, uint32_t Preload_batch_num = 3);
    ~data_Loader();

    bool load_file(std::string data_path, uint32_t img_height, uint32_t img_width, uint32_t img_channel);   // 加载数据，记录数据信息 (准确地说是记录数据位置方便后续索引)
    bool get_single_frame(frame_data_mono& frame);      // 获取单帧数据



private:

    // 文件信息
    std::string data_path;                  // 数据路径
    std::fstream data_fs;                   // 数据文件流
    std::streampos end_of_file;             // 文件大小 (单位：字节)
    std::vector<size_t> frame_positions;    // 每一帧数据的文件位置 (单位：字节)


    // 视频信息
    uint64_t total_frame_num;       // 总帧数
    uint32_t img_height, img_width; // 图像高度和宽度
    uint32_t img_channel;           // 图像通道数


    // 播放配置
    uint32_t batch_size;            // 每个批次的帧数
    uint32_t Preload_batch_num;     // 预加载批次数


    // 播放信息
    uint64_t current_frame_seq;         // 当前播放帧位置 (序号，从0开始计数)
    std::atomic<bool> playing;          // 用于控制子线程是否工作
    std::deque<frame_data_mono> frames; // 播放数据窗


    // 加载新批次相关
    std::atomic<uint64_t> current_cache_num;    // 当前缓冲区剩余帧数   (由 check_thread、Load_file、get_single_frame 维护，其他成员可读)
    std::atomic<uint64_t> cache_end_seq;        // 缓冲区末尾帧序号     (由 check_thread、Load_file 维护，其他成员可读)
    std::thread check_thread;                   // 检查新批次数据线程
    std::mutex check_mtx;                       // 用于保护check_now的互斥锁
    std::mutex queue_mtx;                       // 用于保护frames的互斥锁
    std::condition_variable Thread_waker;       // 用于唤醒check_thread的条件变量
    std::atomic<bool> check_now;                // 用于通知check_thread有新数据
    
    void check_for_new_batch();     // 检查是否需要加载新的批次数据
    
};