#include "dataLoader.h"




data_Loader::data_Loader(uint32_t batch_size, uint32_t Preload_batch_num)
{
    this->batch_size = batch_size;
    this->Preload_batch_num = Preload_batch_num;

    this->playing.store(true);
    this->current_frame_seq = 0;
    this->total_frame_num = 0;
    this->end_of_file = 0;

    this->img_height = 0;
    this->img_width = 0;
    this->img_channel = 0;

    this->check_now = false;
    this->current_cache_num.store(0);
    this->cache_end_seq = 0;

    // 启动检查新批次数据线程
    this->check_thread = std::thread(&data_Loader::check_for_new_batch, this);

}


data_Loader::~data_Loader()
{
    if(this->data_fs.is_open())
    this->data_fs.close();

    // 停止检查新批次数据线程
    this->playing.store(false);

    if(check_thread.joinable())
    {
        // 通知check_thread退出
        std::unique_lock<std::mutex> lock_1(check_mtx);
        check_now.store(true);
        Thread_waker.notify_all();
        lock_1.unlock();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check_thread.join();
}






bool data_Loader::load_file(std::string data_path, uint32_t img_height, uint32_t img_width, uint32_t img_channel)
{

    // 清空frames
    this->frames.clear();
    this->frame_positions.clear();

    this->current_frame_seq = 0;
    this->total_frame_num = 0;
    this->cache_end_seq = 0;


    // Load_file行为目录：
    // 1、配置data_fs
    // 2、更新文件路径
    // 3、记录每一帧数据位置
    // 4、记录总帧数
    // 5、记录图像高度、宽度、通道数
    // 6、初始化前几个批次数据
    


    // 1、配置data_fs
    if(this->data_fs.is_open())
        this->data_fs.close();

    this->data_fs.open(data_path, std::ios::binary | std::ios::in);

    if (!this->data_fs.is_open()){
        // 无法加载文件
        return false;
    }




    // 2、更新文件路径
    this->data_path = data_path;


    // 3、记录每一帧数据位置
    this->frame_positions.clear();

    uint32_t Mat_size;
    size_t head_size = sizeof(uint64_t) + sizeof(bool) + sizeof(float)*5;
    size_t tail_size;
    this->data_fs.seekg(0, std::ios::end);
    this->end_of_file = this->data_fs.tellg();
    this->data_fs.seekg(0, std::ios::beg);
    std::cout << "end_of_file: " << this->end_of_file << std::endl;


    // 遍历文件，记录每一帧数据位置
    while(this->data_fs.tellg() < end_of_file)
    {
        // 记录当前帧位置
        this->frame_positions.push_back(this->data_fs.tellg());

        // 读取Mat_size
        this->data_fs.seekg(head_size, std::ios::cur);
        this->data_fs.read((char*)&Mat_size, sizeof(uint32_t));

        // 根据Mat_size跳过数据尾
        this->data_fs.seekg(Mat_size, std::ios::cur);
    }
    this->data_fs.seekg(0, std::ios::beg);



    // 4、记录总帧数
    this->total_frame_num = this->frame_positions.size();
    std::cout << "total_frame_num: " << this->total_frame_num << std::endl;


    // 5、记录图像高度、宽度、通道数
    this->img_height = img_height;
    this->img_width = img_width;
    this->img_channel = img_channel;
    std::vector<uint8_t> bin_img_buffer;
    bin_img_buffer.reserve(img_height * img_width * img_channel);

    std::cout << "记录完毕" << std::endl;



    std::unique_lock<std::mutex> lock_2(queue_mtx);


    // 6、初始化前几个批次数据
    std::cout << "locked successfully, start to load data" << std::endl;
    float bboxes[5];

    while(this->current_cache_num.load() < this->Preload_batch_num * this->batch_size && this->cache_end_seq < this->total_frame_num)
    {
        std::cout << "current_cache_num: " << this->current_cache_num.load() << std::endl;
        // 读取一帧数据
        frame_data_mono frame_to_push;

        // 解码曝光时间
        this->data_fs.read((char*)&frame_to_push.timestamp_ns, sizeof(uint64_t));

        // 解码检测框
        this->data_fs.read((char*)&frame_to_push.detected, sizeof(bool));
        this->data_fs.read((char*)&bboxes, sizeof(float)*5);
        frame_to_push.bboxes = std::make_tuple(bboxes[0], bboxes[1], bboxes[2], bboxes[3], bboxes[4]);

        // 解码图像数据
        this->data_fs.read((char*)&Mat_size, sizeof(uint32_t));
        bin_img_buffer.resize(Mat_size);
        this->data_fs.read((char*)&bin_img_buffer[0], Mat_size);
        frame_to_push.image = cv::imdecode(bin_img_buffer, cv::IMREAD_COLOR);
        bin_img_buffer.clear();


        // 压入frames
        this->frames.push_back(frame_to_push);
        this->current_cache_num++;
        this->cache_end_seq++;
    }

    lock_2.unlock();


    return true;
}






bool data_Loader::get_single_frame(frame_data_mono& frame)
{

    if(this->current_frame_seq >= this->total_frame_num - 1)
    {
        // 到达文件末尾
        std::cout << "reach end of file" << std::endl;
        return false;
    }


    if(this->frames.empty())
    {
        // 无数据可读
        std::cout << "frames is empty" << std::endl;
        return false;
    }

    // 读取一帧数据
    std::unique_lock<std::mutex> lock_2(queue_mtx);
    frame = frames.front();
    frames.pop_front();
    lock_2.unlock();


    // 更新当前帧序号 (+1)
    this->current_frame_seq++;

    // 更新当前缓存帧数 (-1)
    this->current_cache_num--;


    // 双条件：A 缓冲帧数不满足要求 && B 缓冲末尾未达到总帧数
    if((this->current_cache_num.load() <= (this->Preload_batch_num - 1) * this->batch_size) && (this->cache_end_seq.load() < (this->total_frame_num - 1)))
    {
        std::cout << "current_cache_num: " << this->current_cache_num.load() / this->batch_size << std::endl;
        // 通知线程应加载新缓存数据
        std::unique_lock<std::mutex> lock_1(check_mtx);
        check_now.store(true);
        Thread_waker.notify_all();
        lock_1.unlock();
    }

    return true;
}




// 线程任务
void data_Loader::check_for_new_batch()
{
    // 初始化锁
    std::unique_lock<std::mutex> lock_1(check_mtx, std::defer_lock);
    std::unique_lock<std::mutex> lock_2(queue_mtx, std::defer_lock);

    // 
    std::vector<frame_data_mono> wait_to_link;

    while(true)
    {
        // 等待有新数据
        std::cout << "准备上锁" << std::endl;
        lock_1.lock();
        std::cout << "已上锁，检查激活通知" << std::endl;
        Thread_waker.wait(lock_1, [this] { return check_now.load(); });
        lock_1.unlock();

        if(!playing.load())
        {
            // 退出检查点
            std::cout << "退出" << std::endl;
            break;
        }

        std::cout<< "已唤醒线程，准备检查新批次数据"<< std::endl;
        check_now.store(false);

        // 清空候选队列
        wait_to_link.clear();
        
        // 移动fs指针
        this->data_fs.seekg(this->frame_positions[this->cache_end_seq.load()], std::ios::beg);

        size_t Mat_size;
        std::vector<uint8_t> bin_img_buffer;
        float bboxes[5];



        while(this->data_fs.tellg() < this->end_of_file && wait_to_link.size() < this->batch_size)
        {
            // 读取一帧数据
            frame_data_mono frame_to_push;
            
            // 解码曝光时间
            this->data_fs.read((char*)&frame_to_push.timestamp_ns, sizeof(uint64_t));

            // 解码检测框
            this->data_fs.read((char*)&frame_to_push.detected, sizeof(bool));
            this->data_fs.read((char*)&bboxes[0], sizeof(float)*5);
            frame_to_push.bboxes = std::make_tuple(bboxes[0], bboxes[1], bboxes[2], bboxes[3], bboxes[4]);

            // 解码图像数据
            this->data_fs.read((char*)&Mat_size, sizeof(uint32_t));
            bin_img_buffer.resize(Mat_size);
            this->data_fs.read((char*)&bin_img_buffer[0], Mat_size);
            frame_to_push.image = cv::imdecode(bin_img_buffer, cv::IMREAD_COLOR);
            bin_img_buffer.clear();


            // 压入一帧数据
            wait_to_link.push_back(frame_to_push);
        }

        if(!wait_to_link.empty())
        {
            std::cout << "写入ing" << std::endl;
            lock_2.lock();

            this->frames.insert(this->frames.end(), wait_to_link.begin(), wait_to_link.end());
            this->current_cache_num += wait_to_link.size();
            this->cache_end_seq += wait_to_link.size();

            lock_2.unlock();

            wait_to_link.clear();
        }

    }    
}