
// Run with: LD_PRELOAD=/usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so ./Wdata




// Memory struct:

    // uint64_t timestamp_ns;          // 帧的时间戳（纳秒）
    // bool detected = false;          // 是否检测到目标
    // std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down
    // uint32_t Mat_size (byte)
    // std::vector<uint8_t> bin_data;  // 帧的二进制图像数据   




#include <iostream>
#include <fstream>
#include <gst/gst.h>
#include <string>
#include <sstream>
#include <chrono>
#include <queue>
#include <iomanip>
#include <thread>
#include "gstnvdsmeta.h" 
#include <glib-unix.h>
#include <filesystem>
#include <atomic>
#include <vector>

// socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Keyboard input
#include <termios.h>










struct frame_data_mono_4_saving{
    uint64_t timestamp_ns;          // 帧的时间戳（纳秒）
    std::vector<uint8_t> bin_data;  // 帧的二进制图像数据
    bool detected = false;          // 是否检测到目标
    std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down                                                                                           
};






#include <mutex>
#include <shared_mutex>
#include <deque>
#include <Eigen/Dense>
#include <tuple>



template<typename T>
class ExposureRecorder {


    public:
        // 获取单例实例
        static ExposureRecorder<T>& instance();
        
        // 记录曝光开始时间
        void recordExposureStart();


        void push2History(T frame);
        
        // 获取最近一次曝光开始时间
        uint64_t getLastExposureStart() const;
        
        // 获取曝光历史记录
        std::deque<T> getExposureHistory() const;

        bool getLatestExposureData(T& latest) const;


    private:
        // 最近一次的曝光开始时间（纳秒）
        std::atomic<uint64_t> last_exposure_start_ns_{0};
        
        // 曝光时间队列（保存最近100次曝光）
        std::deque<T> exposure_history_ ; // 使用指针以便存储图像数据
        mutable std::shared_mutex history_mutex_;
        const size_t max_history_size_ = 100;
        
        // 私有构造函数（单例）
        ExposureRecorder() = default;
};













// 获取单例实例
template<typename T>
ExposureRecorder<T>& ExposureRecorder<T>::instance() {
    static ExposureRecorder<T> recorder;
    return recorder;
}



// 记录曝光开始时间
template<typename T>
void ExposureRecorder<T>::recordExposureStart() {
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    
    // 更新最新曝光时间
    last_exposure_start_ns_.store(time_ns);
}






// 添加曝光数据到历史记录
template<typename T>
void ExposureRecorder<T>::push2History(T frame) {
        
    std::unique_lock<std::shared_mutex> lock(history_mutex_);

    exposure_history_.push_back(frame);
    
    // 保持历史记录大小不超过max_history_size_
    if (exposure_history_.size() > max_history_size_) {
        exposure_history_.pop_front();
    }
}



// 获取最近一次曝光开始时间
template<typename T>
uint64_t ExposureRecorder<T>::getLastExposureStart() const {
    return last_exposure_start_ns_.load();
}




// 获取曝光历史记录
template<typename T>
std::deque<T> ExposureRecorder<T>::getExposureHistory() const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    return exposure_history_;
}




// 获取最新曝光数据
template<typename T>
bool ExposureRecorder<T>::getLatestExposureData(T& latest) const {
    std::shared_lock<std::shared_mutex> lock(history_mutex_);
    if (!exposure_history_.empty()) {

        if(latest.timestamp_ns == exposure_history_.back().timestamp_ns)
        {
            return true;
        }

        latest = exposure_history_.back();
        return true;
    }
    return false; // 没有最新数据
}















// ==================== 全局变量 ====================
std::queue<std::chrono::high_resolution_clock::time_point> frame_times;     // 用于存储帧时间戳的队列
const int img_width = 1280, img_height = 1024, fast_frame_rate = 200, slow_frame_rate = 30; // 图像尺寸和帧率
int frame_counter = 0;              // 帧计数器
int detect_count = 0;               // 检测计数器
uint64_t last_refresh_tick = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    // 上一刷新时间（纳秒）
int refresh_epoch = 1;              // 刷新周期（秒）

bool Give_img = true;               // 是否给图像

std::atomic<bool> thread_exit = false;      // 线程退出标志
int run_seconds = 600;                      // 运行时间（秒）
float CONF_THRESHOLD = 0.7f;                // 置信度阈值
















// ==================== 录制线程变量 ====================

// 录制状态(定义)
enum RecordState{
    stopped,
    recording
};

std::filesystem::path OPT_DIR = "/home/nvidia/share/Realtime_data/";
const size_t batch_size = 100;

std::atomic<int> instruction = 0;   // 录制指令
std::atomic<bool> on_packing = false;       // 是否正在打包数据
std::atomic<bool> new_img = false;          // 是否有新图像可用
std::filesystem::path record_dir;           // 录制目录
std::atomic<RecordState> working_state = stopped;    // 录制状态
frame_data_mono_4_saving frame_tosave;      // 待保存图像
std::ofstream ofs;

// 写入线程变量
std::atomic<int8_t> batch_ready = 0;
std::vector<uint8_t> batch_buffer_1;
std::vector<uint8_t> batch_buffer_2;
std::atomic<bool> on_saving = false;        // 是否正在保存数据










// ============================ 键盘监听线程 =============================
// 用ai写了我真的太困了
void KeyBoard_listen()
{
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;   // 完全非阻塞
    newt.c_cc[VTIME] = 0;  // 不使用 termios 自带超时
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    std::cout << "Press any key ...\n";

    while (!thread_exit) {
        // ========== 使用 select 实现键盘输入 10ms 超时 ==========
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms = 10,000 微秒

        // 等待输入，超时时间 10ms
        int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

        // 
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {

            // 有按键输入
            if (read(STDIN_FILENO, &c, 1) > 0) {
                instruction.store(int(c));
            }
        }
        // ret == 0 表示超时，无需处理，继续循环
        // 这里不需要额外的 sleep，因为 select 已经阻塞了 10ms
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}











// ============================ 保存线程 =============================
void save_batches()
{
    std::cout << "保存线程已打开" << std::endl;

    // 记录总数据大小
    uint64_t total_size = 0;


    std::vector<uint8_t>* batch_tosave;
    int batch_seq = 0;

    while(!thread_exit)
    {
        if(working_state.load() == recording)
        {

            if(batch_ready.load() == 0) 
            {
                // 等待有数据可保存
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            if(batch_seq == 0)
            {
                std::cout << "尝试打开文件：" << record_dir / "data.bin" << std::endl;
                ofs.open((record_dir / "data.bin"), std::ios::binary | std::ios::out | std::ios::trunc);

                if(!ofs.is_open()){
                    std::cout << "文件打开失败" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
            }

            if(batch_ready.load() == 1)
            {
                // batch_buffer_1 已准备进入保存流程
                batch_tosave = &batch_buffer_1;
            }
            else if(batch_ready.load() == 2)
            {
                // batch_buffer_2 已准备进入保存流程
                batch_tosave = &batch_buffer_2;
            }
            else{
                // batch_ready 状态错误
                std::cout << "batch_ready state error" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            

            // 标志位设为正在保存
            on_saving.store(true);

            // 记录批次大小
            total_size += batch_tosave->size();
            ofs.write(reinterpret_cast<const char*>(batch_tosave->data()), batch_tosave->size());
            ofs.flush();

            batch_tosave->clear();


            batch_seq++;

            // 标志位设为空闲
            on_saving.store(false);
            batch_ready.store(0); 
        }

        
        
        else if(working_state.load() == stopped)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(batch_ready.load() != 0)
            {
                if(batch_ready.load() == 1){
                    batch_tosave = &batch_buffer_1;
                }
                else{
                    batch_tosave = &batch_buffer_2;
                }


                
                on_saving.store(true);

                total_size += batch_tosave->size();
                ofs.write(reinterpret_cast<const char*>(batch_tosave->data()), batch_tosave->size());
                if(!ofs.good())
                {
                    std::cout << "写入文件失败，丢弃批次" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                ofs.flush();

                batch_tosave->clear();


                batch_seq++;

                // 标志位设为空闲
                on_saving.store(false);
                batch_ready.store(0); 



            }
            
            if(batch_seq != 0)
            {
                ofs.close();
                
                std::cout << "录制结束，总数据大小：";
                if(total_size / 1024.0 > 1)
                {
                    total_size /= 1024.0;

                    if(total_size / 1024.0 > 1)
                    {
                        total_size /= 1024.0;

                        if(total_size / 1024.0 > 1)
                        {
                            total_size /= 1024.0;
                            std::cout << total_size << "GB" << std::endl;
                        }
                        else
                        {
                            std::cout << total_size << "MB" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << total_size << "KB" << std::endl;
                    }
                }
                else
                {
                    std::cout << total_size << "Bytes" << std::endl;
                }

                batch_seq = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            std::cout << "working_state error" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}












// ============================ 打包线程 =============================
void data_pack_thread()
{

    std::thread save_batches_thread(save_batches);

    std::vector<uint8_t>* batch_buffer = &batch_buffer_1;


    size_t frame_count_in_batch = 0;


    // 分配batch内存 (交替使用)
    batch_buffer_1.reserve((8 + 1 + (4*5) + 4 + (300*1024)) * batch_size);
    batch_buffer_2.reserve((8 + 1 + (4*5) + 4 + (300*1024)) * batch_size);

    

    while (!thread_exit)
    {

        // state switcher
        if(instruction.load() == 10)
        {
            // Rst
            instruction.store(0);
            frame_count_in_batch = 0;


            if(working_state.load() == stopped){


                // 记录开始时间戳
                std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::tm* now_tm = std::localtime(&now_time_t);
            
                // 创建目录
                std::stringstream ss_tmp;
                ss_tmp << std::put_time(now_tm, "%Y-%m-%d_%H-%M-%S");

                record_dir = OPT_DIR / ss_tmp.str();
                std::filesystem::create_directories(record_dir);



                working_state.store(recording);
                std::cout << "录制开始" << std::endl;

            }

            else if(working_state.load() == recording)
            {
                working_state.store(stopped);
            }
        }






        if (working_state.load() == recording)
        {
            if (new_img.load())
            {
                // 保存图像标志位升起
                on_packing.store(true);




                

                // 序列化数据
                std::vector<uint8_t> buffer;            // 用于存储序列化后的数据
                buffer.reserve(8 + 1 + (4*5) + 4 + frame_tosave.bin_data.size());        // 预分配内存

                    // uint64_t timestamp_ns;          // 帧的时间戳（纳秒）
                    // bool detected = false;          // 是否检测到目标
                    // std::tuple<float, float, float, float, float> bboxes; // 检测框列表（left, top, width, height, confidence）x-positive = right y-positive = down
                    // uint32_t Mat_size (byte)
                    // std::vector<uint8_t> bin_data;  // 帧的二进制图像数据                                                                                     


                // 时间戳
                buffer.insert(buffer.end(), (uint8_t*)&frame_tosave.timestamp_ns, (uint8_t*)&frame_tosave.timestamp_ns + sizeof(uint64_t));
                
                // 是否检测到目标
                buffer.push_back(frame_tosave.detected);

                // 检测框列表
                if (frame_tosave.detected){
                    float bbox[5] = {std::get<0>(frame_tosave.bboxes), std::get<1>(frame_tosave.bboxes), std::get<2>(frame_tosave.bboxes), std::get<3>(frame_tosave.bboxes), std::get<4>(frame_tosave.bboxes)};
                    buffer.insert(buffer.end(), (uint8_t*)bbox, (uint8_t*)bbox + 20);
                }
                else{
                    float bbox[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    buffer.insert(buffer.end(), (uint8_t*)bbox, (uint8_t*)bbox + 20);
                }

                // 图像数据大小
                uint32_t Mat_size = frame_tosave.bin_data.size();
                buffer.insert(buffer.end(), (uint8_t*)&Mat_size, (uint8_t*)&Mat_size + sizeof(uint32_t));

                // 图像数据
                buffer.insert(buffer.end(), frame_tosave.bin_data.begin(), frame_tosave.bin_data.end());



                (*batch_buffer).insert((*batch_buffer).end(), buffer.begin(), buffer.end());
                frame_count_in_batch++;






                if(frame_count_in_batch >= batch_size){

                    while(on_saving.load()){
                        std::cout << "上一批次数据尚未保存完成，准备重新尝试" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }

                    frame_count_in_batch = 0;

                    // 标记批次为已准备
                    batch_ready.store((batch_buffer == &batch_buffer_1) ? 1 : 2);
                    batch_buffer =   ((batch_buffer == &batch_buffer_1) ? &batch_buffer_2 : &batch_buffer_1);
                    batch_buffer->clear();
                }




                // 保存图像标志位降下
                on_packing.store(false);
                new_img.store(false);
            }
            else    // 无新图像可用
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }
        else if (working_state.load() == stopped)    // 非录制状态
        {

            if(frame_count_in_batch > 0){

                while(on_saving.load())
                {
                    std::cout << "上一批次数据尚未保存完成，准备重新尝试" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                // 标记批次为已准备
                batch_ready.store((batch_buffer == &batch_buffer_1) ? 1 : 2);

                // 切换工作区 (buffer)
                batch_buffer =   ((batch_buffer == &batch_buffer_1) ? &batch_buffer_2 : &batch_buffer_1);

                // 清扫切换后的新工作区 (buffer)
                batch_buffer->clear();


                frame_count_in_batch = 0;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            while(on_saving.load())
            {
                std::cout << "上一批次数据尚未保存完成，等待完成后结束录制" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            // 等待录制指令
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
    }


    save_batches_thread.join();

}











// ==================== 数据中转server ====================
void recieve_thread(){

    frame_data_mono_4_saving frame_recieved{0, {}, {}, {}};
    uint64_t last_frame_tick = 0;
    bool skip = false;


    std::thread save_thread(data_pack_thread);
    std::thread keyboard_thread(KeyBoard_listen);






    while(!thread_exit){

        if(ExposureRecorder<frame_data_mono_4_saving>::instance().getLatestExposureData(frame_recieved))
        {


            if(frame_recieved.timestamp_ns == last_frame_tick){
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }

            last_frame_tick = frame_recieved.timestamp_ns;

            if(!on_packing.load())
            {

                skip = false;

                frame_tosave = frame_recieved;

                new_img.store(true);

            }
            else
            {
                if(skip){
                    std::cout << "警告，有未保存的图像被忽略" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                std::cout << "上一时刻数据未保存完成，跳过本次尝试" << std::endl;
                skip = true;
                continue;
            }



            

        }
    }



    save_thread.join();
    keyboard_thread.join();


}







// ==================== Probe探针函数 ====================
static GstPadProbeReturn exposure_probe_callback(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
    
    // 记录曝光时间
    ExposureRecorder<frame_data_mono_4_saving>::instance().recordExposureStart();
    
    return GST_PAD_PROBE_OK;
}










// ==================== Appsink回调函数 ====================
static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data) {




    // 计算帧率
    // 临时使用变量(纳秒)
    uint64_t exposure_start = ExposureRecorder<frame_data_mono_4_saving>::instance().getLastExposureStart();



    if(exposure_start - last_refresh_tick >= refresh_epoch * 1e9){

        std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm* now_tm = std::localtime(&now_time_t);

        std::cout << "\n=============== log ================" << std::endl;
        std::cout << "   " << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
        std::cout << "   帧率："<< std::fixed << std::setprecision(1) << frame_counter / (refresh_epoch) << " fps" << std::endl;
        std::cout << "   识别率："<< std::fixed << std::setprecision(1) << (double)detect_count * 100 / frame_counter << " %   ( " << std::to_string(detect_count) << "/" << frame_counter << " )" << std::endl;
        std::cout << "====================================" << std::endl;

        frame_counter = 0;
        detect_count = 0;
        last_refresh_tick = exposure_start;
    }
    frame_counter++;





    
    // 拉取并处理样本
    GstSample* sample = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    

    if (sample) {

        // 获取样本缓冲区
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        frame_data_mono_4_saving frame;
        frame.timestamp_ns = exposure_start;



        // 压入相机画面
        if(Give_img){

            GstMapInfo map;
            if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {


                // 从缓冲区中获取图像数据
                frame.bin_data = std::vector<uint8_t>(map.data, map.data + map.size);
                
                gst_buffer_unmap(buffer, &map);

            }
            else {
                std::cerr << "[!] 无法映射缓冲区图像" << std::endl;
            }
        }


        // ====== 提取置信度最高的检测框 ======
        NvDsBatchMeta* batch_meta = gst_buffer_get_nvds_batch_meta(buffer);

        if (batch_meta && batch_meta->frame_meta_list) {

            NvDsFrameMeta* frame_meta = (NvDsFrameMeta*)batch_meta->frame_meta_list->data;
            
            
            float max_conf = CONF_THRESHOLD;

            NvDsObjectMeta* best_obj = nullptr;

            for (NvDsMetaList* l_obj = frame_meta->obj_meta_list; l_obj; l_obj = l_obj->next) {
                NvDsObjectMeta* obj = (NvDsObjectMeta*)l_obj->data;



                // 过滤置信度低于阈值的检测框
                if (obj->confidence > max_conf) {
                    max_conf = obj->confidence;
                    best_obj = obj;
                }
            }


            
            if (best_obj) {

                //若存在目标则保存当前帧信息
                frame.detected = true;
                frame.bboxes = std::make_tuple( best_obj->rect_params.left,
                                                best_obj->rect_params.top,
                                                best_obj->rect_params.width,
                                                best_obj->rect_params.height,
                                                max_conf );

                detect_count++;
                // std::cout << "[info] 检测到目标，已push该帧数据" << std::endl;
                
            }
            else{
                frame.detected = false;
            }

        } 
        else {
            frame.detected = false;
            std::cout << "[err] 无元数据" << std::endl;
        }
        ExposureRecorder<frame_data_mono_4_saving>::instance().push2History(frame);

        gst_sample_unref(sample);
    }

    else{
        return GST_FLOW_EOS;
    }
    
    return GST_FLOW_OK;
}











// ==================== 辅助函数 ====================
// 绑定probe探针函数到pipeline指定环节
void add_exposure_probe(GstElement* pipeline, const char* element_name) {
    GstElement* element = gst_bin_get_by_name(GST_BIN(pipeline), element_name);

    if (element) {
        GstPad* src_pad = gst_element_get_static_pad(element, "src");
        
        if (src_pad) {
            // 添加probe
            gulong probe_id = gst_pad_add_probe(
                src_pad,
                GST_PAD_PROBE_TYPE_BUFFER,
                exposure_probe_callback,
                nullptr,
                nullptr
            );
            
            std::cout << "已添加曝光时间记录probe (ID: " << probe_id << ")" << std::endl;
            gst_object_unref(src_pad);
        }
        
        gst_object_unref(element);
    }
    else if (!element) {
        // 尝试通过类型查找
        GstIterator* it = gst_bin_iterate_elements(GST_BIN(pipeline));
        GValue value = G_VALUE_INIT;
        
        while (gst_iterator_next(it, &value) == GST_ITERATOR_OK) {
            GstElement* elem = GST_ELEMENT(g_value_get_object(&value));
            const gchar* name = GST_OBJECT_NAME(elem);
            
            // 查找包含"v4l2src"的元素
            if (name && strstr(name, "v4l2src")) {
                element = GST_ELEMENT(gst_object_ref(elem));
                break;
            }
            g_value_unset(&value);
        }
        gst_iterator_free(it);
    }
    else {
        std::cerr << "警告: 未找到v4l2src元素，无法添加曝光probe" << std::endl;
    }
}





// ==================== Ctrl+C 处理 ====================
static gboolean sigint_handler(gpointer user_loop){

    if(working_state.load() == recording)
    {
        working_state.store(stopped);
        std::cout << "等待保存完成..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    GMainLoop* loop = (GMainLoop*)user_loop;
    thread_exit = true;
    g_main_loop_quit(loop);
    return G_SOURCE_REMOVE;
}










// ==================== 主函数 ====================
// 若添加显示窗口掉帧严重, 无法模拟实际部署环境
int main() {

    setenv("GST_PLUGIN_FEATURE_RANK", "jpegenc:NONE", 1);
    gst_init(nullptr, nullptr); // Gstream初始化


    
    // 构建pipeline字符串
    std::stringstream ss;
    ss << "v4l2src name=my_camera device=/dev/video" << 0 << " ";
    ss << "io-mode=4 brightness=24 contrast=76 saturation=100 hue=0 ";
    ss << "do-timestamp=false ";
    ss << "extra-controls=c,"
       << "white_balance_temperature_auto=1,"
       << "gamma=300,"
       << "gain=128,"
       << "sharpness=100,"
       << "exposure_auto=1,"
       << "exposure_absolute=10,"
       << "exposure_auto_priority=1 ";

    ss << "! ";
    ss << "image/jpeg,width=" << img_width << ",height=" << img_height << ",framerate=" << fast_frame_rate << "/1 ";
    ss << "! ";
    ss << "nvv4l2decoder mjpeg=1 ";     // 解码JPEG图像
    ss << "! ";
    ss << "nvvideoconvert ";     // 将数据转移至显存
    ss << "! ";
    ss << "video/x-raw(memory:NVMM), format=NV12, framerate=" << fast_frame_rate << "/1";

    ss << "! ";
    ss << "mux.sink_0 nvstreammux name=mux batch-size=1 width=" << img_width << " height=" << img_height;
    ss << " nvbuf-memory-type=0 ";
    ss << " live-source=1 batched-push-timeout=40000 ";

    ss << "! ";
    ss << "nvinfer name=pgie config-file-path=../resource/config_infer_primary_yolo26.txt ";
    ss << "! ";

       // ========== 插入：nvtracker  ==========
    ss << "nvtracker name=tracker "
       << "ll-lib-file=/opt/nvidia/deepstream/deepstream-6.3/lib/libnvds_nvmultiobjecttracker.so "  // KLT 追踪器
       << "gpu-id=0 tracking-id-reset-mode=2 compute-hw=1 ";
    ss << "! ";
       // ======================================

    ss << "nvjpegenc quality=85 ";
    ss << "! ";
    ss << "appsink name=my_sink emit-signals=true max-buffers=1 drop=true sync=false ";
    
    std::string pipeline_str = ss.str();
    std::cout << "创建pipeline..." << std::endl;
    

    // 创建pipeline
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    
    if (error) {
        std::cerr << "管道创建失败: " << error->message << std::endl;
        g_error_free(error);
        return -1;
    }
    

    // 添加曝光时间记录probe
    add_exposure_probe(pipeline, "my_camera");




    




    // 设置appsink
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "my_sink");
    if (sink) {
        g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), nullptr);
        gst_object_unref(sink);
        std::cout << "已设置appsink回调" << std::endl;
    } else {
        std::cerr << "警告: 无法找到appsink元素" << std::endl;
    }


    

    // 启动pipeline
    std::cout << "启动管道..." << std::endl;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    

    // 运行
    std::cout << "运行"<< std::to_string(run_seconds) <<"秒测试..." << std::endl;
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    
    g_timeout_add_seconds(run_seconds, [](gpointer data) -> gboolean {
        GMainLoop* loop = static_cast<GMainLoop*>(data);
        std::cout << "\n测试时间结束, 正在停止..." << std::endl;
        
        thread_exit = true;
        g_main_loop_quit(loop);
        return FALSE;
    }, loop);








    // 启动后处理线程
    std::thread post_thread(recieve_thread);


    
    // 主循环
    g_unix_signal_add(SIGINT, sigint_handler, loop);
    g_main_loop_run(loop);


    // 主循环结束
    post_thread.join();
    


    // 清理资源
    std::cout << "\n清理资源..." << std::endl;
    gst_element_send_event(pipeline, gst_event_new_eos());
    g_usleep(100000);

    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    



    std::cout << "测试完成！" << std::endl;
    return 0;
}