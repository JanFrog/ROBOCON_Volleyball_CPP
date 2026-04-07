#include <iostream>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <chrono>
#include <queue>
#include <iomanip>
#include <thread>
// #include <nvdsmeta.h>
#include "gstnvdsmeta.h" 
#include <glib-unix.h>

// socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Self defined
#include "exposure_recorder.h"
#include "predictor.h"









// ctrl + 左键 跳转
int main();
void post_process_thread();
void post_record_thread();
static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data);







// ==================== 全局变量 ====================
std::queue<std::chrono::high_resolution_clock::time_point> frame_times;     // 用于存储帧时间戳的队列
const int img_width = 1280, img_height = 1024, fast_frame_rate = 200, slow_frame_rate = 30; // 图像尺寸和帧率
int frame_counter = 0;              // 帧计数器
int detect_count = 0;               // 检测计数器
uint64_t last_refresh_tick = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    // 上一刷新时间（纳秒）
int refresh_epoch = 1;              // 刷新周期（秒）

bool thread_exit = false;           // 线程退出标志
int run_seconds = 600;              // 运行时间（秒）
int64_t timeout_threshold = 320;    // 超时阈值（纳秒）
int64_t timeout_count = 0;          // 超时计数器
float conf = 0.f;                   // 置信度阈值












// ==================== 后处理线程( 滤波+预测 ) ====================
void post_process_thread() {

    // ===================== Socket =====================
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in destAddr{};
    if (sockfd < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
    }
    else{
        destAddr.sin_family = AF_INET;          // IPv4
        destAddr.sin_port = htons(12345);        // port
        destAddr.sin_addr.s_addr = inet_addr("192.168.137.1");
    }
    std::stringstream ss_1, ss_2;
    std::string msg_1, msg_2;



    // ================== Locator 变量 ==================
    const int frame_height = img_height, frame_width = img_width;
    const float radius = 0.106;
    Eigen::Matrix3d MTX;

    MTX << 1.31527123e+03,  0.00000000e+00,  5.82870287e+02,
            0.00000000e+00, 1.31458950e+03,  5.74049534e+02,
            0.00000000e+00, 0.00000000e+00,  1.00000000e+00;

    Eigen::Vector3d locate_result;

    
    
    // 配置Predictor参数
    Predictor_Params predictor_params;
    predictor_params.locator_params.mtx = MTX;
    predictor_params.locator_params.radius = radius;
    predictor_params.locator_params.img_width = frame_width;
    predictor_params.locator_params.img_height = frame_height;

    Predictor predictor(predictor_params);


    
    std::cout << "后处理线程已准备，正在等待数据..." << std::endl;

    uint64_t last_ts = 0;
    frame_data_mono latest;

    while (!thread_exit) {
        
        if (ExposureRecorder<frame_data_mono>::instance().getLatestExposureData(latest)) {
            
            

                if (latest.timestamp_ns == last_ts) continue;   //与上一次检测记录相同

                last_ts = latest.timestamp_ns;
                
                if(std::get<4>(latest.bboxes) > 0.7 && predictor.locate(std::get<0>(latest.bboxes), std::get<1>(latest.bboxes), std::get<2>(latest.bboxes), std::get<3>(latest.bboxes), locate_result)){

                    std::cout<< "locate_result: "<< locate_result.transpose() << std::endl;
                    
                    ss_1 << "0 " << locate_result[0] << ' ' << locate_result[1] << ' ' << locate_result[2] << " y_ 0.01";
                    msg_1 = ss_1.str();

                    ssize_t sentBytes_1 = sendto(sockfd, msg_1.c_str(), msg_1.length(), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
                    
                    if (sentBytes_1 < 0) {
                        std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
                    }

                    ss_1.str("");
                    ss_1.clear();

                    // std::cout << "最新检测记录 - 时间: " << latest.timestamp_ns << "ns, 检测框: ("
                    //     << "left" << std::get<0>(latest.bboxes) << ",\n"
                    //     << "top" << std::get<1>(latest.bboxes) << ",\n"
                    //     << "width" << std::get<2>(latest.bboxes) << ",\n"
                    //     << "height" << std::get<3>(latest.bboxes) << ",\n"
                    //     << "confidence" << std::get<4>(latest.bboxes) << ")\n";
                }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  //最大刷新率 1kHz
    }
}








// 未完成
// ============================ 录制线程 =============================
void post_record_thread(){


    // 立刻声明CV所属线程
    cv::namedWindow("Record", cv::WINDOW_AUTOSIZE);
    
    std::string OPT_DIR = "/home/nvidia/share/Realtime_data";


    // log
    std::cout << "############ 录制模式 ############" << std::endl;
    std::cout << "保存目录: " << OPT_DIR << std::endl;
    std::cout << "录制线程已准备，正在等待数据..." << std::endl;



    // 上一次曝光开始时间（纳秒）
    uint64_t last_ts = 0;
    // 最新一帧图像
    cv::Mat frame;
    // 最新一帧数据
    frame_data_mono latest;


    while (!thread_exit) {



        
        if (ExposureRecorder<frame_data_mono>::instance().getLatestExposureData(latest)) {
            

            if (latest.timestamp_ns == last_ts) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;   //与上一次检测记录相同
            }

            last_ts = latest.timestamp_ns;  // 更新上一次曝光开始时间

            // 复制最新一帧图像
            latest.image.copyTo(frame);


            // 显示最新一帧图像

        }
        else
        {
            frame = cv::Mat::zeros(img_height, img_width, CV_8UC4);
        }
        cv::imshow("Record", frame);
        cv::pollKey();

    }

    cv::destroyAllWindows();
    std::cout << "[GUI 线程] 已退出" << std::endl;

}







// ==================== Probe探针函数 ====================
static GstPadProbeReturn exposure_probe_callback(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
    
    // 记录曝光时间
    ExposureRecorder<frame_data_mono>::instance().recordExposureStart();
    
    return GST_PAD_PROBE_OK;
}










// ==================== Appsink回调函数 ====================
static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data) {

    // 获取videorate元素
    GstElement* videorate = static_cast<GstElement*>(user_data);


    
    // 获取最近一次曝光开始时间
    uint64_t exposure_start = ExposureRecorder<frame_data_mono>::instance().getLastExposureStart();


    // 计算帧率
    
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


    // std::cout << "曝光开始时间: " << exposure_start << "ns\n" << std::flush ;

    
    // 拉取并处理样本
    GstSample* sample = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    

    if (sample) {

        // 获取样本缓冲区
        GstBuffer* buffer = gst_sample_get_buffer(sample);


        // 初始化数据结构
        frame_data_mono data_to_push;
        data_to_push.timestamp_ns = exposure_start;




        // 压入相机画面
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            
            cv::Mat frame(1024, 1280, CV_8UC4, map.data); //BGRx
            // cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);// 转换格式
            data_to_push.image = frame.clone();

            gst_buffer_unmap(buffer, &map);

        }
        else {
            std::cerr << "[!] 无法映射缓冲区图像" << std::endl;
        }


        // ====== 提取置信度最高的检测框 ======
        NvDsBatchMeta* batch_meta = gst_buffer_get_nvds_batch_meta(buffer);

        if (batch_meta && batch_meta->frame_meta_list) {

            NvDsFrameMeta* frame_meta = (NvDsFrameMeta*)batch_meta->frame_meta_list->data;
            
            
            float max_conf = 0.5f;

            NvDsObjectMeta* best_obj = nullptr;

            for (NvDsMetaList* l_obj = frame_meta->obj_meta_list; l_obj; l_obj = l_obj->next) {
                NvDsObjectMeta* obj = (NvDsObjectMeta*)l_obj->data;




                // if (obj->object_id != UNTRACKED_OBJECT_ID) {
                //     g_print("Tracked object ID=%lu, class=%d, conf=%.2f\n",
                //             obj->object_id,
                //             obj->class_id,
                //             obj->confidence);
                // } 
                // else {
                //     g_print("UNTRACKED object! (tracker not working)\n");
                // }



                
                // 过滤异常尺寸（防止"乱飘"的极端bbox）
                // if (obj->rect_params.width < 10 || obj->rect_params.height < 10) continue;
                // if (obj->rect_params.width > frame_meta->source_frame_width * 0.8) continue;
                
                // 置信度阈值
                // if (obj->confidence < 0.5f) continue;
                
                if (obj->confidence > max_conf) {
                    max_conf = obj->confidence;
                    best_obj = obj;
                }
            }


            
            if (best_obj) {

                //若存在目标则保存当前帧信息
                detect_count++;

                data_to_push.detected = true;
                data_to_push.bboxes = std::make_tuple(  best_obj->rect_params.left,
                                                        best_obj->rect_params.top,
                                                        best_obj->rect_params.width,
                                                        best_obj->rect_params.height,
                                                        max_conf );



                if(timeout_count >= timeout_threshold){
                    g_object_set(videorate, "max-rate", fast_frame_rate, nullptr);
                }
                timeout_count = 0;

                    
                // std::cout << "[info] 检测到目标，已push该帧数据" << std::endl;
                
            } 
            else {

                // 无检测目标，累计1000帧后切换为慢速率

                data_to_push.detected = false;

                if(timeout_count >= timeout_threshold){
                    g_object_set(videorate, "max-rate", slow_frame_rate, nullptr);
                }
                else{
                    timeout_count++;
                }
                
            }
        } 
        else {
            data_to_push.detected = false;
            std::cout << "[err] 无元数据" << std::endl;
        }

        ExposureRecorder<frame_data_mono>::instance().push2History(data_to_push);
        

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
    GMainLoop* loop = (GMainLoop*)user_loop;
    thread_exit = true;
    g_main_loop_quit(loop);
    return G_SOURCE_REMOVE;

}










// ==================== 主函数 ====================
int main() {

    // 启动后处理线程
    std::thread post_thread(post_process_thread);
    // std::thread post_thread(post_record_thread);



    gst_init(nullptr, nullptr); // Gstream初始化


    
    // 设置相机参数
    // std::cout << "设置相机参数ing" << std::endl;
    // system("v4l2-ctl -d /dev/video0 -c brightness=24");
    // system("v4l2-ctl -d /dev/video0 -c contrast=76");
    // system("v4l2-ctl -d /dev/video0 -c saturation=100");
    // system("v4l2-ctl -d /dev/video0 -c gain=128");
    // system("v4l2-ctl -d /dev/video0 -c sharpness=100");
    // system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
    // system("v4l2-ctl -d /dev/video0 -c exposure_absolute=10");
    
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
    ss << "nvv4l2decoder mjpeg=1 ";
    ss << "! ";
    ss << "nvvideoconvert ";
    ss << "! ";
    ss << "video/x-raw(memory:NVMM), format=NV12, width=1280, height=1024, framerate=" << fast_frame_rate << "/1";
    ss << "! ";
    ss << "videorate name=videorate0 drop-only=true silent=true ";
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

    ss << "nvvideoconvert " ;
    ss << "! ";
    ss << "video/x-raw, format=BGRx ";
    ss << "! ";
    ss << "appsink name=my_sink emit-signals=true max-buffers=1 drop=true sync=false";

    
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





    // 设置videorate
    GstElement* videorate = gst_bin_get_by_name(GST_BIN(pipeline), "videorate0");
    // 初始化为高性能模式
    g_object_set(videorate, "max-rate", fast_frame_rate, nullptr); 
    




    // 设置appsink
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "my_sink");
    if (sink) {
        g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample), videorate);
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





    
    // 主循环
    g_unix_signal_add(SIGINT, sigint_handler, loop);
    g_main_loop_run(loop);



    // 主循环结束
    post_thread.join();
    


    // 清理资源
    std::cout << "\n清理资源..." << std::endl;
    gst_element_send_event(pipeline, gst_event_new_eos());
    g_usleep(200000);

    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    



    std::cout << "测试完成！" << std::endl;
    return 0;
}