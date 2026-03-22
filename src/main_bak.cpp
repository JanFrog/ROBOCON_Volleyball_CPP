#include<iostream>
#include <gst/gst.h>
#include<opencv2/opencv.hpp>
#include<string>
#include<sstream>
#include<chrono>
#include<queue>
#include<iomanip>
#include<thread>
#include <nvdsmeta.h>
#include "gstnvdsmeta.h" 
#include "exposure_recorder.h"





// ==================== 全局变量 ====================
std::queue<std::chrono::high_resolution_clock::time_point> frame_times;
const int FRAME_WINDOW = 100;
static bool SHOW = true;
int frame_counter = 100;
uint64_t last_frame_time = 0;
bool thread_exit = false;




// ==================== 后处理线程 ====================
void post_process_thread() {
    int processed_count = 0;

    std::cout << "后处理线程已启动，正在等待数据..." << std::endl;

    while (!thread_exit) {

        auto history = ExposureRecorder::instance().getExposureHistory();
        
        if (!history.empty()) {
            auto& latest = history.back();
            std::cout << "最新曝光记录 - 时间: " << latest.timestamp_ns << "ns, 检测框: ("
                      << std::get<0>(latest.bboxes) << ", "
                      << std::get<1>(latest.bboxes) << ", "
                      << std::get<2>(latest.bboxes) << ", "
                      << std::get<3>(latest.bboxes) << ", "
                      << std::get<4>(latest.bboxes) << ")\n";
        }
        else {
            std::cout << "当前无曝光记录\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 每100ms检查一次退出标志
    }
}




// ==================== Probe回调函数 ====================
static GstPadProbeReturn exposure_probe_callback(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
    
    ExposureRecorder::instance().recordExposureStart();
    
    return GST_PAD_PROBE_OK;
}




// ==================== Appsink回调函数 ====================
static GstFlowReturn on_new_sample(GstElement* sink, gpointer data) {

    
    // 获取最近一次曝光开始时间
    uint64_t exposure_start = ExposureRecorder::instance().getLastExposureStart();


    // 计算帧率
    if(frame_counter >= 100){
        std::cout<< "帧率："<< std::fixed << std::setprecision(1) << (double)1e11 / (exposure_start - last_frame_time) << " fps\n";
        frame_counter = 0;
        last_frame_time = exposure_start;
    }
    frame_counter++;


    // std::cout << "曝光开始时间: " << exposure_start << "ns\n" << std::flush ;

    
    // 拉取并处理样本
    GstSample* sample = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    
    if (sample) {

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        frame_data data_to_push;
        data_to_push.timestamp_ns = exposure_start;


        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            
            cv::Mat frame(1024, 1280, CV_8UC4, map.data); //BGRx
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);// 转换格式
            data_to_push.image = frame.clone();

            gst_buffer_unmap(buffer, &map);
        }


        // ====== 提取置信度最高的检测框 ======
        NvDsBatchMeta* batch_meta = gst_buffer_get_nvds_batch_meta(buffer);

        if (batch_meta && batch_meta->frame_meta_list) {

            NvDsFrameMeta* frame_meta = (NvDsFrameMeta*)batch_meta->frame_meta_list->data;
            
            // 单次遍历找出最高置信度
            float max_conf = 0.0f;
            float best_x = 0, best_y = 0, best_w = 0, best_h = 0;
            
            for (NvDsMetaList* l_obj = frame_meta->obj_meta_list; l_obj; l_obj = l_obj->next) {

                NvDsObjectMeta* obj = (NvDsObjectMeta*)l_obj->data;

                if (obj->confidence > max_conf) {

                    max_conf = obj->confidence;
                    best_x = obj->rect_params.left;
                    best_y = obj->rect_params.top;
                    best_w = obj->rect_params.width;
                    best_h = obj->rect_params.height;
                }
            }
            
            if (max_conf > 0) {

                //若存在目标则保存当前帧信息

                data_to_push.detected = true;
                data_to_push.bboxes = std::make_tuple(best_x, best_y, best_w, best_h, max_conf);
                    
                // ExposureRecorder::instance().push2History(exposure_start, frame, std::make_tuple(best_x, best_y, best_w, best_h, max_conf));
                // std::cout << "[✓] 检测到目标，已push该帧数据" << std::endl;
                
            } 
            else {
                data_to_push.detected = false;
                // std::cout << "[×] 无检测" << std::endl;
            }
        } 
        else {
            data_to_push.detected = false;
            std::cout << "[!] 无元数据" << std::endl;
        }

        ExposureRecorder::instance().push2History(data_to_push);
        

        gst_sample_unref(sample);
    }
    
    return GST_FLOW_OK;
}




// ==================== 辅助函数 ====================
// 目的：绑定probe回调函数到pipeline指定环节
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





// ==================== 主函数 ====================
int main() {
    gst_init(nullptr, nullptr);
    
    const int img_width = 1280, img_height = 1024, frame_rate = 200;
    
    // 设置相机参数
    std::cout << "设置相机参数ing" << std::endl;
    system("v4l2-ctl -d /dev/video0 -c brightness=24");
    system("v4l2-ctl -d /dev/video0 -c contrast=76");
    system("v4l2-ctl -d /dev/video0 -c saturation=100");
    system("v4l2-ctl -d /dev/video0 -c gain=128");
    system("v4l2-ctl -d /dev/video0 -c sharpness=100");
    system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
    system("v4l2-ctl -d /dev/video0 -c exposure_absolute=10");
    
    // 构建pipeline字符串
    std::stringstream ss;
    ss << "v4l2src name=my_camera device=/dev/video" << 0 << " ";
    ss << "io-mode=2 ";
    ss << "do-timestamp=false ";
    ss << "! ";
    ss << "image/jpeg,width=" << img_width << ",height=" << img_height << ",framerate=" << frame_rate << "/1 ";
    ss << "! ";
    ss << "nvv4l2decoder mjpeg=1 ";
    ss << "! ";
    ss << "nvvideoconvert ";
    ss << "! ";
    ss << "video/x-raw(memory:NVMM), format=NV12, width=1280, height=1024, framerate=" << frame_rate << "/1";
    ss << "! ";
    ss << "mux.sink_0 nvstreammux name=mux batch-size=1 width=" << img_width << " height=" << img_height;
    ss << " nvbuf-memory-type=0 ";
    ss << " live-source=1 batched-push-timeout=40000 ";
    ss << "! ";
    ss << "nvinfer name=pgie config-file-path=../resource/config_infer_primary_yolo26.txt ";
    ss << "! ";
    ss << "nvdsosd name=sosd ";
    ss << "! ";
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
    std::cout << "运行15秒测试..." << std::endl;
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    
    g_timeout_add_seconds(15, [](gpointer data) -> gboolean {
        GMainLoop* loop = static_cast<GMainLoop*>(data);
        std::cout << "\n测试时间结束, 正在停止..." << std::endl;
        thread_exit = true;
        g_main_loop_quit(loop);
        return FALSE;
    }, loop);

    // 启动后处理线程
    std::thread post_thread(post_process_thread);
    

    g_main_loop_run(loop);

    post_thread.join();

    


    // 清理资源
    std::cout << "\n清理资源..." << std::endl;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    
    std::cout << "测试完成！" << std::endl;
    return 0;
}