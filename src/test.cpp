#include <gst/gst.h>
#include "gstnvdsmeta.h" 
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <chrono>




#define run_seconds 20
#define pipeline_option 2








// ss_1 问题：onnx=custom_model depth-mode=NEURAL时 远远达不到60fps (实际约为20fps)
/*
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.12.0) /home/nvidia/opencv-4.12.0/modules/core/src/matrix_wrap.cpp:1219: error: (-215:Assertion failed) !fixedSize() || ((Mat*)obj)->size.operator()() == Size(_cols, _rows) in function 'create'

Aborted (core dumped)


*/



static void display_sink_callback(GstElement *fpsdisplaysink,
                                gdouble fps,
                                gdouble droprate,
                                gdouble avgfps,
                                gpointer user_data) {

    // 简单打印，你可以将数据保存到全局变量或 user_data
    std::cout << "[FPS] current: " << fps
              << " | drop: " << droprate
              << " | avg: " << avgfps << std::endl;
}




int main(){

    gst_init(nullptr, nullptr);

    std::stringstream ss_1, ss_2;
    /*
    zedsrc properties:

    depth-mode: Depth Mode
        flags: readable, writable
        Enum "GstZedsrcDepthMode" Default: 0, "NONE"
            (6): NEURAL_PLUS      - More accurate Neural disparity estimation, Requires AI module.
            (5): NEURAL           - End to End Neural disparity estimation, requires AI module
            (3): ULTRA            - Computation mode favorising edges and sharpness. Requires more GPU memory and computation power.
            (2): QUALITY          - Computation mode designed for challenging areas with untextured surfaces.
            (1): PERFORMANCE      - Computation mode optimized for speed.
            (0): NONE             - This mode does not compute any depth map. Only rectified stereo images will be available.
    
            
    stream-type: Image stream type
        flags: readable, writable
        Enum "GstZedSrcCoordSys" Default: 0, "Left image [BGRA]"
            (0): Left image [BGRA] - 8 bits- 4 channels Left image
            (1): Right image [BGRA] - 8 bits- 4 channels Right image
            (2): Stereo couple up/down [BGRA] - 8 bits- 4 channels bit Left and Right
            (3): Depth image [GRAY16_LE] - 16 bits depth
            (4): Left and Depth up/down [BGRA] - 8 bits- 4 channels Left and Depth(image)
            
            
    camera-resolution: Camera Resolution
        flags: readable, writable
        Enum "GstZedSrcRes" Default: 6, "Default value for the camera model"
            (0): HD2K (USB3)      - 2208x1242
            (1): HD1080 (USB3/GMSL2) - 1920x1080
            (2): HD1200 (GMSL2)   - 1920x1200
            (3): HD720 (USB3)     - 1280x720
            (4): SVGA (GMSL2)     - 960x600
            (5): VGA (USB3)       - 672x376
            (6): Default value for the camera model - Automatic

    */





    // pipeline 配置:

    // option 1
    {
        ss_1 << "zedsrc name=zedsrc0 camera-fps=60 camera-resolution=3 \
                coordinate-system=5 depth-mode=1 od-enabled=true enable-positional-tracking=true od-enable-tracking=true \
                od-detection-model=6 od-custom-onnx-file=\"/home/nvidia/Projects/Volleyball_cpp/resource/model/zed_yolo11s_640_half.onnx\" \
                od-custom-onnx-dynamic-input-shape-w=640 od-custom-onnx-dynamic-input-shape-h=640";
                //神人隐藏接口 ↑ ↑ ↑
                
        ss_1 << " ! ";
        ss_1 << "queue";
        ss_1 << " ! ";
        ss_1 << "zedodoverlay";
        ss_1 << " ! ";
        ss_1 << "queue";
        ss_1 << " ! ";
        ss_1 << "autovideoconvert";
        ss_1 << " ! ";
        ss_1 << "queue";
        ss_1 << " ! ";
        ss_1 << "fpsdisplaysink name=final_sink0";
    }


    // option 2
    {
        ss_2 << "zedsrc name=zedsrc0 camera-fps=60 camera-resolution=3 stream-type=4 coordinate-system=5 depth-mode=1 od-enabled=false";
        ss_2 << " ! ";
        ss_2 << "zeddemux name=demux0 is-mono=false is-depth=true stream-data=true ";

        // 细分支 1
        {
            ss_2 << "demux0.src_left";
            ss_2 << " ! ";
            ss_2 << "videoconvert";
            ss_2 << " ! ";
            ss_2 << "video/x-raw, format=(string)BGRx, width="<< 1280 <<", height="<< 720 <<", framerate=" << 60 << "/1";
            ss_2 << " ! ";
            ss_2 << "queue";
            ss_2 << " ! ";
            ss_2 << "nvvideoconvert";
            ss_2 << " ! ";
            ss_2 << "video/x-raw(memory:NVMM), format=(string)NV12";
            ss_2 << " ! ";
            ss_2 << "queue";
            ss_2 << " ! ";
            ss_2 << "mux.sink_0 nvstreammux name=mux batch-size=1 nvbuf-memory-type=0 live-source=1 batched-push-timeout=40000 width=" << 1280 << " height=" << 720;
            ss_2 << " ! ";
            ss_2 << "nvinfer name=pgie config-file-path=../resource/config_infer_primary_yolo26.txt";

            // ========== 插入：nvtracker  ==========
            // ss_2 << "nvtracker name=tracker"
            // << " ll-lib-file=/opt/nvidia/deepstream/deepstream-6.3/lib/libnvds_nvmultiobjecttracker.so"  // KLT 追踪器
            // << " gpu-id=0 tracking-id-reset-mode=2 compute-hw=1";
            // ss_2 << " ! ";
            // ======================================
            
            // ss_2 << "queue";
            // ss_2 << " ! ";
            // ss_2 << "nvdsosd ";
            
            ss_2 << " ! ";
            ss_2 << "appsink name=infer_sink sync=false max-buffers=1 drop=true ";
        }

        // 细分支 2
        {
            ss_2 << "demux0.src_aux";
            ss_2 << " ! ";
            ss_2 << "appsink name=aux_sink sync=false max-buffers=1 drop=true ";
        }

        // 细分支 3
        {
            ss_2 << "demux0.src_data";
            ss_2 << " ! ";
            ss_2 << "appsink name=data_sink sync=false max-buffers=1 drop=true ";
        }
    }






    // 选择pipeline配置
    std::string pipeline_str;

    switch(pipeline_option){
        case 1:
            pipeline_str = ss_1.str();
            break;
            
        case 2:
            pipeline_str = ss_2.str();
            break;
    }

    std::cout << "Pipeline string: " << pipeline_str << std::endl;
    std::cout << "创建pipeline..." << std::endl;
    


    // 创建pipeline
    GError* error = nullptr;
    GstElement *pipeline = gst_parse_launch_full(pipeline_str.c_str(), nullptr, GST_PARSE_FLAG_NONE, &error);
    if (error) {
        std::cerr << "Parse error: " << error->message << std::endl;
        // 如果 error->message 只是 "syntax error"，还可以尝试打印 error->code
        std::cerr << "Error code: " << error->code << std::endl;
        g_error_free(error);
        return -1;
    }














    // from option 1
    // // 绑定sink到回调函数
    // GstElement *final_sink = gst_bin_get_by_name(GST_BIN(pipeline), "final_sink0");

    // if(final_sink){
    //     // 启用量测信号
    //     g_object_set(final_sink, "signal-fps-measurements", TRUE, NULL);
    //     // 连接回调
    //     g_signal_connect(final_sink, "fps-measurements", G_CALLBACK(display_sink_callback), nullptr);
    //     gst_object_unref(final_sink);  // 不再需要额外引用
    // }
    // else{
    //     std::cerr << "未找到display_sink元素" << std::endl;
    //     return -1;
    // }






    // 启动pipeline
    std::cout << "启动管道..." << std::endl;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    







    // 设置loop流程
    std::cout << "运行"<< std::to_string(run_seconds) <<"秒测试..." << std::endl;
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    
    g_timeout_add_seconds(run_seconds, [](gpointer data) -> gboolean {
        GMainLoop* loop = static_cast<GMainLoop*>(data);
        std::cout << "\n测试时间结束, 正在停止..." << std::endl;

        g_main_loop_quit(loop);
        return FALSE;
    }, loop);









    // 主循环 start !
    g_main_loop_run(loop);





    // 清理资源 :)
    std::cout << "\n清理资源..." << std::endl;
    gst_element_send_event(pipeline, gst_event_new_eos());
    g_usleep(200000);

    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    





    std::cout << "测试完成！" << std::endl;
    return 0;
}