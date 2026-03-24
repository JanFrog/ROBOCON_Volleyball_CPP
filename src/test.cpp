#include <gst/gst.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <chrono>

//2026-3-24 遗留问题：onnx=custom_model depth-mode=NEURAL时 远远达不到60fps (实际约为20fps)


static int run_seconds = 10;


static void sink_callback(GstElement *fpsdisplaysink,
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

    std::stringstream ss;
    ss << "zedsrc name=zedsrc0 camera-fps=60 camera-resolution=3 \
            coordinate-system=5 depth-mode=1 od-enabled=true enable-positional-tracking=true od-enable-tracking=true \
            od-detection-model=6 od-custom-onnx-file=\"/home/nvidia/Projects/Volleyball_cpp/resource/model/zed_yolo11s_640_half.onnx\" \
            od-custom-onnx-dynamic-input-shape-w=640 od-custom-onnx-dynamic-input-shape-h=640";
            //神人隐藏接口 ↑ ↑ ↑
            
    ss << " ! ";
    ss << "queue";
    ss << " ! ";
    ss << "zedodoverlay";
    ss << " ! ";
    ss << "queue";
    ss << " ! ";
    ss << "autovideoconvert";
    ss << " ! ";
    ss << "queue";
    ss << " ! ";
    ss << "fpsdisplaysink name=final_sink";

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
    else{
        std::cout << "管道创建成功" << std::endl;
    }


    GstElement *final_sink = gst_bin_get_by_name(GST_BIN(pipeline), "final_sink");
    if(final_sink){
        // 启用量测信号
        g_object_set(final_sink, "signal-fps-measurements", TRUE, NULL);
        // 连接回调
        g_signal_connect(final_sink, "fps-measurements", G_CALLBACK(sink_callback), nullptr);
        gst_object_unref(final_sink);  // 不再需要额外引用
    }
    else{
        std::cerr << "final_sink 未找到" << std::endl;
        return -1;
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

        g_main_loop_quit(loop);
        return FALSE;
    }, loop);




    // 主循环start !
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