#include "sl/Camera.hpp"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <fstream> 
#include <filesystem>
#include <exception>  
#include <chrono>

using namespace nvinfer1;






// 3.29 遗留问题：用上一时刻的YOLO推理结果映射当前时刻的深度数据



// config

    // ONNX构建配置(如果没有engine文件)
    const bool FP16_MODE = true;            // Orin NX 建议开启 FP16
    const int MAX_WORKSPACE_SIZE = 256;     // MB

    // 输入张量必须静态
    // 模型路径
    std::string ONNX_PATH   = "../resource/model/volleyball_26n_640_zed_300.onnx";
    std::string ENGINE_PATH = "";

    // IO层张量名称
    const char* INPUT_NAME = "images";      // 输入张量名
    const char* OUTPUT_NAME = "output0";    // 输出张量名

    // Pre process
    int img_height = 720;
    int img_width = 1280;
    double SCALER = MIN(double(640) / img_width, double(640) / img_height);

    // OD
    float conf_threshold = 0.8;








// Logger (trt)
class Logger : public ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // suppress info-level messages
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;









// MAIN ==========================================================
int main(){




// cam preparation ===============================================

    // 创建ZED相机对象
    sl::Camera zed;


    // 初始化相机参数
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;      // 720P
    init_params.camera_fps = 60;                                // 60 FPS
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl::UNIT::METER; // Set units in meters
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE; // depth mode
    


    // 打开相机
    sl::ERROR_CODE returned_state = zed.open(init_params);
    if (returned_state > sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << ", exit program...\n";
        return -1;
    }




    // 目标跟踪部分

    // 总配置
    sl::ObjectDetectionParameters detection_parameters;
    detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    detection_parameters.enable_tracking = true; // ID 一致性
    detection_parameters.enable_segmentation = true; // 掩膜mask输出

    if (detection_parameters.enable_tracking) {
        // Set positional tracking parameters
        sl::PositionalTrackingParameters positional_tracking_parameters;
        // Enable positional tracking
        zed.enablePositionalTracking(positional_tracking_parameters);
    }

    // 物体坐标获取配置
    sl::ObjectDetectionRuntimeParameters detection_parameters_rt;
    detection_parameters_rt.detection_confidence_threshold = 25;


    // 启用目标检测功能
    sl::ERROR_CODE zed_error = zed.enableObjectDetection(detection_parameters);
    if (zed_error != sl::ERROR_CODE::SUCCESS) {
        std::cout << "enableObjectDetection: " << zed_error << "\nExit program.";
        zed.close();
        return -1;
    }



    

    

















    

// engine preparation =========================================


    // load a model file
    std::ifstream engine_model_file, onnx_model_file;



    // 创建模型
    ICudaEngine* engine = nullptr;
    IRuntime* runtime = createInferRuntime(logger);
    


    if (ENGINE_PATH != ""){         // 从engine开始构建 ===================
        try
        {
            engine_model_file.open(ENGINE_PATH, std::ios::binary | std::ios::ate);
        }
        catch(const std::exception& e)
        {
            std::cerr << "cannot read file: " << ENGINE_PATH << std::endl;
            return -1;
        }


        // forgive me ...
        Engine_OK:



        if (!engine_model_file.is_open()){
            std::cerr << "cannot load file ! " << std::endl;
            return -1;
        }



        std::streamsize model_size = engine_model_file.tellg();

        if (model_size == -1) {
            std::cerr << "获取文件大小失败" << std::endl;
            return -1;
        }

        engine_model_file.seekg(0, std::ios::beg);


        // 模型比特流 -> 字符向量
        std::vector<char> model_buffer(model_size);
        
        if (!engine_model_file.read(model_buffer.data(), model_size)){
            std::cerr << "读取文件失败" << std::endl;
            return -1;
        }
        // 释放内存
        engine_model_file.close();






        // 创建运行框架
        engine = runtime->deserializeCudaEngine(model_buffer.data(), model_buffer.size());
        
    }



    else if (ONNX_PATH != ""){      // 从onnx开始构建 ============

        size_t last_dot = ONNX_PATH.find_last_of('.');
        std::string tmp_path;
    
        if (last_dot == std::string::npos) {
            // 没有后缀，直接添加 .engine
            tmp_path = ONNX_PATH + ".engine";
        }
        // 改后缀为 .engine
        tmp_path = ONNX_PATH.substr(0, last_dot) + ".engine";


        // 若已存在文件尝试读取
        if (std::filesystem::exists(tmp_path))
        {
            try
            {
                engine_model_file.open(tmp_path, std::ios::binary | std::ios::ate);
                goto Engine_OK; // forgive me ...
            }
            catch(const std::exception& e){
                std::cerr << "Warning! engine cache broken, trying to rebuild engine ...\n";
            }
        }
        



        // 正式开始从 ONNX 文件构建
        std::cout << "Building engine from ONNX: " << ONNX_PATH << std::endl;


        // 创建构建器
        IBuilder* builder = createInferBuilder(logger);

        uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        
        INetworkDefinition* network = builder->createNetworkV2(flag);

        nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);


        // load
        if(!parser->parseFromFile(ONNX_PATH.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING)))
        {
            // find error
            std::cerr << "Failed to parse ONNX file!" << std::endl;
            for (int32_t i = 0; i < parser->getNbErrors(); ++i)
                std::cerr << parser->getError(i)->desc() << std::endl;

            delete parser;
            delete network;
            delete builder;

            return -1;
        }


        // 转换配置
        IBuilderConfig* config = builder->createBuilderConfig();
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, static_cast<size_t>(MAX_WORKSPACE_SIZE) * 1024 * 1024);

        if(FP16_MODE)
            config->setFlag(BuilderFlag::kFP16);




        std::cout << "Building... (may take 1-5 minutes)" << std::endl;
        auto t1 = std::chrono::high_resolution_clock::now();
        
        IHostMemory* serialized = builder->buildSerializedNetwork(*network, *config);

        auto t2 = std::chrono::high_resolution_clock::now();

        if(!serialized){
            std::cerr << "Build fail ! " << std::endl;
            return -1;
        }


        // 保存

        engine = runtime->deserializeCudaEngine(serialized->data(), serialized->size());


        std::ofstream out(tmp_path, std::ios::binary);
        out.write((char*)serialized->data(), serialized->size());
        out.close();


        std::cout << "Saved to: " << tmp_path << std::endl;

        delete serialized;
        delete parser;
        delete network;
        delete config;
        delete builder;

    }
    else{

        std::cout<<"model file not selected !"<<std::endl;
        zed.close();
        exit(0);
    }





    // 暴露cuda接口
    IExecutionContext *context = engine->createExecutionContext();


    // IO张量维度
    Dims input_dims = engine->getTensorShape(INPUT_NAME);   // v2 API
    Dims output_dims = engine->getTensorShape(OUTPUT_NAME);



    // 打印维度信息（调试用）
    // input shape
    std::cout << "Input shape: [";
    for (int i = 0; i < input_dims.nbDims; i++) {
        std::cout << input_dims.d[i] << (i < input_dims.nbDims-1 ? "," : "");
    }
    std::cout << "]" << std::endl;

    // output shape
    std::cout << "Output shape: [";
    for (int i = 0; i < output_dims.nbDims; i++) {
        std::cout << output_dims.d[i] << (i < output_dims.nbDims-1 ? "," : "");
    }
    std::cout << "]" << std::endl;



    // 计算IO缓冲区大小
    // I
    size_t input_size = 1;
    for (int i = 0; i < input_dims.nbDims; i++) {
        input_size *= input_dims.d[i];
    }
    input_size *= sizeof(float);

    // O
    size_t output_size = 1;
    for (int i = 0; i < output_dims.nbDims; i++) {
        output_size *= output_dims.d[i];
    }
    output_size *= sizeof(float);

    std::cout << "Input buffer: " << input_size << " bytes" << std::endl;
    std::cout << "Output buffer: " << output_size << " bytes" << std::endl;




    // 分配显存
    void* d_input = nullptr;
    void* d_output = nullptr;
    cudaMalloc(&d_input, input_size);
    cudaMalloc(&d_output, output_size);



    // 分配给runtime
    context->setTensorAddress(INPUT_NAME, d_input);
    context->setTensorAddress(OUTPUT_NAME, d_output);




    // 创建cuda流
    cudaStream_t frame_stream;
    cudaStreamCreate(&frame_stream);














    







    

// Main Loop ================================================

    sl::Mat left_BGR;   // 左目图像
    sl::Mat depth_map;  // 深度图像
    cv::Mat cv_img;   // cv图像(8UC3)
    cv::Mat resized_img;


    // 新尺寸
    int new_width   = int(img_width * SCALER);
    int new_height  = int(img_height * SCALER);
    cv::Size new_size(new_width, new_height);

    // 创建 padding 画布
    cv::Mat img_padded(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));

    // 填充位移
    int pad_w = int((640 - new_width) / 2);
    int pad_h = int((640 - new_height) / 2);
    std::cout << "[padding info] pad_W:" << pad_w << " pad_H:" << pad_h << std::endl; 

    // 归一化图像
    cv::Mat img_normalized;


    // 通道变换用：
    std::vector<float> h_input(640 * 640 * 3, 114.f);
    size_t space_input_cost = 640 * 640 * sizeof(float);
    std::vector<cv::Mat> channels;
    // CPU输出缓冲区（后处理用）
    std::vector<float> h_output(output_size / sizeof(float));



    // 检测目标 (分配内存)
    sl::Objects volleyballs;











    // 初始计时
    std::chrono::time_point last_tick = std::chrono::high_resolution_clock::now();

    // 帧数记录 (用于计算帧率)
    int frame_counter = 0;


    for(int i = 0; i < 1000; i++, frame_counter++)
    {

        // 计算帧率
        if(frame_counter >= 100){

            frame_counter = 0;
            std::chrono::time_point now_tick = std::chrono::high_resolution_clock::now();

            std::cout << "FPS:"<< 100 * 1000000 / std::chrono::duration_cast<std::chrono::microseconds>(now_tick - last_tick).count() << std::endl;
            last_tick = now_tick;
        }




    // YOLO infer add to Queue =======================================

        // 异步拷贝至显存
        cudaMemcpyAsync(d_input, h_input.data(), h_input.size() * sizeof(float), cudaMemcpyHostToDevice, frame_stream);

        // ###### 推理 ######
        context->enqueueV3(frame_stream);

        // 将结果拷贝回内存
        cudaMemcpyAsync(h_output.data(), d_output, output_size, cudaMemcpyDeviceToHost, frame_stream);

    // Done ==================================








        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

            zed.retrieveImage(left_BGR, sl::VIEW::LEFT_BGR); // left image
            // format = U8C3 BGR
        }
        else continue;






    // Pre Handle image =================================

        // 缩放  640 * 640
        cv_img = cv::Mat(720, 1280, CV_8UC3, left_BGR.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::resize(cv_img, resized_img, new_size);
        // padding with 114:114:114 GRAY
        resized_img.copyTo(img_padded(cv::Rect(pad_w, pad_h, new_width, new_height)));


        // 归一化
        img_padded.convertTo(img_normalized, CV_32FC3, 1.0 / 255.0);


        // 通道顺序变换 HWC → NCHW
        channels.clear();
        cv::split(img_normalized, channels);

        for(int i = 0; i < 3; i++)
        {
            // 内存拷贝(交换通道顺序)
            memcpy(h_input.data() + i * 640 * 640, channels[i].data, space_input_cost);
        }


    // Pre Handle Done =================================





    // get last result =================================

            // center_x     = h_output[i * 6];
            // center_y     = h_output[i * 6 + 1];
            // box.width    = h_output[i * 6 + 2];
            // box.height   = h_output[i * 6 + 3];
            // box.obj_conf = h_output[i * 6 + 4];
            // box.cls_id   = h_output[i * 6 + 5];

        cudaStreamSynchronize(frame_stream);

        float max_obj_conf = 0;
        int max_j = -1;
        
        // 扫描所有结果 保留置信度最大目标
        for(int j = 0; j < 300; j++){
            
            if (double(h_output[j * 6 + 4]) > max_obj_conf){
                max_obj_conf = double(h_output[j * 6 + 4]);
                max_j = j;
            }
        }


        if(max_obj_conf > conf_threshold && max_j >= 0)
        {

            sl::CustomBoxObjectData tmp;
            std::vector<sl::CustomBoxObjectData> Zed_od_input;
            std::vector<sl::uint2> box_tmp;
            tmp.unique_object_id = sl::generate_unique_id();

            // 提取bbox (x, y, w, h)
            int x1, y1, x2, y2;
            x1 = (double(h_output[max_j * 6]    ) - pad_w) / SCALER; 
            y1 = (double(h_output[max_j * 6 + 1]) - pad_h) / SCALER;
            x2 = (double(h_output[max_j * 6 + 2]) - pad_w) / SCALER;
            y2 = (double(h_output[max_j * 6 + 3]) - pad_h) / SCALER;


            cv::rectangle(cv_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 3);
            

            // 转成bounding 2D (vector)
            box_tmp.push_back(sl::uint2(x1, y1));
            box_tmp.push_back(sl::uint2(x2, y1));
            box_tmp.push_back(sl::uint2(x2, y2));
            box_tmp.push_back(sl::uint2(x1, y2));


            // make BOD
            tmp.unique_object_id    = sl::generate_unique_id();
            tmp.probability         = (double)h_output[max_j * 6 + 4];
            tmp.label               = (int)h_output[max_j * 6 + 5];
            tmp.bounding_box_2d     = box_tmp;
            tmp.is_grounded         = false;

            Zed_od_input.push_back(tmp);

            zed.ingestCustomBoxObjects(Zed_od_input);

    // get last result Done ============================










        // get depth =======================================

            zed.retrieveObjects(volleyballs, detection_parameters_rt); 

            for(auto object : volleyballs.object_list)
                std::cout << object.id << ": " << object.position << std::endl;

        // get depth Done ==================================



        }


        // 显示 (可选)
        cv::imshow("wow", cv_img);
        cv::waitKey(1);
    }



























// Free resources ===========================================

    // 清理GPU缓存
    cudaFree(d_input);
    cudaFree(d_output);


    zed.close();
    
    return 0;
}