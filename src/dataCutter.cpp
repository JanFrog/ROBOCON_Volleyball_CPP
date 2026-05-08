# include <iostream>
# include <fstream>
# include <vector>
# include <opencv2/opencv.hpp>
# include <filesystem>
# include <string>

# pragma pack(push, 1)



struct frame_header{
    uint64_t time_stamp; // 时间戳  
    bool isDetected; // 是否有检测框
    float bbox[5]; // bbox[left, top, width, height, conf]
    uint32_t dSize; // 数据大小
};
# pragma pack(pop)

struct frame_data{
    frame_header header;
    std::vector<uint8_t> data; // 数据
};

struct segmentToDelete{
    int start;
    int end;
};

std::vector<segmentToDelete> deleteList;
std::vector<size_t> frameOffsets;

int currentFrame = 0; // 当前帧
bool isPlaying = true; // 是否正在播放（空格切换）
bool needsJump = false; // 是否需要跳帧
bool isUpdatingTrackbar = false; // 是否正在更新进度条
int L_down = 0;         // 左键点击次数
int deleteStart = -1;

std::ifstream file;

void onTrackbar(int pos, void*){ // 进度条回调函数
    // 忽略更新进度条产生的回调
    if (isUpdatingTrackbar) return;

    // 将进度移到进度条当前位置
    currentFrame = pos;
    needsJump = true;
}

void onMouse(int event, int, int, int, void*) { // 鼠标回调函数
    if (event != cv::EVENT_LBUTTONDOWN) return;

    L_down++;
    if (L_down == 1) {
        deleteStart = currentFrame;
        std::cout << "Delete start: " << deleteStart << std::endl;
    }
    else if (L_down == 2) {
        if (deleteStart < currentFrame) {
            deleteList.push_back({deleteStart, currentFrame});
            L_down = 0;
            std::cout << "Delete end: " << currentFrame << std::endl;
            std::cout<< "Length: " << currentFrame - deleteStart << std::endl;
        }
        else {
            std::cout << "Error: Invalid delete range!" << std::endl;
            L_down = 0;
        }
    }
}

void drawVisualProgressBar(cv::Mat& img, int currentFrame, int totalFrames, const std::vector<segmentToDelete>& deleteList) {
    // 绘制进度条
    int barHeight = 15;
    int barWidth = img.cols;
    
    cv::Rect bgRect(0, img.rows - barHeight, barWidth, barHeight);
    cv::rectangle(img, bgRect, cv::Scalar(50, 50, 50), -1);

    for (const auto& range : deleteList) {
        int x_start = (int)((double)range.start / totalFrames * barWidth);
        int x_end = (int)((double)range.end / totalFrames * barWidth);
        cv::Rect deleteRect(x_start, img.rows - barHeight, x_end - x_start, barHeight);
        cv::rectangle(img, deleteRect, cv::Scalar(0, 0, 255), -1);
    }

    int cursorX = (int)((double)currentFrame / totalFrames * barWidth);
    cv::line(img, cv::Point(cursorX, img.rows - barHeight), 
                  cv::Point(cursorX, img.rows), cv::Scalar(255, 255, 255), 2);
}

int main(){

    std::filesystem::path input_path = "/home/nvidia/share/Realtime_data/2026-05-02_12-03-31/data.bin";
    std::filesystem::path output_path = "/home/nvidia/share/Realtime_data/2026-05-02_12-03-31/data_editted.bin";

    file.open(input_path, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file" << std::endl;
        return -1;
    }

    // 先遍历文件得到总帧数和索引
    while (!file.eof()) {
        size_t offset = file.tellg();
        frame_header header;
        file.read((char*)&header, sizeof(frame_header));
        if (file.eof()) break;
        frameOffsets.push_back(offset);
        file.seekg(header.dSize, std::ios::cur); // 跳过数据部分
    }

    if (frameOffsets.empty()) {
        std::cerr << "Error: No frames found" << std::endl;
        file.close();
        return -1;
    }

    file.clear();
    file.seekg(frameOffsets[0]); // 从第一帧开始播放
    int totalFrames = frameOffsets.size(); // 获取总帧数

    cv::namedWindow("frame");
    cv::createTrackbar("Progress", "frame", nullptr, totalFrames - 1, onTrackbar, &currentFrame);
    cv::setMouseCallback("frame", onMouse);

    // 监听循环
    while (true) { 
        bool jumpedThisLoop = false; // 标记本次循环是否发生了跳帧

        if (file.eof()){
            std::cout << "End Of File!" << std::endl;
            needsJump = true;
            currentFrame = 0;
        }

        if (needsJump){
           // 仅在此处执行跳帧操作
            file.clear();
            file.seekg(frameOffsets[currentFrame]);
            needsJump = false;
            jumpedThisLoop = true;
        }

        int key = cv::waitKey(1);

        if (key == ' '){
            isPlaying = !isPlaying;
        }
        else if (key == 'd'){ // 前进一帧
            needsJump = true;
            currentFrame = currentFrame + 1;
            std::cout << "Next frame!" << std::endl;
        }
        else if (key == 'a'){ // 后退一帧
            needsJump = true;
            currentFrame = currentFrame - 1;
            std::cout << "Previous frame!" << std::endl;
        }
        else if (key == 'c'){ // 清除上一段
            deleteList.pop_back();
            std::cout << "Last segment cleared!" << std::endl;
        }
        else if (key == 'q'){ // 退出
            file.close();
            break;
        }
        else if (key == 's'){ // 保存
            // 保存更改
        std::ifstream src(input_path, std::ios::binary);
        std::ofstream dest(output_path, std::ios::binary);

        if (!src.is_open() || !dest.is_open()) {
            std::cerr << "Error: Failed to open file" << std::endl;
            return -1;
        }

        for (int i = 0; i < totalFrames; i++){ // 遍历所有帧
            bool shouldDelete = false;
            for (const auto& segment : deleteList){ // 判断是否在删除列表内
                if (i >= segment.start && i <= segment.end){
                    shouldDelete = true;
                    break;
                }
            }
            if (!shouldDelete){
                src.seekg(frameOffsets[i]);
                frame_header header;
                src.read((char*)&header, sizeof(frame_header)); // 读帧头
                dest.write((char*)&header, sizeof(frame_header)); // 写帧头
                if (src.eof()) break;
                std::vector<uint8_t> data(header.dSize);
                src.read((char*)data.data(), header.dSize); // 读数据
                dest.write((char*)data.data(), header.dSize); // 写数据
            }
        }

        src.close();
        dest.close();
        cv::destroyAllWindows();

        std::cout<<"Done!"<<std::endl;

        return 0;
        }
        else if (isPlaying || jumpedThisLoop){
            // 正常播放或跳帧后继续播放
            frame_data frame;
            file.read((char*)&frame.header, sizeof(frame_header));
            frame.data.resize(frame.header.dSize);
            file.read((char*)frame.data.data(), frame.header.dSize);

            cv::Mat img = cv::imdecode(frame.data, cv::IMREAD_COLOR); // jpeg转cv

            if(img.empty()){
                std::cerr << "Error: Failed to decode image" << std::endl;
                continue;
            }

            drawVisualProgressBar(img, currentFrame, totalFrames, deleteList);

            for(const auto& segment : deleteList){ // 删除区域可视化
                if (currentFrame >= segment.start && currentFrame <= segment.end) {
                    cv::rectangle(img, cv::Rect(0, 0, img.cols, img.rows), cv::Scalar(0, 0, 255), 4);
                    cv::putText(img, "DELETING", cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 0, 255), 3);
                }
            }

            if (frame.header.isDetected) { // 检测框可视化
                cv::Rect box(frame.header.bbox[0], frame.header.bbox[1], frame.header.bbox[2], frame.header.bbox[3]);
                cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
            }

            cv::resize(img, img, cv::Size(640, 480));
            cv::imshow("frame", img);

            isUpdatingTrackbar = true;
            cv::setTrackbarPos("Progress", "frame", currentFrame); // 更新进度条位置
            isUpdatingTrackbar = false;

            if (isPlaying) { // 仅在正常播放时才进行帧数递增
                currentFrame++;
            }
        }
    }

    cv::destroyAllWindows();
    cv::waitKey(1);

    return 0;
}