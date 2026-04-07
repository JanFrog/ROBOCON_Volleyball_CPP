
cmake_minimum_required(VERSION 3.16)

# # 消除 FindCUDA 和 exec_program 废弃警告
# cmake_policy(SET CMP0146 NEW)
# cmake_policy(SET CMP0153 NEW)

project(Volleyball_cpp LANGUAGES CXX CUDA)








# --- 基础配置 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 默认构建类型（允许用户通过 -DCMAKE_BUILD_TYPE 覆盖）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (Debug/Release)" FORCE)
endif()

# Jetson Orin NX: Ampere 架构 (SM 87)
set(CMAKE_CUDA_ARCHITECTURES "87" CACHE STRING "CUDA architecture for Jetson Orin NX (Ampere)")











# --- 查找依赖 ---
# 标准 CMake 包
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ZED REQUIRED)
include_directories(${ZED_INCLUDE_DIRS})

 


# TensorRT (DeepStream 未提供标准 CMake config，需手动查找)
find_path(TensorRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include /usr/include/aarch64-linux-gnu
    NO_DEFAULT_PATH
)
find_library(TensorRT_LIB nvinfer
    PATHS /usr/lib/aarch64-linux-gnu
    NO_DEFAULT_PATH
)


# GStreamer
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-video-1.0)

# DeepStream 库 (手动查找)
set(DEEPSTREAM_LIB_PATHS
    /usr/lib/aarch64-linux-gnu
    /opt/nvidia/deepstream/deepstream-6.3/lib
)


find_library(NVDSGST_HELPER_LIB nvdsgst_helper PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(NVDS_META_LIB nvds_meta PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(NVDSGST_META_LIB nvdsgst_meta PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(NVBUFSURFACE_LIB nvbufsurface PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(NVDS_INFER_LIB nvds_infer PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(NVDS_UTILS_LIB nvds_utils PATHS ${DEEPSTREAM_LIB_PATHS} NO_DEFAULT_PATH)
find_library(SOCKETCAN_LIB socketcan)

if(NOT NVDSGST_HELPER_LIB OR NOT NVDS_META_LIB OR NOT NVDSGST_META_LIB OR
   NOT NVBUFSURFACE_LIB OR NOT NVDS_INFER_LIB OR NOT NVDS_UTILS_LIB)
    message(FATAL_ERROR "One or more DeepStream libraries not found!")
endif()

set(DEEPSTREAM_INCLUDE_DIR "/opt/nvidia/deepstream/deepstream-6.3/sources/includes")







# --- 公共接口库 (封装所有第三方依赖) ---
add_library(ProjectDeps INTERFACE)



message(STATUS "ZED_INCLUDE_DIR = ${ZED_INCLUDE_DIRS}")
target_include_directories(ProjectDeps INTERFACE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${TensorRT_INCLUDE_DIR}
    ${DEEPSTREAM_INCLUDE_DIR}
    ${GSTREAMER_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${ZED_INCLUDE_DIRS}

)


target_link_libraries(ProjectDeps INTERFACE
    
    # SocketCAN
    ${SOCKETCAN_LIB}

    # TensorRT
    nvinfer
    nvonnxparser
    
    # CUDA
    CUDA::cudart
    
    # OpenCV (使用 IMPORTED targets 更健壮)
    ${OpenCV_LIBS}  # 或替换为: opencv_core opencv_imgproc opencv_videoio ...
    
    # DeepStream
    ${NVDSGST_HELPER_LIB}
    ${NVDS_META_LIB}
    ${NVDSGST_META_LIB}
    ${NVBUFSURFACE_LIB}
    ${NVDS_INFER_LIB}
    ${NVDS_UTILS_LIB}
    
    # GStreamer 及其依赖
    PkgConfig::GSTREAMER
    glib-2.0
    gobject-2.0
    pthread

    ${ZED_LIBRARIES}
)

target_compile_definitions(ProjectDeps INTERFACE DEEPSTREAM_ENABLED)

get_target_property(inc_dirs ProjectDeps INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "ProjectDeps INTERFACE_INCLUDE_DIRECTORIES: ${inc_dirs}")









# --- 项目内部库 ---
add_library(My_SHARED_LIB SHARED 
    src/lib/locator_2d.cpp
    src/lib/filter.cpp
    src/lib/points_generator.cpp
    src/lib/predictor.cpp
    src/lib/CAN.cpp
)

# --- 公共接口库 ---
target_include_directories(My_SHARED_LIB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 共享库自动继承静态库的 PUBLIC 接口
target_link_libraries(My_SHARED_LIB PUBLIC ProjectDeps)











# --- 可执行文件 ---
add_executable(Main src/main.cpp)
target_link_libraries(Main PRIVATE ProjectDeps My_SHARED_LIB)

add_executable(BAK_Main src/main_bak.cpp)
target_link_libraries(BAK_Main PRIVATE ProjectDeps My_SHARED_LIB)

add_executable(Test src/test.cpp)
target_link_libraries(Test PRIVATE ProjectDeps My_SHARED_LIB)

add_executable(Module_Test src/module_test.cpp)
target_link_libraries(Module_Test PRIVATE ProjectDeps My_SHARED_LIB)

add_executable(ZED_legacy src/zed_legacy_api.cpp)  
target_link_libraries(ZED_legacy PRIVATE ProjectDeps My_SHARED_LIB)
target_include_directories(ZED_legacy PRIVATE ${ZED_INCLUDE_DIRS})   # 新增这一行


# # --- 构建输出目录优化 (可选) ---
# set_target_properties(Main Test BAK_Main ZED_legacy Module_Test My_SHARED_LIB PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
#     LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
# )