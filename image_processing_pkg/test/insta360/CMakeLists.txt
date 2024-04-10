cmake_minimum_required(VERSION 3.5)
project(YourProjectName)

# 设置 C++ 编译器
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)


# 添加头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(image_processor
        main.cpp
)

# 链接 FFmpeg 库
target_link_libraries(image_processor ${OpenCV_LIBRARIES})
