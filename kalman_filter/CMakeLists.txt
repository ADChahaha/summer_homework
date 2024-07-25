cmake_minimum_required(VERSION 3.5)

project(kalman_filter)

find_package(Eigen3 REQUIRED)

add_executable(kalman_filter src/main.cpp src/kalman.cpp)

target_include_directories(kalman_filter PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}  # 添加Eigen头文件目录
)