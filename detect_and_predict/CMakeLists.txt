cmake_minimum_required(VERSION 3.5)

project(detect_and_predict)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(detect_and_predict src/main.cpp src/get_armor.cpp 
                src/kalman.cpp src/number_classifier.cpp
                src/pnp.cpp src/pose.cpp src/client.cpp 
                src/handle_algorithm.cpp)

target_include_directories(detect_and_predict PUBLIC 
                        ${CMAKE_SOURCE_DIR}/include 
                        ${EIGEN3_INCLUDE_DIR}
                        ${OPENCV_INCLUDE_DIRS})

target_link_libraries(detect_and_predict PUBLIC ${OpenCV_LIBS})