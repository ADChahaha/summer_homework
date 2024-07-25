cmake_minimum_required(VERSION 3.5)
project(pnp_solve)

find_package(OpenCV REQUIRED)

add_executable(pnp_solve src/pnp_solve.cpp src/armor.cpp src/number_classifier.cpp)

target_include_directories(pnp_solve PUBLIC ${OPENCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(pnp_solve PUBLIC ${OpenCV_LIBS})