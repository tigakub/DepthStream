
cmake_minimum_required(VERSION 3.10)

project(DepthStream)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    /Users/tigakub/Documents/Projects/Robotics/DepthAI/depthai-python/depthai-core/build/install/include
    /Users/tigakub/Documents/Projects/Robotics/DepthAI/depthai-python/depthai-core/build/install/include/depthai-shared/3rdparty
    /Users/tigakub/Documents/Projects/Robotics/DepthAI/depthai-python/depthai-core/build/install/lib/cmake/depthai/dependencies/include
    /Users/tigakub/Documents/Projects/Robotics/opencv/include
)

add_executable(DepthStream main.cpp)

target_link_libraries(DepthStream PRIVATE depthai::opencv ${OpenCV_LIBS})
