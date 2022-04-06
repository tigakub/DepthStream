
cmake_minimum_required(VERSION 3.10)

project(DepthStream)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ~/Documents/Robotics/depthai-python/depthai-core/include
    ~/Documents/Robotics/depthai-python/depthai-core/build/install/include/depthai-shared/3rdparty
    ~/Documents/Robotics/depthai-python/depthai-core/build/install/lib/cmake/depthai/dependencies/include
    /opt/opencv/include/opencv4
)

set(SOURCES main.cpp Server.hpp Server.cpp Buffer.hpp Buffer.cpp Connection.hpp Connection.cpp Exception.cpp Exception.hpp DepthProxy.hpp StructuredPointCloud.hpp)
add_executable(DepthStream ${SOURCES})

target_link_libraries(DepthStream PRIVATE depthai::opencv ${OpenCV_LIBS})
