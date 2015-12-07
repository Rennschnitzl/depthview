project(realsense-noQT)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/)
find_package( OpenCV 2.4.9 REQUIRED )
find_package(PCL REQUIRED )
find_package(octomap REQUIRED)
include_directories()
#find_package(KernelHeaders REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#link_libraries()

add_executable(${PROJECT_NAME} ${SRC_LIST})
#include_directories(${KERNELHEADERS_INCLUDE_DIRS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
