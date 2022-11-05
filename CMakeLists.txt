CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(Kinect)
set(CMAKE_CXX_FLAGS "-g")

set(CMAKE_CXX_FLAGS "-lstdc++ -pthread" )
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3")

find_package(k4a REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
find_package(PCL 1.10 REQUIRED)
find_package(Threads REQUIRED)



IF(NOT WIN32)
    add_definitions(-Wno-format-extra-args)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
ENDIF()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

add_definitions(${PCL_DEFINITIONS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

ADD_EXECUTABLE(run main.cpp ${HDR_FILES} ${SRC_FILES} ${SOURCE_FILES} include/rotate_cloud.hpp src/rotate_cloud.cpp)
add_definitions(-std=c++14 -O2)

target_link_libraries(run ${OpenCV_LIBS})
target_link_libraries(run k4a)
target_link_libraries(run ${PCL_LIBRARIES})
target_link_libraries(run Threads::Threads)
target_link_libraries(run PRIVATE ${EXTERNAL_LIBRARIES})