# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(GokboruTracker)

set(CMAKE_CXX_STANDARD 17)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui features2d calib3d video tracking)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ai_drone_tracker main.cpp)

target_link_libraries(ai_drone_tracker Threads::Threads ${OpenCV_LIBS})

# Auto-apply capabilities for real-time OS scheduling without sudo
add_custom_command(TARGET ai_drone_tracker POST_BUILD
    COMMAND sudo setcap 'cap_sys_nice=eip' $<TARGET_FILE:ai_drone_tracker>
    COMMENT "Elevating CAP_SYS_NICE for real-time thread scheduling..."
)

