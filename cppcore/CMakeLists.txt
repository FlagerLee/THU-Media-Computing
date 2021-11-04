cmake_minimum_required(VERSION 3.16)
project(MediaProc)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(HEADER
        library.h
        transfer_color.h
        img_video_edit_v1.h
        img_video_edit_v2.h
        img_completion.h
        img_matting.h
        img_resizing.h
        img_compositing.h
)

set(SOURCE
        library.cpp
        transfer_color.cpp
        img_video_edit_v1.cpp
        img_video_edit_v2.cpp
        img_completion.cpp
        img_matting.cpp
        img_resizing.cpp
        img_compositing.cpp
)

if(BUILD_LIBRARY)
    add_library(${PROJECT_NAME} SHARED ${HEADER} ${SOURCE})
else()
    add_executable(${PROJECT_NAME} main.cpp ${SOURCE})
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})