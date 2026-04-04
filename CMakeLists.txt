cmake_minimum_required(VERSION 3.16)
project(apriltag_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui calib3d)
find_package(nlohmann_json CONFIG QUIET)

# Prefer a single apriltag install root to avoid mixing headers from
# /usr/local/include and /usr/include in the same translation unit.
set(APRILTAG_INCLUDE_DIR "")
set(APRILTAG_LIBRARY "")
foreach(APRILTAG_PREFIX /usr /usr/local)
    if(EXISTS "${APRILTAG_PREFIX}/include/apriltag/apriltag.h")
        find_library(APRILTAG_LIBRARY_CANDIDATE
            NAMES apriltag
            PATHS
                "${APRILTAG_PREFIX}/lib"
                "${APRILTAG_PREFIX}/lib/aarch64-linux-gnu"
                "${APRILTAG_PREFIX}/lib/x86_64-linux-gnu"
            NO_DEFAULT_PATH
        )
        if(APRILTAG_LIBRARY_CANDIDATE)
            set(APRILTAG_INCLUDE_DIR "${APRILTAG_PREFIX}/include")
            set(APRILTAG_LIBRARY "${APRILTAG_LIBRARY_CANDIDATE}")
            break()
        endif()
    endif()
endforeach()

if(NOT APRILTAG_LIBRARY)
    find_package(apriltag REQUIRED)
endif()

option(ENABLE_NETWORKTABLES "Enable NetworkTables publishing" ON)

set(HAVE_NTCORE 0)
if(ENABLE_NETWORKTABLES)
    find_package(cscore CONFIG QUIET)
    find_package(cameraserver CONFIG QUIET)
    find_package(ntcore CONFIG QUIET)
    find_package(wpiutil CONFIG QUIET)

    if(cscore_FOUND AND cameraserver_FOUND AND ntcore_FOUND AND wpiutil_FOUND)
        set(HAVE_NTCORE 1)
    else()
        message(WARNING "NetworkTables dependencies were not found; building without NT/CameraServer support")
    endif()
endif()

set(OPENCV_CUDA_LIBS)
foreach(cuda_lib opencv_cudaimgproc opencv_cudaarithm)
    if(TARGET ${cuda_lib})
        list(APPEND OPENCV_CUDA_LIBS ${cuda_lib})
    endif()
endforeach()

add_executable(apriltag_vision
    src/main.cpp
    src/calibration.cpp
    src/detector.cpp
    src/field_layout.cpp
    src/persistent_config.cpp
    src/pose.cpp
    src/visualize.cpp
    src/publisher.cpp
)

if(APRILTAG_INCLUDE_DIR)
    target_include_directories(apriltag_vision BEFORE PRIVATE "${APRILTAG_INCLUDE_DIR}")
endif()

target_compile_definitions(apriltag_vision PRIVATE HAVE_NTCORE=${HAVE_NTCORE})
if(WIN32)
    target_compile_definitions(apriltag_vision PRIVATE NOMINMAX)
endif()
if(OPENCV_CUDA_LIBS)
    target_compile_definitions(apriltag_vision PRIVATE HAVE_OPENCV_CUDA=1)
endif()

target_link_libraries(apriltag_vision PRIVATE
    opencv_core
    opencv_imgproc
    opencv_videoio
    opencv_highgui
    opencv_calib3d
    ${OPENCV_CUDA_LIBS}
    $<$<BOOL:${APRILTAG_LIBRARY}>:${APRILTAG_LIBRARY}>
    $<$<NOT:$<BOOL:${APRILTAG_LIBRARY}>>:apriltag>
    nlohmann_json::nlohmann_json
)

if(HAVE_NTCORE)
    target_link_libraries(apriltag_vision PRIVATE
        cscore
        cameraserver
        ntcore
        wpiutil
    )
endif()
