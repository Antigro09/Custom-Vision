cmake_minimum_required(VERSION 3.16)
project(apriltag_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_NETWORKTABLES "Enable NetworkTables publishing (ntcore/cscore/cameraserver/wpiutil)" ON)

# OpenCV (vcpkg usually provides opencv4 config)
find_package(opencv4 CONFIG QUIET)
if(opencv4_FOUND)
    message(STATUS "Found opencv4")
    set(OPENCV_LIBS opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d)
else()
    find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc videoio highgui calib3d)
    message(STATUS "Found OpenCV")
    set(OPENCV_LIBS opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d)
endif()

# nlohmann/json
find_package(nlohmann_json CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# apriltag (prefer CMake config; fallback pkg-config)
find_package(apriltag CONFIG QUIET)
if(NOT apriltag_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(APRILTAG REQUIRED apriltag)
endif()

# NetworkTables stack (optional)
set(HAVE_NTCORE 0)
if(ENABLE_NETWORKTABLES)
    find_package(ntcore CONFIG QUIET)
    find_package(cscore CONFIG QUIET)
    find_package(cameraserver CONFIG QUIET)
    find_package(wpiutil CONFIG QUIET)

    if(ntcore_FOUND AND cscore_FOUND AND cameraserver_FOUND AND wpiutil_FOUND)
        message(STATUS "NetworkTables stack found: enabling NT publish")
        set(HAVE_NTCORE 1)
    else()
        message(WARNING "NetworkTables dependencies not fully found; building without NT publish")
    endif()
endif()

add_executable(apriltag_vision
    src/main.cpp
    src/calibration.cpp
    src/detector.cpp
    src/pose.cpp
    src/visualize.cpp
    src/publisher.cpp
)

target_include_directories(apriltag_vision PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(NOT apriltag_FOUND)
    target_include_directories(apriltag_vision PRIVATE ${APRILTAG_INCLUDE_DIRS})
    target_link_directories(apriltag_vision PRIVATE ${APRILTAG_LIBRARY_DIRS})
endif()

target_compile_definitions(apriltag_vision PRIVATE HAVE_NTCORE=${HAVE_NTCORE})

target_link_libraries(apriltag_vision PRIVATE
    ${OPENCV_LIBS}
    nlohmann_json::nlohmann_json
)

if(apriltag_FOUND)
    target_link_libraries(apriltag_vision PRIVATE apriltag::apriltag)
else()
    target_link_libraries(apriltag_vision PRIVATE ${APRILTAG_LIBRARIES})
    target_compile_options(apriltag_vision PRIVATE ${APRILTAG_CFLAGS_OTHER})
endif()

if(HAVE_NTCORE)
    set(NTCORE_TARGET ntcore)
    if(TARGET ntcore::ntcore)
        set(NTCORE_TARGET ntcore::ntcore)
    endif()

    set(CSCORE_TARGET cscore)
    if(TARGET cscore::cscore)
        set(CSCORE_TARGET cscore::cscore)
    endif()

    set(CAMERASERVER_TARGET cameraserver)
    if(TARGET cameraserver::cameraserver)
        set(CAMERASERVER_TARGET cameraserver::cameraserver)
    endif()

    set(WPIUTIL_TARGET wpiutil)
    if(TARGET wpiutil::wpiutil)
        set(WPIUTIL_TARGET wpiutil::wpiutil)
    endif()

    target_link_libraries(apriltag_vision PRIVATE
        ${NTCORE_TARGET}
        ${CSCORE_TARGET}
        ${CAMERASERVER_TARGET}
        ${WPIUTIL_TARGET}
    )
endif()