cmake_minimum_required(VERSION 3.16)
project(apriltag_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_NETWORKTABLES "Enable NetworkTables publishing (ntcore/wpiutil)" ON)

# ---------------- OpenCV ----------------
# Jetson apt installs generally expose classic OpenCV package first.
find_package(OpenCV QUIET COMPONENTS core imgproc videoio highgui calib3d)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV (system): ${OpenCV_VERSION}")
    set(OPENCV_LIBS opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d)
else()
    # Fallback for vcpkg-style config package naming
    find_package(opencv4 CONFIG REQUIRED)
    message(STATUS "Found opencv4 (config)")
    set(OPENCV_LIBS opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d)
endif()

# ---------------- CUDA (optional) ----------------
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    message(STATUS "CUDA toolkit found: ${CUDAToolkit_VERSION}")
    add_compile_definitions(HAVE_CUDA=1)
else()
    message(STATUS "CUDA toolkit not found; CPU path will be used")
    add_compile_definitions(HAVE_CUDA=0)
endif()

# ---------------- nlohmann/json ----------------
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

# ---------------- apriltag ----------------
find_package(apriltag CONFIG QUIET)
if(NOT apriltag_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(APRILTAG REQUIRED apriltag)
endif()

# ---------------- NetworkTables (optional) ----------------
set(HAVE_NTCORE 0)
if(ENABLE_NETWORKTABLES)
    find_package(ntcore CONFIG QUIET)
    find_package(wpiutil CONFIG QUIET)

    if(ntcore_FOUND AND wpiutil_FOUND)
        message(STATUS "Found ntcore + wpiutil: NT publishing enabled")
        set(HAVE_NTCORE 1)
    else()
        message(WARNING "ntcore/wpiutil not found; NT publishing disabled")
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

    set(WPIUTIL_TARGET wpiutil)
    if(TARGET wpiutil::wpiutil)
        set(WPIUTIL_TARGET wpiutil::wpiutil)
    endif()

    target_link_libraries(apriltag_vision PRIVATE
        ${NTCORE_TARGET}
        ${WPIUTIL_TARGET}
    )
endif()
