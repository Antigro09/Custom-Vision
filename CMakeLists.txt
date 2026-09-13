cmake_minimum_required(VERSION 3.18)
project(custom_vision_native VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
option(CUSTOM_VISION_NATIVE_CPU "Optimize for this computer's CPU; rebuild when moving hardware" ON)
option(CUSTOM_VISION_CUDA_PREPROCESS "Enable optional OpenCV CUDA grayscale preprocessing" ON)
option(CUSTOM_VISION_CUDA_APRILTAGS "Use optional NVIDIA cuAprilTags binary (JetPack 6, NVIDIA license)" OFF)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
# A private pinned static build prevents accidental linkage against old robot
# software installed under /usr/local, and optimizes the detector itself.
include(FetchContent)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_PYTHON_WRAPPER OFF)
set(BUILD_TESTING OFF)
FetchContent_Declare(vision_apriltag
  URL https://codeload.github.com/AprilRobotics/apriltag/tar.gz/94be783968e5091bcc9972c72c84fd63efce2935
  URL_HASH SHA256=9cd32cbdafab30e34609eb141376d4288cfea6c3c0b3a24eb01e0d8ae5291226)
FetchContent_MakeAvailable(vision_apriltag)
if(CUSTOM_VISION_CUDA_APRILTAGS)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(FATAL_ERROR "The pinned cuAprilTags build supports JetPack 6 aarch64 only")
  endif()
  find_package(CUDAToolkit REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d cudaimgproc cudawarping)
elseif(CUSTOM_VISION_CUDA_PREPROCESS)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d OPTIONAL_COMPONENTS cudaimgproc)
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
endif()

pybind11_add_module(_native MODULE native/apriltags.cpp)
target_include_directories(_native SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(_native PRIVATE apriltag ${OpenCV_LIBS})
if(CUSTOM_VISION_CUDA_APRILTAGS)
  set(CUAPRILTAGS_DIR "${CMAKE_BINARY_DIR}/_deps/cuapriltags")
  file(MAKE_DIRECTORY "${CUAPRILTAGS_DIR}")
  set(CUAPRILTAGS_REV "e3a29804b6de38fefbf7040864124d983bfceb6a")
  file(DOWNLOAD
    "https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/${CUAPRILTAGS_REV}/isaac_ros_nitros/lib/cuapriltags/cuapriltags/cuAprilTags.h"
    "${CUAPRILTAGS_DIR}/cuAprilTags.h"
    EXPECTED_HASH SHA256=ce9a9bc04df19821448c50313e10d060a4926ff2c246fb82d4a6c277cf92c699)
  file(DOWNLOAD
    "https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/isaac_ros_nitros/${CUAPRILTAGS_REV}/isaac_ros_nitros/lib/cuapriltags/lib_aarch64_jetpack61/libcuapriltags.a"
    "${CUAPRILTAGS_DIR}/libcuapriltags.a"
    EXPECTED_HASH SHA256=e67298e8ee52d6253cd702f4c55b3727bea177f57236aa23797bde50d74dfc24)
  message(STATUS "cuAprilTags: NVIDIA ISAAC ROS SOFTWARE LICENSE; see native/NVIDIA_ISAAC_ROS_LICENSE.txt")
  target_include_directories(_native SYSTEM PRIVATE "${CUAPRILTAGS_DIR}")
  target_link_libraries(_native PRIVATE "${CUAPRILTAGS_DIR}/libcuapriltags.a" CUDA::cudart)
  target_compile_definitions(_native PRIVATE CUSTOM_VISION_HAVE_CUAPRILTAGS=1)
  # Compile upstream's exact decoder with a tiny same-translation-unit adapter.
  # This retains its real decision margin and decoded corner ordering for GPU
  # quads without repeating the expensive full-frame CPU search.
  get_target_property(APRILTAG_SOURCES apriltag SOURCES)
  list(REMOVE_ITEM APRILTAG_SOURCES apriltag.c)
  set_property(TARGET apriltag PROPERTY SOURCES "${APRILTAG_SOURCES}")
  target_sources(apriltag PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/native/apriltag_gpu_verify.c")
endif()
target_compile_options(_native PRIVATE -Wall -Wextra -Wpedantic)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(_native PRIVATE $<$<CONFIG:Release>:-O3>)
  target_compile_options(apriltag PRIVATE $<$<CONFIG:Release>:-O3> -fvisibility=hidden)
  target_link_options(_native PRIVATE -Wl,--exclude-libs,ALL)
  if(CUSTOM_VISION_NATIVE_CPU)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      target_compile_options(_native PRIVATE -mcpu=native)
      target_compile_options(apriltag PRIVATE -mcpu=native)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
      target_compile_options(_native PRIVATE -march=native)
      target_compile_options(apriltag PRIVATE -march=native)
    endif()
  endif()
endif()
# Do not use fast-math: finite-value and calibrated pose checks must remain valid.
if((CUSTOM_VISION_CUDA_PREPROCESS OR CUSTOM_VISION_CUDA_APRILTAGS) AND OpenCV_cudaimgproc_FOUND)
  target_compile_definitions(_native PRIVATE CUSTOM_VISION_HAVE_CUDA=1)
endif()
target_compile_definitions(_native PRIVATE CUSTOM_VISION_APRILTAG_VERSION="3.4.5")
set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/custom_vision")
message(STATUS "AprilTag: private static 3.4.5; OpenCV: ${OpenCV_VERSION}; CUDA preprocessing: ${OpenCV_cudaimgproc_FOUND}")
