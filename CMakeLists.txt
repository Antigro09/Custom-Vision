cmake_minimum_required(VERSION 3.16)
project(apriltag_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_NETWORKTABLES "Enable NetworkTables publishing (ntcore/wpiutil)" ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui calib3d)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)

# apriltag: try CONFIG first, then pkg-config
set(HAVE_APRILTAG_TARGET 0)
find_package(apriltag CONFIG QUIET)
if(apriltag_FOUND)
  if(TARGET apriltag::apriltag)
    set(HAVE_APRILTAG_TARGET 1)
  elseif(TARGET apriltag)
    set(HAVE_APRILTAG_TARGET 2)
  else()
    pkg_check_modules(APRILTAG REQUIRED apriltag)
  endif()
else()
  pkg_check_modules(APRILTAG REQUIRED apriltag)
endif()

# CUDA is optional (runtime fallback still done in code)
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  add_compile_definitions(HAVE_CUDA_TOOLKIT=1)
else()
  add_compile_definitions(HAVE_CUDA_TOOLKIT=0)
endif()

set(HAVE_NTCORE 0)
if(ENABLE_NETWORKTABLES)
  find_package(ntcore CONFIG QUIET)
  find_package(wpiutil CONFIG QUIET)
  if(ntcore_FOUND AND wpiutil_FOUND)
    set(HAVE_NTCORE 1)
  endif()
endif()

add_executable(apriltag_vision
  src/main.cpp
  src/types.hpp
  src/config.hpp
  src/config.cpp
  src/calibration.hpp
  src/calibration.cpp
  src/detector.hpp
  src/detector.cpp
  src/pose.hpp
  src/pose.cpp
  src/publisher.hpp
  src/publisher.cpp
  src/visualize.hpp
  src/visualize.cpp
)

target_include_directories(apriltag_vision PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(NOT apriltag_FOUND OR (HAVE_APRILTAG_TARGET EQUAL 0))
  target_include_directories(apriltag_vision PRIVATE ${APRILTAG_INCLUDE_DIRS})
  target_link_directories(apriltag_vision PRIVATE ${APRILTAG_LIBRARY_DIRS})
  target_compile_options(apriltag_vision PRIVATE ${APRILTAG_CFLAGS_OTHER})
endif()

target_compile_definitions(apriltag_vision PRIVATE HAVE_NTCORE=${HAVE_NTCORE})

target_link_libraries(apriltag_vision PRIVATE
  opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d
  nlohmann_json::nlohmann_json
)

if(HAVE_APRILTAG_TARGET EQUAL 1)
  target_link_libraries(apriltag_vision PRIVATE apriltag::apriltag)
elseif(HAVE_APRILTAG_TARGET EQUAL 2)
  target_link_libraries(apriltag_vision PRIVATE apriltag)
else()
  target_link_libraries(apriltag_vision PRIVATE ${APRILTAG_LIBRARIES})
endif()

if(HAVE_NTCORE)
  if(TARGET ntcore::ntcore)
    target_link_libraries(apriltag_vision PRIVATE ntcore::ntcore)
  else()
    target_link_libraries(apriltag_vision PRIVATE ntcore)
  endif()

  if(TARGET wpiutil::wpiutil)
    target_link_libraries(apriltag_vision PRIVATE wpiutil::wpiutil)
  else()
    target_link_libraries(apriltag_vision PRIVATE wpiutil)
  endif()
endif()
