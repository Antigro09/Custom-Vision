cmake_minimum_required(VERSION 3.16)
project(apriltag_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_NETWORKTABLES "Enable NetworkTables publishing (ntcore/wpiutil)" ON)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui calib3d)
set(OPENCV_LIBS opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d)

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

# apriltag (robust)
set(HAVE_APRILTAG_TARGET 0)
find_package(apriltag CONFIG QUIET)

if(apriltag_FOUND)
  message(STATUS "apriltag config found")
  if(TARGET apriltag::apriltag)
    set(HAVE_APRILTAG_TARGET 1)
    message(STATUS "Using target apriltag::apriltag")
  elseif(TARGET apriltag)
    set(HAVE_APRILTAG_TARGET 2)
    message(STATUS "Using target apriltag")
  else()
    message(STATUS "apriltag config found but no imported target; trying pkg-config")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(APRILTAG REQUIRED apriltag)
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(APRILTAG REQUIRED apriltag)
endif()

# NetworkTables optional
set(HAVE_NTCORE 0)
if(ENABLE_NETWORKTABLES)
  find_package(ntcore CONFIG QUIET)
  find_package(wpiutil CONFIG QUIET)
  if(ntcore_FOUND AND wpiutil_FOUND)
    set(HAVE_NTCORE 1)
    message(STATUS "NTCore enabled")
  else()
    message(WARNING "ntcore/wpiutil not found; NT disabled")
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

target_include_directories(apriltag_vision PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(apriltag_vision PRIVATE HAVE_NTCORE=${HAVE_NTCORE})

target_link_libraries(apriltag_vision PRIVATE
  ${OPENCV_LIBS}
  nlohmann_json::nlohmann_json
)

if(HAVE_APRILTAG_TARGET EQUAL 1)
  target_link_libraries(apriltag_vision PRIVATE apriltag::apriltag)
elseif(HAVE_APRILTAG_TARGET EQUAL 2)
  target_link_libraries(apriltag_vision PRIVATE apriltag)
else()
  target_include_directories(apriltag_vision PRIVATE ${APRILTAG_INCLUDE_DIRS})
  target_link_directories(apriltag_vision PRIVATE ${APRILTAG_LIBRARY_DIRS})
  target_link_libraries(apriltag_vision PRIVATE ${APRILTAG_LIBRARIES})
  target_compile_options(apriltag_vision PRIVATE ${APRILTAG_CFLAGS_OTHER})
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
