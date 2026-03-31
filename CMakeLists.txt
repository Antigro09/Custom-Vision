find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui calib3d)
find_package(apriltag CONFIG REQUIRED)
find_package(nlohmann_json CONFIG QUIET)
find_package(cscore CONFIG REQUIRED)
find_package(cameraserver CONFIG REQUIRED)
find_package(ntcore CONFIG REQUIRED)
find_package(wpiutil CONFIG REQUIRED)

add_executable(apriltag_vision
  src/main.cpp src/calibration.cpp src/detector.cpp src/pose.cpp src/visualize.cpp src/publisher.cpp
)

target_link_libraries(apriltag_vision PRIVATE
  opencv_core opencv_imgproc opencv_videoio opencv_highgui opencv_calib3d
  apriltag
  cscore
  cameraserver
  ntcore
  wpiutil
  nlohmann_json::nlohmann_json
)
