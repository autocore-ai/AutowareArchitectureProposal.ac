cmake_minimum_required(VERSION 3.5)
project(spinnaker_opencv_rosnode)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

include_directories(
    /opt/spinnaker/include
    ${OpenCV_INCLUDE_DIRS}
    )


link_directories(
    /opt/spinnaker/lib
)

ament_auto_add_executable(spinnaker_opencv_rosnode
    src/multi_cam_node.cpp
)

ament_auto_add_library(flir_multicam_driver_lib SHARED
    src/lib/spinnaker_driver_multiple.cpp
)

target_link_libraries(flir_multicam_driver_lib
    Spinnaker ${OpenCV_LIBRARIES}
)

target_link_libraries(spinnaker_opencv_rosnode
    flir_multicam_driver_lib
)

ament_auto_package(INSTALL_TO_SHARE
launch
)

install(
TARGETS flir_multicam_driver_lib
LIBRARY DESTINATION lib
INCLUDES DESTINATION include
)

