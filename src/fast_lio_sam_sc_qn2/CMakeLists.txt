cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(fast_lio_sam_sc_qn2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(FAST_LIO_SAM_SC_QN2_QUATRO_TBB "Enable TBB in the vendored Quatro matcher" ON)
option(FAST_LIO_SAM_SC_QN2_QUATRO_DEBUG "Enable Quatro debug output" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree features registration)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(teaserpp REQUIRED)

set(GTSAM_LINK_LIB gtsam)
if(TARGET GTSAM::gtsam)
  set(GTSAM_LINK_LIB GTSAM::gtsam)
endif()

if(FAST_LIO_SAM_SC_QN2_QUATRO_TBB)
  find_package(TBB QUIET)
  if(NOT TBB_FOUND)
    find_library(TBB_LIBRARY tbb HINTS /usr/lib /usr/local/lib)
  endif()
endif()

include(ProcessorCount)
ProcessorCount(PROC_COUNT)
if(PROC_COUNT GREATER 0)
  set(QUATRO_FLANN_CORES ${PROC_COUNT})
else()
  set(QUATRO_FLANN_CORES 1)
endif()

add_definitions(${PCL_DEFINITIONS})

set(ROS2_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  rosbag2_cpp
  rosbag2_storage
  livox_ros_driver2
)

# Keep the vendored algorithm build order aligned with the upstream ROS 1
# project: nano_gicp first, then quatro, then the main back-end code.
add_library(nano_gicp_vendor
  third_party/nano_gicp/src/nanoflann.cc
  third_party/nano_gicp/src/nano_gicp.cc
)
target_include_directories(nano_gicp_vendor PUBLIC
  third_party/nano_gicp/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(nano_gicp_vendor PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_library(quatro_vendor
  third_party/quatro/src/fpfh.cc
  third_party/quatro/src/matcher.cc
  third_party/quatro/src/quatro_module.cc
)
target_include_directories(quatro_vendor PUBLIC
  third_party/quatro/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(quatro_vendor PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  teaserpp::teaser_registration
  teaserpp::teaser_io
  OpenMP::OpenMP_CXX
)
target_compile_definitions(quatro_vendor PUBLIC
  QUATRO_FLANN_CORES=${QUATRO_FLANN_CORES}
)
add_dependencies(quatro_vendor nano_gicp_vendor)

if(FAST_LIO_SAM_SC_QN2_QUATRO_TBB AND (TBB_FOUND OR TBB_LIBRARY))
  if(PROC_COUNT GREATER 4)
    set(QUATRO_TBB_PROC_NUM ${PROC_COUNT})
  else()
    set(QUATRO_TBB_PROC_NUM 1)
  endif()
  target_compile_definitions(quatro_vendor PUBLIC
    TBB_EN
    TBB_PROC_NUM=${QUATRO_TBB_PROC_NUM}
  )
  if(TARGET TBB::tbb)
    target_link_libraries(quatro_vendor PUBLIC TBB::tbb)
  else()
    target_link_libraries(quatro_vendor PUBLIC ${TBB_LIBRARY})
  endif()
endif()

if(FAST_LIO_SAM_SC_QN2_QUATRO_DEBUG)
  target_compile_definitions(quatro_vendor PUBLIC QUATRO_DEBUG)
endif()

add_library(scancontext_vendor
  third_party/scancontext_tro/src/Scancontext.cpp
)
target_include_directories(scancontext_vendor PUBLIC
  third_party/scancontext_tro/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scancontext_vendor PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
add_dependencies(scancontext_vendor quatro_vendor)

add_library(${PROJECT_NAME}_core
  src/loop_closure.cpp
  src/fast_lio_sam_sc_qn2.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  include
  third_party/nano_gicp/include
  third_party/quatro/include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}_core
  nano_gicp_vendor
  quatro_vendor
  scancontext_vendor
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${GTSAM_LINK_LIB}
  OpenMP::OpenMP_CXX
)
add_dependencies(${PROJECT_NAME}_core scancontext_vendor)
ament_target_dependencies(${PROJECT_NAME}_core ${ROS2_DEPS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node ${ROS2_DEPS})

add_executable(livox_raw_preview_node src/livox_raw_preview_node.cpp)
ament_target_dependencies(livox_raw_preview_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

install(TARGETS
  ${PROJECT_NAME}_node
  livox_raw_preview_node
  ${PROJECT_NAME}_core
  nano_gicp_vendor
  quatro_vendor
  scancontext_vendor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/fast_lio_web_broker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
