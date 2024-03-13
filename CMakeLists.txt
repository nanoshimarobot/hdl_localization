cmake_minimum_required(VERSION 3.8)
project(hdl_localization)

# find_package(catkin REQUIRED COMPONENTS
#   nodelet
#   tf2
#   tf2_ros
#   tf2_eigen
#   tf2_geometry_msgs
#   eigen_conversions
#   pcl_ros
#   roscpp
#   rospy
#   sensor_msgs
#   geometry_msgs
#   message_generation
#   ndt_omp
#   fast_gicp
#   hdl_global_localization
# )

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp
  src/${PROJECT_NAME}/pose_estimator.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "hdl_localization::HdlLocalization")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)
ament_auto_add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_main.cpp)
target_include_directories(${PROJECT_NAME}_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_component PRIVATE ${PCL_DEFINITIONS})

ament_auto_add_library(globalmap_server_component SHARED
  src/globalmap_server_component.cpp
)
target_compile_options(globalmap_server_component PUBLIC -Wall)
ament_auto_add_executable(globalmap_server src/globalmap_server_main.cpp)
target_include_directories(globalmap_server_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(globalmap_server_component ${PCL_LIBRARIES})
target_compile_definitions(globalmap_server_component PRIVATE ${PCL_DEFINITIONS})

ament_auto_package()