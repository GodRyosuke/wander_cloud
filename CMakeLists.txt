cmake_minimum_required(VERSION 3.0.2)
project(wander_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  eigen_conversions
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  tf
  tf_conversions
  genmsg
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  roscpp
  rospy
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# Use ${EIGEN3_...} variables in either case below

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveRealsenseData.srv
)

add_service_files(
  FILES
  DetectCylinder.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   pcl_msgs#   sensor_msgs#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME} roscpp
 CATKIN_DEPENDS eigen_conversions geometry_msgs pcl_conversions pcl_msgs pcl_ros sensor_msgs std_msgs roscpp rospy cv_bridge image_transport
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wander_cloud.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(cloud_creator src/cloud_creator.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cloud_creator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_test_create src/cloud_test_create.cpp)
target_link_libraries(cloud_test_create
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_from_realsense src/cloud_from_realsense.cpp)
add_dependencies(cloud_from_realsense ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_from_realsense
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(save_realsense_data src/save_realsense_data.cpp)
add_dependencies(save_realsense_data ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_realsense_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(wander_detect_cylinder src/wander_detect_cylinder.cpp)
add_dependencies(wander_detect_cylinder ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(wander_detect_cylinder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(detect_cylinder_from_realsense src/detect_cylinder_from_realsense.cpp)
add_dependencies(detect_cylinder_from_realsense ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_cylinder_from_realsense
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(detect_intersect_arc src/detect_intersect_arc.cpp)
add_dependencies(detect_intersect_arc ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_intersect_arc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  
)

add_library(testImp src/testImp.cpp)
target_link_libraries(testImp ${catkin_LIBRARIES})
add_dependencies(testImp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(testClient src/testClient.cpp)
add_dependencies(testClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testClient testImp ${catkin_LIBRARIES})

add_executable (wander_mani src/wander_mani.cpp)
add_dependencies(wander_mani ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wander_mani ${catkin_LIBRARIES})

add_library(detect_objects_imp src/detect_objects_imp.cpp)
target_link_libraries(detect_objects_imp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(detect_objects_imp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(detect_objects_node src/detect_objects_node.cpp)
add_dependencies(detect_objects_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_objects_node detect_objects_imp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(test_opencv src/test_opencv.cpp)
add_dependencies(test_opencv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(opencv_test_pub src/opencv_test_pub.cpp)
add_dependencies(opencv_test_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(opencv_test_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(wander_opencv_pub src/wander_opencv_pub.cpp)
add_dependencies(wander_opencv_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wander_opencv_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (wander_create_2value_img src/wander_create_2value_img.cpp)
add_dependencies(wander_create_2value_img ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wander_create_2value_img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(wander_detect_face src/wander_detect_face.cpp)
add_dependencies(wander_detect_face ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wander_detect_face ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(check_vertical src/check_vertical.cpp)
add_dependencies(check_vertical ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(check_vertical ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(get_pcd_jpg src/get_pcd_jpg.cpp)
add_dependencies(get_pcd_jpg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_pcd_jpg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(realtime_detection src/realtime_detection.cpp)
add_dependencies(realtime_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realtime_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} detect_objects_imp)

add_executable(detect_cylinder_2plane src/detect_cylinder_2plane.cpp)
add_dependencies(detect_cylinder_2plane ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_cylinder_2plane ${catkin_LIBRARIES} ${PCL_LIBRARIES} detect_objects_imp)










