cmake_minimum_required(VERSION 3.8)
project(positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)


aux_source_directory(src SRC_LIST)
include_directories (include) 

add_executable(icp_node ${SRC_LIST})
ament_target_dependencies(icp_node  rclcpp Eigen3 PCL sensor_msgs nav_msgs tf2_ros builtin_interfaces pcl_conversions)
target_include_directories(icp_node
    PRIVATE
    include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}#后面跟PCL之类的PCL_INLUDE_DIR,就是你的第三方依赖库
)

# add_executable(icp_node ${SRC_LIST})
# ament_target_dependencies(icp_node rclcpp Eigen3 tf2_ros PCL sensor_msgs pcl_conversions nav_msgs builtin_interfaces)
# target_include_directories(icp_node PRIVATE include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS icp_node
DESTINATION lib/${PROJECT_NAME})

ament_package()
