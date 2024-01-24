cmake_minimum_required(VERSION 3.8)
project(rigid_body_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(phasespace_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

### NOTE if you want to add python libs add them here
ament_python_install_package(python_lib)
### NOTE

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories (
    ${EIGEN3_INCLUDE_DIR}
  )

### NOTE add Cpp nodes
add_executable(rb_capture src/rb_capture.cpp)
target_include_directories(rb_capture PUBLIC
### NOTE you can add additional include dirs for Cpp here
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
### NOTE
  $<INSTALL_INTERFACE:include>)
target_compile_features(rb_capture PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17




### NOTE add dependencies to the cpp target
ament_target_dependencies(rb_capture
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "phasespace_msgs"
  "Eigen3"
)
### NOTE


install(TARGETS rb_capture
  DESTINATION lib/${PROJECT_NAME})


add_executable(drone_odom_node src/drone_odom_node.cpp)

target_include_directories(drone_odom_node PUBLIC
### NOTE you can add additional include dirs for Cpp here
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
### NOTE
  $<INSTALL_INTERFACE:include>)
target_compile_features(drone_odom_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

### NOTE add dependencies to the cpp target
ament_target_dependencies(drone_odom_node
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "phasespace_msgs"
  "Eigen3"
)
### NOTE

install(TARGETS drone_odom_node
  DESTINATION lib/${PROJECT_NAME})



### NOTE add python nodes
install(PROGRAMS
	scripts/node_py.py
	DESTINATION lib/${PROJECT_NAME})

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

ament_package()
