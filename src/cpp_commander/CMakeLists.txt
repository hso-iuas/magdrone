cmake_minimum_required(VERSION 3.8)
project(cpp_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(custom_services REQUIRED)
find_package(cpp_commands REQUIRED)
find_package(cpp_drone_control REQUIRED)
find_package(cpp_service_client REQUIRED)

set(SOURCE_FILES
    src/mavros_commander_node.cpp
    src/mavros_commander.cpp
)

add_executable(mavros_commander ${SOURCE_FILES})
ament_target_dependencies(mavros_commander rclcpp std_msgs geometry_msgs mavros_msgs tf2 tf2_ros custom_services cpp_commands cpp_drone_control cpp_service_client)

target_include_directories(mavros_commander PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/cpp_commander>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS
        mavros_commander
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
