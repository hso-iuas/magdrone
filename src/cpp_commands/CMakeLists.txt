cmake_minimum_required(VERSION 3.8)
project(cpp_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cpp_drone_control_interface REQUIRED)
find_package(cpp_service_client REQUIRED)
find_package(cpp_velocity_controller REQUIRED)

add_library(${PROJECT_NAME}_lib
        src/commands.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
        rclcpp std_msgs geometry_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs custom_services cpp_drone_control_interface cpp_service_client cpp_velocity_controller
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Export information so other packages can find and use this library
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs custom_services cpp_drone_control_interface cpp_service_client cpp_velocity_controller)

ament_package()