cmake_minimum_required(VERSION 3.8)
project(cpp_service_client)

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
find_package(custom_services REQUIRED)
find_package(cpp_drone_control_interface REQUIRED)

add_library(${PROJECT_NAME}_lib
        src/magnet_service_client.cpp
        src/force_arming_service_client.cpp
        src/set_mode_service_client.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
        rclcpp std_msgs geometry_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs custom_services cpp_drone_control_interface
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
ament_export_dependencies(rclcpp std_msgs geometry_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs custom_services cpp_drone_control_interface)

ament_package()