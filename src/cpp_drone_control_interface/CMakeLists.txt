cmake_minimum_required(VERSION 3.8)
project(cpp_drone_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cpp_velocity_controller REQUIRED)

add_library(${PROJECT_NAME}_lib INTERFACE)

target_include_directories(${PROJECT_NAME}_lib INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}_lib INTERFACE
        ${cpp_velocity_controller_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_lib INTERFACE
        ${cpp_velocity_controller_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cpp_velocity_controller)

ament_package()