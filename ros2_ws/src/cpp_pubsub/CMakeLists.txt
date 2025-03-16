cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)

# Add JSONCPP include directories explicitly
include_directories(/usr/include/jsoncpp)

add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs)
target_link_libraries(talker curl jsoncpp)

install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

ament_package()
