cmake_minimum_required(VERSION 4.2)
project(snapshotter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    srv/TakeSnapshot.srv
    DEPENDENCIES
    std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_lib OBJECT
  src/Snapshotter.cpp
  src/Snapshotter.hpp
  src/MessageRingBuffer.hpp
  src/MessageRingBuffer.cpp
  src/TopicFilter.cpp
  src/TopicFilter.hpp
  src/SingleMessageBuffer.hpp
  src/SingleMessageBuffer.cpp
  src/Node.hpp
  src/Node.cpp
)
target_link_libraries(
    ${PROJECT_NAME}_lib
    PUBLIC ${cpp_typesupport_target} rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp rosbag2_compression::rosbag2_compression rosbag2_transport::rosbag2_transport
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${cpp_typesupport_target}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(utest test/test.test test/test_node.cpp)
    target_include_directories(utest PRIVATE src)
    target_link_libraries(utest ${PROJECT_NAME}_lib ${std_srvs_TARGETS})

    ament_add_gtest(utest_reduction_rules test/test_reduction_rules.cpp)
    target_include_directories(utest_reduction_rules PRIVATE src)
    target_link_libraries(utest_reduction_rules ${PROJECT_NAME}_lib)
    target_compile_definitions(utest_reduction_rules PRIVATE
        TEST_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/test_reduction_rules_params.yaml"
    )
endif()

ament_package()
