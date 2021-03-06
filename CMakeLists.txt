cmake_minimum_required(VERSION 3.10)
project(snapshotter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_compile_options(-Wall -Wextra -Werror)

set(catkinDependencies
    rosbag
    rosbag_storage
    roscpp
    rosgraph_msgs
    std_msgs)

find_package(catkin REQUIRED COMPONENTS ${catkinDependencies} message_generation)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(DIRECTORY srv FILES TakeSnapshot.srv)

generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)

catkin_package(CATKIN_DEPENDS ${catkinDependencies} message_runtime)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    src
)

add_library(${PROJECT_NAME}
  src/Snapshotter.cpp
  src/Snapshotter.hpp
  src/ShapeShifterMsg.hpp
  src/ShapeShifterMsg.cpp
  src/MessageRingBuffer.hpp
  src/MessageRingBuffer.cpp
  src/TopicFilter.cpp
  src/TopicFilter.hpp
  src/SingleMessageBuffer.hpp
  src/SingleMessageBuffer.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############
find_package(rostest REQUIRED)

add_rostest_gtest(utest test/test.test test/test_node.cpp)
target_link_libraries(utest
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}
    stdc++fs)
