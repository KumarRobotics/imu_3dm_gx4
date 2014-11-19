cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx4)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  message_generation
  roscpp
  geometry_msgs
  sensor_msgs
)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

# include boost
find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

add_definitions("-std=c++0x -Wall -Werror")

add_executable(${PROJECT_NAME} src/imu_3dm_gx4.cpp src/imu.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
