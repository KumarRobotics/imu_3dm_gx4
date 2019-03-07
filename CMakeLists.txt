cmake_minimum_required(VERSION 3.3)
project(imu_3dm_gx4)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
             COMPONENTS 

                        roscpp
                        tf2_ros
                        std_msgs
                        sensor_msgs
                        geometry_msgs
                        diagnostic_updater
                        message_generation)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs)

add_executable(${PROJECT_NAME} src/imu_3dm_gx4.cpp src/imu.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
