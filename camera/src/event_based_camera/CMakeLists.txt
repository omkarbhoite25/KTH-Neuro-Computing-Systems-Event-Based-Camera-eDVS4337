cmake_minimum_required(VERSION 3.0.2)
project(event_based_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_library(CAER_LIBRARY NAMES caer)

# add_message_files(
#   DIRECTORY
#   FILES
#     Control.msg
# )
add_message_files(
  FILES
    Control.msg
)


generate_messages(
  DEPENDENCIES
    std_msgs
    
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
)

###########
## Build ##
###########
# add_library(
#   cm
#   src/new.cpp
#   src/new.hpp
# )
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # add_executable(event_based_camera src/event_based_camera.cpp)
add_executable(event_based_camera src/event_based_camera.cpp)
# target_link_libraries(event_based_camera ${catkin_LIBRARIES})
target_link_libraries(event_based_camera PRIVATE ${catkin_LIBRARIES} ${CAER_LIBRARY})

install(TARGETS event_based_camera
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

