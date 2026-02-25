cmake_minimum_required(VERSION 3.0.2)
project(event_based_camera)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_library(CAER_LIBRARY NAMES caer)

# --- Rust crate build ---
set(RUST_CRATE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rust/edvs_processing)
set(RUST_LIB ${RUST_CRATE_DIR}/target/release/libedvs_processing.a)

add_custom_command(
  OUTPUT ${RUST_LIB}
  COMMAND cargo build --release
  WORKING_DIRECTORY ${RUST_CRATE_DIR}
  COMMENT "Building Rust edvs_processing crate"
)
add_custom_target(edvs_processing_rust ALL DEPENDS ${RUST_LIB})

# --- ROS messages ---
add_message_files(
  FILES
    Control.msg
    Event.msg
    EventArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
)

# --- Build ---
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(edvs_camera
  src/edvs_driver_node.cpp
  src/edvs_driver.cpp
  src/security.cpp
)

add_dependencies(edvs_camera
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  edvs_processing_rust
)

target_link_libraries(edvs_camera PRIVATE
  ${catkin_LIBRARIES}
  ${CAER_LIBRARY}
  ${RUST_LIB}
  pthread
  dl
)

# --- Install ---
install(TARGETS edvs_camera
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
