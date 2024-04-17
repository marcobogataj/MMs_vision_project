cmake_minimum_required(VERSION 3.0.2)
project(zivid_capture)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  zivid_camera
  dynamic_reconfigure
)

add_service_files(FILES ZividCaptureSuggested.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   CATKIN_DEPENDS zivid_camera
)

#################
## C++ Samples ##
#################

function(register_cpp_sample)
  cmake_parse_arguments(
    ARG
    ""
    "NAME;SRC"
    ""
    ${ARGN}
  )
  add_executable(${ARG_NAME} ${ARG_SRC})
  add_dependencies(${ARG_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(${ARG_NAME}
    SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${ARG_NAME}
    ${catkin_LIBRARIES}
  )
  install(TARGETS ${ARG_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

register_cpp_sample(NAME hello_cpp_e SRC src/hello_cpp_f.cpp)
register_cpp_sample(NAME capture_assistant SRC src/capture_assistant.cpp)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

