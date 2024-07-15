cmake_minimum_required(VERSION 3.8)
project(helix_transmission_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCurrent.srv"
  "srv/GoToGripperPoseVector.srv"
  "srv/GoToGripperPoseQuat.srv"
  "srv/GoToGripperPoseEuler.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
