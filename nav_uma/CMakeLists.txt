cmake_minimum_required(VERSION 3.0.2)
project(nav_uma)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LibsModule
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(LibsModule
   include/Angle.cpp
   include/BasePose.cpp
   include/Joints.cpp
   include/JointState.cpp
   include/MotionCommand.cpp
   include/Pose.cpp
   include/Quaternion.cpp
   include/Time.cpp
   include/Vector3.cpp
   include/Waypoint.cpp
   include/WaypointNavigation.cpp
 )

 add_executable(main src/main.cpp)
 target_link_libraries(main LibsModule)
 target_link_libraries(main ${catkin_LIBRARIES})
