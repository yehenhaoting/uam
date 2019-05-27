cmake_minimum_required(VERSION 2.8.3)
project(uam_arm)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  kdl_parser
  trac_ik_lib
  message_generation
  dynamic_reconfigure
)
find_package(orocos_kdl)
find_package(Boost REQUIRED COMPONENTS date_time)

generate_dynamic_reconfigure_options(
        cfg/uam_cfg.cfg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(KeyboardTeleopNode src/KeyboardTeleopNode.cpp)
#add_executable(HololensServer src/HololensServer.cpp)
add_executable(TRACIKsolver src/TRACIKsolver.cpp)
add_executable(ftServo src/ftServo.cpp src/ftServo.h)
add_executable(test_servo src/test_servo.cpp src/ftServo.h)

target_link_libraries(KeyboardTeleopNode ${catkin_LIBRARIES})
target_link_libraries(ftServo ${catkin_LIBRARIES})
#target_link_libraries(HololensServer ${catkin_LIBRARIES})
target_link_libraries(TRACIKsolver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
target_link_libraries(test_servo ${catkin_LIBRARIES})

add_dependencies(KeyboardTeleopNode ${catkin_EXPORTED_TARGETS})
#add_dependencies(HololensServer ${catkin_EXPORTED_TARGETS})
add_dependencies(ftServo ${catkin_EXPORTED_TARGETS})
add_dependencies(test_servo ${catkin_EXPORTED_TARGETS})


