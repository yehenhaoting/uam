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
find_package(joy)

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
<<<<<<< HEAD
add_executable(test_servo src/test_servo.cpp src/ftServo.h)
=======
add_executable(Joystick_node src/Joystick_node.cpp)
>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0

target_link_libraries(KeyboardTeleopNode ${catkin_LIBRARIES})
target_link_libraries(ftServo ${catkin_LIBRARIES})
#target_link_libraries(HololensServer ${catkin_LIBRARIES})
target_link_libraries(TRACIKsolver 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
<<<<<<< HEAD
target_link_libraries(test_servo ${catkin_LIBRARIES})
=======
target_link_libraries(Joystick_node ${catkin_LIBRARIES})

>>>>>>> 31cb061caf1041fd79889e3255a8fe5c84f435c0

add_dependencies(KeyboardTeleopNode ${catkin_EXPORTED_TARGETS})
#add_dependencies(HololensServer ${catkin_EXPORTED_TARGETS})
add_dependencies(ftServo ${catkin_EXPORTED_TARGETS})
add_dependencies(test_servo ${catkin_EXPORTED_TARGETS})


<<<<<<< HEAD
=======
add_executable(ik_test src/ik_test.cpp)
target_link_libraries(ik_test
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
        )
>>>>>>> d4895e3771e61fb031a25039ae990c8c0d6d995f
