# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/zm/setup/clion-2018.2.6/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zm/setup/clion-2018.2.6/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zm/uam_ws/src/uam_arm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zm/uam_ws/src/uam_arm/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/TRACIKsolver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TRACIKsolver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TRACIKsolver.dir/flags.make

CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o: CMakeFiles/TRACIKsolver.dir/flags.make
CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o: ../src/TRACIKsolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zm/uam_ws/src/uam_arm/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o -c /home/zm/uam_ws/src/uam_arm/src/TRACIKsolver.cpp

CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zm/uam_ws/src/uam_arm/src/TRACIKsolver.cpp > CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.i

CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zm/uam_ws/src/uam_arm/src/TRACIKsolver.cpp -o CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.s

# Object files for target TRACIKsolver
TRACIKsolver_OBJECTS = \
"CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o"

# External object files for target TRACIKsolver
TRACIKsolver_EXTERNAL_OBJECTS =

devel/lib/uam_arm/TRACIKsolver: CMakeFiles/TRACIKsolver.dir/src/TRACIKsolver.cpp.o
devel/lib/uam_arm/TRACIKsolver: CMakeFiles/TRACIKsolver.dir/build.make
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libtrac_ik.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libkdl_parser.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/liburdf.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/librosconsole_bridge.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/librostime.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/uam_arm/TRACIKsolver: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/uam_arm/TRACIKsolver: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/uam_arm/TRACIKsolver: CMakeFiles/TRACIKsolver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zm/uam_ws/src/uam_arm/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/uam_arm/TRACIKsolver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TRACIKsolver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TRACIKsolver.dir/build: devel/lib/uam_arm/TRACIKsolver

.PHONY : CMakeFiles/TRACIKsolver.dir/build

CMakeFiles/TRACIKsolver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TRACIKsolver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TRACIKsolver.dir/clean

CMakeFiles/TRACIKsolver.dir/depend:
	cd /home/zm/uam_ws/src/uam_arm/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zm/uam_ws/src/uam_arm /home/zm/uam_ws/src/uam_arm /home/zm/uam_ws/src/uam_arm/cmake-build-debug /home/zm/uam_ws/src/uam_arm/cmake-build-debug /home/zm/uam_ws/src/uam_arm/cmake-build-debug/CMakeFiles/TRACIKsolver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TRACIKsolver.dir/depend

