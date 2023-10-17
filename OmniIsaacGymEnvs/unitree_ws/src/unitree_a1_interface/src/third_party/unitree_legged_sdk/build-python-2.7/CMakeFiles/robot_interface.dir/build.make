# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7

# Include any dependencies generated for this target.
include CMakeFiles/robot_interface.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robot_interface.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robot_interface.dir/flags.make

CMakeFiles/robot_interface.dir/python_interface.cpp.o: CMakeFiles/robot_interface.dir/flags.make
CMakeFiles/robot_interface.dir/python_interface.cpp.o: ../python_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/robot_interface.dir/python_interface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robot_interface.dir/python_interface.cpp.o -c /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/python_interface.cpp

CMakeFiles/robot_interface.dir/python_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot_interface.dir/python_interface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/python_interface.cpp > CMakeFiles/robot_interface.dir/python_interface.cpp.i

CMakeFiles/robot_interface.dir/python_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot_interface.dir/python_interface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/python_interface.cpp -o CMakeFiles/robot_interface.dir/python_interface.cpp.s

CMakeFiles/robot_interface.dir/python_interface.cpp.o.requires:

.PHONY : CMakeFiles/robot_interface.dir/python_interface.cpp.o.requires

CMakeFiles/robot_interface.dir/python_interface.cpp.o.provides: CMakeFiles/robot_interface.dir/python_interface.cpp.o.requires
	$(MAKE) -f CMakeFiles/robot_interface.dir/build.make CMakeFiles/robot_interface.dir/python_interface.cpp.o.provides.build
.PHONY : CMakeFiles/robot_interface.dir/python_interface.cpp.o.provides

CMakeFiles/robot_interface.dir/python_interface.cpp.o.provides.build: CMakeFiles/robot_interface.dir/python_interface.cpp.o


# Object files for target robot_interface
robot_interface_OBJECTS = \
"CMakeFiles/robot_interface.dir/python_interface.cpp.o"

# External object files for target robot_interface
robot_interface_EXTERNAL_OBJECTS =

robot_interface.so: CMakeFiles/robot_interface.dir/python_interface.cpp.o
robot_interface.so: CMakeFiles/robot_interface.dir/build.make
robot_interface.so: CMakeFiles/robot_interface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module robot_interface.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot_interface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robot_interface.dir/build: robot_interface.so

.PHONY : CMakeFiles/robot_interface.dir/build

CMakeFiles/robot_interface.dir/requires: CMakeFiles/robot_interface.dir/python_interface.cpp.o.requires

.PHONY : CMakeFiles/robot_interface.dir/requires

CMakeFiles/robot_interface.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robot_interface.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robot_interface.dir/clean

CMakeFiles/robot_interface.dir/depend:
	cd /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7 /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7 /home/unitree/catkin_ws/src/unitree_a1_interface/src/third_party/unitree_legged_sdk/build-python-2.7/CMakeFiles/robot_interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robot_interface.dir/depend

