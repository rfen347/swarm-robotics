# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/bc/indigo_workspace3/project1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bc/indigo_workspace3/project1/build

# Include any dependencies generated for this target.
include CMakeFiles/R3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/R3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/R3.dir/flags.make

CMakeFiles/R3.dir/src/resident.cpp.o: CMakeFiles/R3.dir/flags.make
CMakeFiles/R3.dir/src/resident.cpp.o: ../src/resident.cpp
CMakeFiles/R3.dir/src/resident.cpp.o: ../manifest.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/cpp_common/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/rostime/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/roscpp_traits/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/roscpp_serialization/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/genmsg/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/genpy/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/message_runtime/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/std_msgs/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/geometry_msgs/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/gencpp/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/genlisp/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/message_generation/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/actionlib_msgs/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/nav_msgs/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/catkin/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/rosbuild/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/rosconsole/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/rosgraph_msgs/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/xmlrpcpp/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/roscpp/package.xml
CMakeFiles/R3.dir/src/resident.cpp.o: /opt/ros/indigo/share/sensor_msgs/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bc/indigo_workspace3/project1/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/R3.dir/src/resident.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/R3.dir/src/resident.cpp.o -c /home/bc/indigo_workspace3/project1/src/resident.cpp

CMakeFiles/R3.dir/src/resident.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/R3.dir/src/resident.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bc/indigo_workspace3/project1/src/resident.cpp > CMakeFiles/R3.dir/src/resident.cpp.i

CMakeFiles/R3.dir/src/resident.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/R3.dir/src/resident.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bc/indigo_workspace3/project1/src/resident.cpp -o CMakeFiles/R3.dir/src/resident.cpp.s

CMakeFiles/R3.dir/src/resident.cpp.o.requires:
.PHONY : CMakeFiles/R3.dir/src/resident.cpp.o.requires

CMakeFiles/R3.dir/src/resident.cpp.o.provides: CMakeFiles/R3.dir/src/resident.cpp.o.requires
	$(MAKE) -f CMakeFiles/R3.dir/build.make CMakeFiles/R3.dir/src/resident.cpp.o.provides.build
.PHONY : CMakeFiles/R3.dir/src/resident.cpp.o.provides

CMakeFiles/R3.dir/src/resident.cpp.o.provides.build: CMakeFiles/R3.dir/src/resident.cpp.o

# Object files for target R3
R3_OBJECTS = \
"CMakeFiles/R3.dir/src/resident.cpp.o"

# External object files for target R3
R3_EXTERNAL_OBJECTS =

../bin/R3: CMakeFiles/R3.dir/src/resident.cpp.o
../bin/R3: CMakeFiles/R3.dir/build.make
../bin/R3: CMakeFiles/R3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/R3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/R3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/R3.dir/build: ../bin/R3
.PHONY : CMakeFiles/R3.dir/build

CMakeFiles/R3.dir/requires: CMakeFiles/R3.dir/src/resident.cpp.o.requires
.PHONY : CMakeFiles/R3.dir/requires

CMakeFiles/R3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/R3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/R3.dir/clean

CMakeFiles/R3.dir/depend:
	cd /home/bc/indigo_workspace3/project1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bc/indigo_workspace3/project1 /home/bc/indigo_workspace3/project1 /home/bc/indigo_workspace3/project1/build /home/bc/indigo_workspace3/project1/build /home/bc/indigo_workspace3/project1/build/CMakeFiles/R3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/R3.dir/depend

