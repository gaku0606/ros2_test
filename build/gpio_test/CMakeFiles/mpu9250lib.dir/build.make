# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pi4/ros2_test/src/gpio_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi4/ros2_test/build/gpio_test

# Include any dependencies generated for this target.
include CMakeFiles/mpu9250lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mpu9250lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mpu9250lib.dir/flags.make

CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o: CMakeFiles/mpu9250lib.dir/flags.make
CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o: /home/pi4/ros2_test/src/gpio_test/src/mpu9250_i2c.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi4/ros2_test/build/gpio_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o"
	/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o -c /home/pi4/ros2_test/src/gpio_test/src/mpu9250_i2c.cpp

CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.i"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi4/ros2_test/src/gpio_test/src/mpu9250_i2c.cpp > CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.i

CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.s"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi4/ros2_test/src/gpio_test/src/mpu9250_i2c.cpp -o CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.s

# Object files for target mpu9250lib
mpu9250lib_OBJECTS = \
"CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o"

# External object files for target mpu9250lib
mpu9250lib_EXTERNAL_OBJECTS =

libmpu9250lib.a: CMakeFiles/mpu9250lib.dir/src/mpu9250_i2c.cpp.o
libmpu9250lib.a: CMakeFiles/mpu9250lib.dir/build.make
libmpu9250lib.a: CMakeFiles/mpu9250lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi4/ros2_test/build/gpio_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmpu9250lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/mpu9250lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpu9250lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mpu9250lib.dir/build: libmpu9250lib.a

.PHONY : CMakeFiles/mpu9250lib.dir/build

CMakeFiles/mpu9250lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mpu9250lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mpu9250lib.dir/clean

CMakeFiles/mpu9250lib.dir/depend:
	cd /home/pi4/ros2_test/build/gpio_test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi4/ros2_test/src/gpio_test /home/pi4/ros2_test/src/gpio_test /home/pi4/ros2_test/build/gpio_test /home/pi4/ros2_test/build/gpio_test /home/pi4/ros2_test/build/gpio_test/CMakeFiles/mpu9250lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mpu9250lib.dir/depend

