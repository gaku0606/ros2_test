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
include CMakeFiles/gpio.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpio.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpio.dir/flags.make

CMakeFiles/gpio.dir/src/I2C_test.cpp.o: CMakeFiles/gpio.dir/flags.make
CMakeFiles/gpio.dir/src/I2C_test.cpp.o: /home/pi4/ros2_test/src/gpio_test/src/I2C_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi4/ros2_test/build/gpio_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpio.dir/src/I2C_test.cpp.o"
	/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpio.dir/src/I2C_test.cpp.o -c /home/pi4/ros2_test/src/gpio_test/src/I2C_test.cpp

CMakeFiles/gpio.dir/src/I2C_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpio.dir/src/I2C_test.cpp.i"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi4/ros2_test/src/gpio_test/src/I2C_test.cpp > CMakeFiles/gpio.dir/src/I2C_test.cpp.i

CMakeFiles/gpio.dir/src/I2C_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpio.dir/src/I2C_test.cpp.s"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi4/ros2_test/src/gpio_test/src/I2C_test.cpp -o CMakeFiles/gpio.dir/src/I2C_test.cpp.s

# Object files for target gpio
gpio_OBJECTS = \
"CMakeFiles/gpio.dir/src/I2C_test.cpp.o"

# External object files for target gpio
gpio_EXTERNAL_OBJECTS =

gpio: CMakeFiles/gpio.dir/src/I2C_test.cpp.o
gpio: CMakeFiles/gpio.dir/build.make
gpio: libmpu9250lib.a
gpio: /opt/ros/foxy/lib/librclcpp.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/librcl.so
gpio: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/librmw_implementation.so
gpio: /opt/ros/foxy/lib/librmw.so
gpio: /opt/ros/foxy/lib/librcl_logging_spdlog.so
gpio: /usr/lib/aarch64-linux-gnu/libspdlog.so.1.5.0
gpio: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
gpio: /opt/ros/foxy/lib/libyaml.so
gpio: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
gpio: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
gpio: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
gpio: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
gpio: /opt/ros/foxy/lib/librosidl_typesupport_c.so
gpio: /opt/ros/foxy/lib/librcpputils.so
gpio: /opt/ros/foxy/lib/librosidl_runtime_c.so
gpio: /opt/ros/foxy/lib/librcutils.so
gpio: /opt/ros/foxy/lib/libtracetools.so
gpio: CMakeFiles/gpio.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi4/ros2_test/build/gpio_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gpio"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpio.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpio.dir/build: gpio

.PHONY : CMakeFiles/gpio.dir/build

CMakeFiles/gpio.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpio.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpio.dir/clean

CMakeFiles/gpio.dir/depend:
	cd /home/pi4/ros2_test/build/gpio_test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi4/ros2_test/src/gpio_test /home/pi4/ros2_test/src/gpio_test /home/pi4/ros2_test/build/gpio_test /home/pi4/ros2_test/build/gpio_test /home/pi4/ros2_test/build/gpio_test/CMakeFiles/gpio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpio.dir/depend
