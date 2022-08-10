cmake_minimum_required(VERSION 3.5)
# Package name
project(gpio_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${COLCON_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_executable(gpio src/gpio.cpp)
add_library(mpu9250lib STATIC src/mpu9250_i2c.cpp)
add_executable(gpio src/I2C_test.cpp)
#add_executable(gpio3 src/gpio3.cpp)

target_compile_options(mpu9250lib
  PUBLIC -Wall -pthread
)

target_link_libraries(mpu9250lib
  pigpio pigpiod_if2
)

target_compile_options(gpio
  PUBLIC -Wall -pthread
)

target_link_libraries(gpio
  pigpio pigpiod_if2 mpu9250lib
)

#target_compile_options(gpio3
#  PUBLIC -Wall -pthread
#)

#target_link_libraries(gpio3
#  pigpio pigpiod_if2
#)

ament_target_dependencies(gpio
  rclcpp std_msgs
)

#ament_target_dependencies(gpio3
#  rclcpp std_msgs
#)

install(TARGETS 
  gpio
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
