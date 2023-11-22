cmake_minimum_required(VERSION 3.8)
project(nucleo_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(nucleo_agent src/nucleo_agent.cpp)

target_include_directories(nucleo_agent PRIVATE ${UDEV_INCLUDE_DIRS})
target_link_libraries(nucleo_agent ${UDEV_LIBRARIES})

target_include_directories(nucleo_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nucleo_agent PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  nucleo_agent
  "rclcpp"
  "std_msgs"
  "rosidl_default_generators"
)

install(TARGETS nucleo_agent
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
