cmake_minimum_required(VERSION 3.8)
project(coop_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# building the library
#add_library(Follower src/Follower.cpp)

# linking against rider1
#ament_target_dependencies(Follower rider1)

# find the header files
include_directories(include)

add_executable(rider1 src/rider1.cpp)
ament_target_dependencies(rider1 rclcpp std_msgs geometry_msgs nav_msgs)

#target_link_libraries(rider1 Follower)

install(TARGETS
  rider1
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
