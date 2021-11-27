cmake_minimum_required(VERSION 3.5)
project(major_tom_cpp)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
 # set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
#endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 # add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})


include_directories(include
  ${apriltag_INCLUDE_DIRS}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.2 REQUIRED) #3.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag REQUIRED)
#find_package(apriltag_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(airsim_img_listener src/show_img.cpp)
ament_target_dependencies(airsim_img_listener rclcpp rclcpp_components std_msgs sensor_msgs OpenCV cv_bridge apriltag tf2_msgs)


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

install(TARGETS
  airsim_img_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
