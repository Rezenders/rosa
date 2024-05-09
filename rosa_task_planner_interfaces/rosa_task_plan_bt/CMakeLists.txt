cmake_minimum_required(VERSION 3.8)
project(rosa_task_plan_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rosa_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  std_msgs
  behaviortree_cpp
  rosa_msgs
)

add_library(rosa_action src/rosa_action.cpp)
target_compile_features(rosa_action PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rosa_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rosa_action ${dependencies})

add_library(is_action_feasible src/is_action_feasible.cpp)
target_compile_features(is_action_feasible PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(is_action_feasible PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(is_action_feasible ${dependencies})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosa_action PRIVATE "ROSA_TASK_PLAN_BT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rosa_action is_action_feasible
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  rosa_action
  is_action_feasible
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
