cmake_minimum_required(VERSION 3.8)
project(rosa_task_plan_plansys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(rosa_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_cascade_lifecycle
  rclcpp_action
  lifecycle_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_msgs
  plansys2_pddl_parser
  plansys2_problem_expert
  plansys2_planner
  rosa_msgs
)


add_library(rosa_plansys_controller src/rosa_plansys_controller.cpp)
target_compile_features(rosa_plansys_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rosa_plansys_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rosa_plansys_controller ${dependencies})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosa_plansys_controller PRIVATE "ROSA_TASK_PLAN_PLANSYS_BUILDING_LIBRARY")

add_library(rosa_action src/rosa_action.cpp)
target_compile_features(rosa_action PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rosa_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rosa_action ${dependencies})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosa_action PRIVATE "ROSA_TASK_PLAN_PLANSYS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rosa_plansys_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS rosa_action
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(rosa_plansys_controller_node src/rosa_plansys_controller_node.cpp)
target_include_directories(rosa_plansys_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rosa_plansys_controller_node rosa_plansys_controller)

install(TARGETS rosa_plansys_controller_node
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  rosa_plansys_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})

ament_package()
