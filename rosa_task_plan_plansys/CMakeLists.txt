cmake_minimum_required(VERSION 3.8)
project(rosa_task_plan_plansys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(rosa_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rosa_action_plansys src/rosa_action_plansys.cpp)
target_compile_features(rosa_action_plansys PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rosa_action_plansys PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  rosa_action_plansys
  "plansys2_msgs"
  "plansys2_executor"
  "plansys2_problem_expert"
  "rosa_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosa_action_plansys PRIVATE "ROSA_TASK_PLAN_PLANSYS_BUILDING_LIBRARY")

add_library(rosa_plansys_controller src/rosa_plansys_controller.cpp)
target_compile_features(rosa_plansys_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rosa_plansys_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  rosa_plansys_controller
  "plansys2_msgs"
  "plansys2_executor"
  "plansys2_problem_expert"
  "rosa_msgs"
  "std_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosa_plansys_controller PRIVATE "ROSA_TASK_PLAN_PLANSYS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rosa_action_plansys rosa_plansys_controller
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
  rosa_action_plansys
  rosa_plansys_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
