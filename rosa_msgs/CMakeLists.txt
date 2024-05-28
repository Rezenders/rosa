cmake_minimum_required(VERSION 3.8)
project(rosa_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/Component.msg"
  "msg/ComponentConfiguration.msg"
  "msg/ComponentProcess.msg"
  "msg/Function.msg"
  "msg/FunctionDesign.msg"
  "msg/FunctionalRequirement.msg"
  "msg/ReconfigurationPlan.msg"
  "srv/ActionQuery.srv"
  "srv/ActionQueryArray.srv"
  "srv/AdaptableFunctions.srv"
  "srv/AdaptableComponents.srv"
  "srv/ComponentQuery.srv"
  "srv/ComponentProcessQuery.srv"
  "srv/ComponentProcessQueryArray.srv"
  "srv/FunctionQuery.srv"
  "srv/FunctionDesignQuery.srv"
  "srv/FunctionalRequirementQuery.srv"
  "srv/GetComponentParameters.srv"
  "srv/GetComponentConfigurationPriority.srv"
  "srv/GetFunctionDesignPriority.srv"
  "srv/ReconfigurationPlanQuery.srv"
  "srv/SelectableComponentConfigurations.srv"
  "srv/SelectableFunctionDesigns.srv"
  "srv/SetReconfigurationPlan.srv"
  "srv/StructuralAdaptation.srv"
  DEPENDENCIES std_msgs rcl_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
