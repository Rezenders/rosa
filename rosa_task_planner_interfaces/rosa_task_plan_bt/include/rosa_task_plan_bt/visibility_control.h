#ifndef ROSA_TASK_PLAN_BT__VISIBILITY_CONTROL_H_
#define ROSA_TASK_PLAN_BT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSA_TASK_PLAN_BT_EXPORT __attribute__ ((dllexport))
    #define ROSA_TASK_PLAN_BT_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSA_TASK_PLAN_BT_EXPORT __declspec(dllexport)
    #define ROSA_TASK_PLAN_BT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSA_TASK_PLAN_BT_BUILDING_LIBRARY
    #define ROSA_TASK_PLAN_BT_PUBLIC ROSA_TASK_PLAN_BT_EXPORT
  #else
    #define ROSA_TASK_PLAN_BT_PUBLIC ROSA_TASK_PLAN_BT_IMPORT
  #endif
  #define ROSA_TASK_PLAN_BT_PUBLIC_TYPE ROSA_TASK_PLAN_BT_PUBLIC
  #define ROSA_TASK_PLAN_BT_LOCAL
#else
  #define ROSA_TASK_PLAN_BT_EXPORT __attribute__ ((visibility("default")))
  #define ROSA_TASK_PLAN_BT_IMPORT
  #if __GNUC__ >= 4
    #define ROSA_TASK_PLAN_BT_PUBLIC __attribute__ ((visibility("default")))
    #define ROSA_TASK_PLAN_BT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSA_TASK_PLAN_BT_PUBLIC
    #define ROSA_TASK_PLAN_BT_LOCAL
  #endif
  #define ROSA_TASK_PLAN_BT_PUBLIC_TYPE
#endif

#endif  // ROSA_TASK_PLAN_BT__VISIBILITY_CONTROL_H_
