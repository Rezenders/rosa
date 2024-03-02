#ifndef ROSA_TASK_PLAN_PLANSYS__VISIBILITY_CONTROL_H_
#define ROSA_TASK_PLAN_PLANSYS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSA_TASK_PLAN_PLANSYS_EXPORT __attribute__ ((dllexport))
    #define ROSA_TASK_PLAN_PLANSYS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSA_TASK_PLAN_PLANSYS_EXPORT __declspec(dllexport)
    #define ROSA_TASK_PLAN_PLANSYS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSA_TASK_PLAN_PLANSYS_BUILDING_LIBRARY
    #define ROSA_TASK_PLAN_PLANSYS_PUBLIC ROSA_TASK_PLAN_PLANSYS_EXPORT
  #else
    #define ROSA_TASK_PLAN_PLANSYS_PUBLIC ROSA_TASK_PLAN_PLANSYS_IMPORT
  #endif
  #define ROSA_TASK_PLAN_PLANSYS_PUBLIC_TYPE ROSA_TASK_PLAN_PLANSYS_PUBLIC
  #define ROSA_TASK_PLAN_PLANSYS_LOCAL
#else
  #define ROSA_TASK_PLAN_PLANSYS_EXPORT __attribute__ ((visibility("default")))
  #define ROSA_TASK_PLAN_PLANSYS_IMPORT
  #if __GNUC__ >= 4
    #define ROSA_TASK_PLAN_PLANSYS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSA_TASK_PLAN_PLANSYS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSA_TASK_PLAN_PLANSYS_PUBLIC
    #define ROSA_TASK_PLAN_PLANSYS_LOCAL
  #endif
  #define ROSA_TASK_PLAN_PLANSYS_PUBLIC_TYPE
#endif

#endif  // ROSA_TASK_PLAN_PLANSYS__VISIBILITY_CONTROL_H_
