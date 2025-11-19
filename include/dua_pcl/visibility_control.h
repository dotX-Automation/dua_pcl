#ifndef DUA_PCL__VISIBILITY_CONTROL_H_
#define DUA_PCL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUA_PCL_EXPORT __attribute__ ((dllexport))
    #define DUA_PCL_IMPORT __attribute__ ((dllimport))
  #else
    #define DUA_PCL_EXPORT __declspec(dllexport)
    #define DUA_PCL_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUA_PCL_BUILDING_LIBRARY
    #define DUA_PCL_PUBLIC DUA_PCL_EXPORT
  #else
    #define DUA_PCL_PUBLIC DUA_PCL_IMPORT
  #endif
  #define DUA_PCL_PUBLIC_TYPE DUA_PCL_PUBLIC
  #define DUA_PCL_LOCAL
#else
  #define DUA_PCL_EXPORT __attribute__ ((visibility("default")))
  #define DUA_PCL_IMPORT
  #if __GNUC__ >= 4
    #define DUA_PCL_PUBLIC __attribute__ ((visibility("default")))
    #define DUA_PCL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUA_PCL_PUBLIC
    #define DUA_PCL_LOCAL
  #endif
  #define DUA_PCL_PUBLIC_TYPE
#endif

#endif  // DUA_PCL__VISIBILITY_CONTROL_H_
