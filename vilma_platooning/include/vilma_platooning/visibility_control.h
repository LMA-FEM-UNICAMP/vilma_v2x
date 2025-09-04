#ifndef VILMA_PLATOONING__VISIBILITY_CONTROL_H_
#define VILMA_PLATOONING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VILMA_PLATOONING_EXPORT __attribute__ ((dllexport))
    #define VILMA_PLATOONING_IMPORT __attribute__ ((dllimport))
  #else
    #define VILMA_PLATOONING_EXPORT __declspec(dllexport)
    #define VILMA_PLATOONING_IMPORT __declspec(dllimport)
  #endif
  #ifdef VILMA_PLATOONING_BUILDING_LIBRARY
    #define VILMA_PLATOONING_PUBLIC VILMA_PLATOONING_EXPORT
  #else
    #define VILMA_PLATOONING_PUBLIC VILMA_PLATOONING_IMPORT
  #endif
  #define VILMA_PLATOONING_PUBLIC_TYPE VILMA_PLATOONING_PUBLIC
  #define VILMA_PLATOONING_LOCAL
#else
  #define VILMA_PLATOONING_EXPORT __attribute__ ((visibility("default")))
  #define VILMA_PLATOONING_IMPORT
  #if __GNUC__ >= 4
    #define VILMA_PLATOONING_PUBLIC __attribute__ ((visibility("default")))
    #define VILMA_PLATOONING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VILMA_PLATOONING_PUBLIC
    #define VILMA_PLATOONING_LOCAL
  #endif
  #define VILMA_PLATOONING_PUBLIC_TYPE
#endif

#endif  // VILMA_PLATOONING__VISIBILITY_CONTROL_H_
