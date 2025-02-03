#ifndef deli_hardware_interface__VISIBILITY_CONTROL_H_
#define deli_hardware_interface__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define deli_hardware_interface_EXPORT __attribute__((dllexport))
#define deli_hardware_interface_IMPORT __attribute__((dllimport))
#else
#define deli_hardware_interface_EXPORT __declspec(dllexport)
#define deli_hardware_interface_IMPORT __declspec(dllimport)
#endif
#ifdef deli_hardware_interface_BUILDING_DLL
#define deli_hardware_interface_PUBLIC deli_hardware_interface_EXPORT
#else
#define deli_hardware_interface_PUBLIC deli_hardware_interface_IMPORT
#endif
#define deli_hardware_interface_PUBLIC_TYPE deli_hardware_interface_PUBLIC
#define deli_hardware_interface_LOCAL
#else
#define deli_hardware_interface_EXPORT __attribute__((visibility("default")))
#define deli_hardware_interface_IMPORT
#if __GNUC__ >= 4
#define deli_hardware_interface_PUBLIC __attribute__((visibility("default")))
#define deli_hardware_interface_LOCAL __attribute__((visibility("hidden")))
#else
#define deli_hardware_interface_PUBLIC
#define deli_hardware_interface_LOCAL
#endif
#define deli_hardware_interface_PUBLIC_TYPE
#endif

#endif  // deli_hardware_interface__VISIBILITY_CONTROL_H_
