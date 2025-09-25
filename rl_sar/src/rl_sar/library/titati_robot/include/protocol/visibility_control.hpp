#ifndef SOCKET_CAN__VISIBILITY_CONTROL_HPP_
#define SOCKET_CAN__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOCKETCAN_PUBLIC __attribute__ ((dllexport))
    #define SOCKETCAN_LOCAL
  #else
    #define SOCKETCAN_PUBLIC __declspec(dllexport)
    #define SOCKETCAN_LOCAL
  #endif
#else
  #define SOCKETCAN_PUBLIC __attribute__ ((visibility("default")))
  #define SOCKETCAN_LOCAL  __attribute__ ((visibility("hidden")))
#endif

#endif  // SOCKET_CAN__VISIBILITY_CONTROL_HPP_

