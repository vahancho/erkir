#ifndef __EXPORT_H_
#define __EXPORT_H_

#ifdef _WIN32
    #ifdef MAKEDLL
    #   define ERKIR_EXPORT __declspec(dllexport)
    #else
    #   define ERKIR_EXPORT __declspec(dllimport)
    #endif
#else
    #define RX_LOG_EXPORT
#endif

#endif // __EXPORT_H_
