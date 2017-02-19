#ifndef EXPORT_DECL_H
#define EXPORT_DECL_H

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
    #ifdef EAGLE_CAMERA_LIBRARY
        #define EAGLE_CAMERA_LIBRARY_EXPORT __declspec(dllexport)
    #else
        #define EAGLE_CAMERA_LIBRARY_EXPORT __declspec(dllimport)
    #endif
#else
    #define EAGLE_CAMERA_LIBRARY_EXPORT
#endif

#endif // EXPORT_DECL_H
