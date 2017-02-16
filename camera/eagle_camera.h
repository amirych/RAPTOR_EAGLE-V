#ifndef EAGLE_CAMERA_H
#define EAGLE_CAMERA_H

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
    #include <windows.h>
#endif

#include <string>

                 /*********************************************
                 *                                            *
                 *      EagleCamera CLASS DECLARATION         *
                 *                                            *
                 *    An C++ wrapper class for control of     *
                 *  Raptor Photonics Eagle-V 4240 CCD Camera  *
                 *                                            *
                 *********************************************/



                        /*    DEFINITIONS    */

// default filename of video format file and its filesystem path

#define EPIX_VIDEO_FMT_FILENAME "raptor_eagle-v.fmt"

#ifdef _WIN32
    #define EPIX_VIDEO_FMT_FILE_PATH "."
#else
    #define EPIX_VIDEO_FMT_FILE_PATH "."
#endif

class EagleCamera
{
public:
    enum PreAmpGain {UnknownGain = -1, HighGain, LowGain};
    enum ReadoutRate {UnknownRate = -1, FastRate, SlowRate}; // FastRate - 2MHz, LowRate - 75 kHz
    enum ShutterState {UnknownShutterState = -1, ShutterClosed, ShutterOpen, ShutterExp};
    enum ReadoutMode {UnknownReadoutMode = -1, NormalReadoutMode, TestReadoutMode};

    EagleCamera();
    EagleCamera(const std::string &epix_video_fmt_filename);
    EagleCamera(const char* epix_video_fmt_filename);

protected:
    /*  camera configuration */
    EagleCamera::PreAmpGain currentPreAmpGain;
    EagleCamera::ReadoutRate currentReadoutRate;
    EagleCamera::ShutterState currentShutterState;
    EagleCamera::ReadoutMode currentReadoutMode;

    /*  methods to work with CameraLink serial connection  */

    char cl_CurrentSystemStatus;
    char cl_CurrentFPGAStatus;

    int read_CLpacket(const char* packet, const size_t packet_len);
    int write_CLpacket(const char* packet, const size_t packet_len);

    int resetFirmware();
};

#endif // EAGLE_CAMERA_H
