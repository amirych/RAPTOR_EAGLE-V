#ifndef EAGLE_CAMERA_H
#define EAGLE_CAMERA_H

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
    #include <windows.h>
#endif

#ifdef _MSC_VER
#if (_MSC_VER > 1800)
    #define NOEXCEPT_DECL noexcept
#else
    #define NOEXCEPT_DECL // empty to compile with VS2013
#endif
#else
    #define NOEXCEPT_DECL noexcept
#endif



#include <iostream>
#include <string>
#include <exception>

#include <export_decl.h>
#include <cameralink_handler.h>
#include <eagle_camera_config.h>


                /*****************************************************
                *                                                    *
                *      EagleCamera_exception CLASS DECLARATION       *
                *                                                    *
                *****************************************************/


class EAGLE_CAMERA_LIBRARY_EXPORT EagleCamera_Exception: public std::exception
{
public:
    EagleCamera_Exception(int err_code, const std::string &context);
    EagleCamera_Exception(int err_code, const char* context);

    int getError() const;

    const char* what() const NOEXCEPT_DECL;

private:
    int errCode;
    std::string _context;
};



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

#define EPIX_ERROR_OK 0

#define EAGLE_CAMERA_DEFAULT_LOG_TAB 3 // default tabulation in symbols for logging

#ifdef _WIN32
    #define EPIX_VIDEO_FMT_FILE_PATH "."
#else
    #define EPIX_VIDEO_FMT_FILE_PATH "."
#endif

class EAGLE_CAMERA_LIBRARY_EXPORT EagleCamera
{
public:
    enum PreAmpGain {UnknownGain = -1, HighGain, LowGain};
    enum ReadoutRate {UnknownRate = -1, FastRate, SlowRate}; // FastRate - 2MHz, LowRate - 75 kHz
    enum ShutterState {UnknownShutterState = -1, ShutterClosed, ShutterOpen, ShutterExp};
    enum ReadoutMode {UnknownReadoutMode = -1, NormalReadoutMode, TestReadoutMode};

    enum LogLevel {LOG_LEVEL_QUIET, LOG_LEVEL_ERROR, LOG_LEVEL_VERBOSE};
    enum LogIdent {LOG_IDENT_BLANK, LOG_IDENT_CAMERA_INFO, LOG_IDENT_CAMERA_ERROR, LOG_IDENT_XCLIB_INFO, LOG_IDENT_XCLIB_ERROR};

    EagleCamera(const char* epix_video_fmt_filename = nullptr);
    EagleCamera(const std::string &epix_video_fmt_filename);

    virtual ~EagleCamera();

    bool initCamera(const int unitmap = 1, std::ostream *log_file = nullptr);

    void setLogLevel(const EagleCamera::LogLevel level);
    EagleCamera::LogLevel getLogevel() const;

    void logToFile(const EagleCamera::LogIdent ident, const std::string &log_str, const int indent_tabs = 0);
    void logToFile(const EagleCamera_Exception &ex, const int indent_tabs = 0);

protected:
                /*  camera configuration */

    int cameraUnitmap;
    std::string cameraVideoFormatFilename;

    EagleCamera::LogLevel logLevel;

    EagleCamera::PreAmpGain currentPreAmpGain;
    EagleCamera::ReadoutRate currentReadoutRate;
    EagleCamera::ShutterState currentShutterState;
    EagleCamera::ReadoutMode currentReadoutMode;

    std::ostream *cameraLog;
    int lastError;

        /*  members and methods to work with CameraLink serial connection  */

    CameraLinkHandler cameralink_handler;

                /*  static members and methods  */

    static size_t createdObjects;
};



#endif // EAGLE_CAMERA_H
