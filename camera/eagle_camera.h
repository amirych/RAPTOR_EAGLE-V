#ifndef EAGLE_CAMERA_H
#define EAGLE_CAMERA_H

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) // needs for xcliball.h
    #include <windows.h>
#endif


#ifdef _MSC_VER
    // for MS compilers: disable multiple warnings about DLL-exports for the STL containers
    // (and many others C++11 defined classes, e.g., thread)
    #pragma warning( disable: 4251 )
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
#include <cstdint>

#include <export_decl.h>
#include <cameralink_handler.h>
#include <eagle_camera_config.h>


// just forward declaration
class EAGLE_CAMERA_LIBRARY_EXPORT EagleCamera_Exception;
class EAGLE_CAMERA_LIBRARY_EXPORT XCLIB_Exception;




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

#define EAGLE_CAMERA_VIDEO_FMT_FILENAME "raptor_eagle-v.fmt"

#define XCLIB_ERROR_OK 0

#define EAGLE_CAMERA_DEFAULT_LOG_TAB 3 // default tabulation in symbols for logging

#ifdef _WIN32
    #define EAGLE_CAMERA_VIDEO_FMT_FILE_PATH "."
#else
    #define EAGLE_CAMERA_VIDEO_FMT_FILE_PATH "."
#endif

class EAGLE_CAMERA_LIBRARY_EXPORT EagleCamera
{
public:
    enum Error {ERROR_OK, ERROR_CONTROLLER_ANSWER, ERROR_BAD_PARAM};
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


    void startExposure();
    void stopExposure();


    void setExposure(const double exp_time);
    double getExposure();

    void setBinning(const uint8_t xbin, const uint8_t ybin);
    void getBinning(uint8_t *xbin, uint8_t *ybin);

    // xstart and ystart are in CCD pixels. width and height are in binned pixels
    // pixel coordinates start from 1
    void setROI(const uint16_t xstart, const uint16_t ystart, const uint16_t width, const uint16_t height);
    void getROI(uint16_t *xstart, uint16_t *ystart, uint16_t *width, uint16_t *height);

    void setReadoutRate(const EagleCamera::ReadoutRate rate);
    EagleCamera::ReadoutRate getReadoutRate();

    void setGain(const EagleCamera::PreAmpGain gain);
    EagleCamera::PreAmpGain getGain();

    void enableTEC(const bool flag = true);
    bool isTECEnabled();

    void logToFile(const EagleCamera::LogIdent ident, const std::string &log_str, const int indent_tabs = 0);
    void logToFile(const XCLIB_Exception &ex, const int indent_tabs = 0);

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

    void logXCLibCall(const std::string &log_str);

        /*  members and methods to work with CameraLink serial connection  */

    unsigned char getCtrlRegister();
    unsigned char getTriggerRegister();

    CameraLinkHandler cameralink_handler;

    CameraLinkHandler::byte_array_t readContinuousRegisters(const CameraLinkHandler::byte_array_t &addr);
    void writeContinuousRegisters(const CameraLinkHandler::byte_array_t &addr, const CameraLinkHandler::byte_array_t &values);

                /*  static members and methods  */

    static size_t createdObjects;
};



            /*****************************************************
            *                                                    *
            *      EagleCamera_Exception CLASS DECLARATION       *
            *                                                    *
            *****************************************************/


class EAGLE_CAMERA_LIBRARY_EXPORT EagleCamera_Exception: public std::exception
{
public:
    EagleCamera_Exception(const EagleCamera::Error err, const std::string &context, const byte contr_ans = CL_ETX);
    EagleCamera_Exception(const EagleCamera::Error err, const char* context, const byte contr_ans = CL_ETX);

    EagleCamera::Error getError() const;
    char getControllerAnswer() const;     // controller answer for serial CameraLink read operation

    const char* what() const NOEXCEPT_DECL;
private:
    EagleCamera::Error error;
    std::string _context;
    char controllerAnswer;
};


            /***********************************************
            *                                              *
            *      XCLIB_Exception CLASS DECLARATION       *
            *                                              *
            ***********************************************/

class EAGLE_CAMERA_LIBRARY_EXPORT XCLIB_Exception: public std::exception
{
public:
    XCLIB_Exception(int err_code, const std::string &context);
    XCLIB_Exception(int err_code, const char* context);

    int getError() const;

    const char* what() const NOEXCEPT_DECL;

private:
    int errCode;
    std::string _context;
};


#endif // EAGLE_CAMERA_H
