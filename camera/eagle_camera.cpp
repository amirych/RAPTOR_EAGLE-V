#include "eagle_camera.h"

#include <chrono>


                    /*  Auxiliary functions */

static void assert_call_api(int err_code, const std::string &context)
{
    if ( err_code != EPIX_ERROR_OK ) {
        throw EagleCamera_Exception(err_code, context);
    }
}

static void assert_call_api(int err_code, const char* context)
{
    if ( err_code != EPIX_ERROR_OK ) {
        throw EagleCamera_Exception(err_code, context);
    }
}



static std::string time_stamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    char time_stamp[100];

    struct std::tm buff;
    buff = *std::localtime(&now_c);

    std::strftime(time_stamp,sizeof(time_stamp),"%c",&buff);

    return std::string(time_stamp);
}


                        /************************************************
                        *                                               *
                        *      EagleCamera CLASS IMPLEMENTATION         *
                        *                                               *
                        ************************************************/


                    /*  Static members initialization  */

size_t EagleCamera::createdObjects = 0;


                /*  Constructors and destructors   */

EagleCamera::EagleCamera(const char* epix_video_fmt_filename):
    cameraUnitmap(-1), cameraVideoFormatFilename(""),
    logLevel(EagleCamera::LOG_LEVEL_VERBOSE),
    currentPreAmpGain(EagleCamera::HighGain), currentReadoutRate(EagleCamera::SlowRate),
    currentShutterState(EagleCamera::ShutterExp), currentReadoutMode(EagleCamera::NormalReadoutMode),
    cameraLog(nullptr), lastError(0),
    cameralink_handler()
{
    if ( !createdObjects ) { // open XCLIB library
        if ( epix_video_fmt_filename != nullptr ) {
            lastError = pxd_PIXCIopen("","",epix_video_fmt_filename);
        } else {
            lastError = pxd_PIXCIopen("","","");
        }
    }

    ++createdObjects;

    setLogLevel(logLevel);
}


EagleCamera::EagleCamera(const std::string &epix_video_fmt_filename):
    EagleCamera(epix_video_fmt_filename.c_str())
{

}


EagleCamera::~EagleCamera()
{
    --createdObjects;

    if ( !createdObjects ) {
        pxd_PIXCIclose();
    }
}


void EagleCamera::setLogLevel(const EagleCamera::LogLevel level)
{
    logLevel = level;
}


EagleCamera::LogLevel EagleCamera::getLogevel() const
{
    return logLevel;
}


bool EagleCamera::initCamera(const int unitmap, std::ostream *log_file)
{
    if ( unitmap < 0 ) lastError = PXERBADPARM;
    cameraUnitmap = unitmap;

    cameraLog = log_file;

    std::string log_str;

    // print logging header (ignore logLevel!)
    if ( cameraLog ) {
        for (int i = 0; i < 5; ++i) *cameraLog << std::endl;
        std::string line;
        line.resize(80,'*');
        *cameraLog << line << std::endl;
        *cameraLog << "   " << time_stamp() << std::endl;
        *cameraLog << "   'EAGLE CAMERA' v. " << EAGLE_CAMERA_VERSION_MAJOR << "." << EAGLE_CAMERA_VERSION_MINOR <<
                      " CONTROL SOFTWARE FOR RAPTOR PHOTONICS EAGLE-V 4240 CCD CAMERA" << std::endl;
        *cameraLog << line << std::endl;
        *cameraLog << std::endl << std::flush;
    }

    // init and setup serial connection
    cameralink_handler.setUnitmap(cameraUnitmap);
    cameralink_handler.reset();
    if ( !cameralink_handler.isValid() ) {
        lastError = cameralink_handler.getLastEPIXError();
        return false;
    }

    cameralink_handler.setSystemState(true,true,false,false);

    // setup camera to default state

    return true;
}


void EagleCamera::logToFile(const EagleCamera::LogIdent ident, const std::string &log_str, const int indent_tabs)
{
    if ( !cameraLog ) return;

    if ( logLevel == EagleCamera::LOG_LEVEL_QUIET ) return;

    switch ( ident ) {
    case EagleCamera::LOG_IDENT_BLANK:

        break;
    case EagleCamera::LOG_IDENT_CAMERA_INFO:

        break;
    case EagleCamera::LOG_IDENT_CAMERA_ERROR:

        break;
    case EagleCamera::LOG_IDENT_XCLIB_INFO:

        break;
    case EagleCamera::LOG_IDENT_XCLIB_ERROR:

        break;
    default:
        break;
    }
}


void EagleCamera::logToFile(const EagleCamera_Exception &ex, const int indent_tabs)
{
    std::string log_str = ex.what();

    logToFile(EagleCamera::LOG_IDENT_XCLIB_ERROR, log_str, indent_tabs);
}
