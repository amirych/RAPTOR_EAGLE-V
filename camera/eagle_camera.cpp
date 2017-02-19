#include "eagle_camera.h"

#include <chrono>
#include <ctime>


                    /*  Auxiliary functions */


int XCLIB_API_CALL(int err_code, const char *context)
{
    if ( err_code < 0 ) {
        throw XCLIB_Exception(err_code, context);
    }

    return err_code;
}

int XCLIB_API_CALL(int err_code, const std::string &context)
{
    if ( err_code < 0 ) {
        throw XCLIB_Exception(err_code, context);
    }

    return err_code;
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

std::string pointer_to_str(void* ptr)
{
    char addr[20];
#ifdef _MSC_VER
    int n = _snprintf_s(addr,20,"%p",ptr);
#else
    int n = snprintf(addr,20,"%p",ptr);
#endif
    return std::string(addr);
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
    cameraLog(nullptr), lastError(XCLIB_ERROR_OK),
    _serialNumber(0), _buildDate(), _buildCode(),
    _microVersion(), _FPGA_Version(),
    ADC_Calibration(), ADC_LinearCoeffs(),
    DAC_Calibration(), DAC_LinearCoeffs(),
    cameralink_handler()
{
    if ( !createdObjects ) { // open XCLIB library
        if ( epix_video_fmt_filename != nullptr ) {
            std::string log_str = std::string("pxd_PIXCIopen(\"\",\"\",") + epix_video_fmt_filename + ")";
            XCLIB_API_CALL( lastError = pxd_PIXCIopen("","",epix_video_fmt_filename), log_str);
        } else {
            std::string log_str = "pxd_PIXCIopen(\"\",\"\",\"\")";
            XCLIB_API_CALL( lastError = pxd_PIXCIopen("","",""), log_str);
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


                                /*  PUBLIC METHODS  */


void EagleCamera::setLogLevel(const EagleCamera::LogLevel level)
{
    logLevel = level;
    if ( logLevel == EagleCamera::LOG_LEVEL_VERBOSE ) {
        CameraLinkHandler::log_func_t log_func = std::bind(
                    static_cast<void(EagleCamera::*)(const std::string&)>
                    (&EagleCamera::logXCLibCall), this, std::placeholders::_1);

        cameralink_handler.setLogFunc(log_func);
    } else cameralink_handler.setLogFunc();
}


EagleCamera::LogLevel EagleCamera::getLogevel() const
{
    return logLevel;
}


bool EagleCamera::initCamera(const int unitmap, std::ostream *log_file)
{
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

    logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "INITIALIZATION OF CCD CAMERA ...");
    logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Try to initialize CameraLink serial connection ...", 1);

    lastError = XCLIB_ERROR_OK;

    // init and setup serial connection

    _serialNumber = 0;
    _buildDate.clear();
    _buildCode.clear();
    memset(ADC_Calibration, 0, sizeof(ADC_Calibration));
    memset(DAC_Calibration, 0, sizeof(DAC_Calibration));

    try {
        cameralink_handler.setUnitmap(cameraUnitmap);

        // setup camera to default state
        cameralink_handler.setSystemState(true,true,false,false);

        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "CameraLink serial connection is established", 1);

        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Try to get manufacturer's data ...", 1);
        getManufacturerData();
        getVersions();

        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Found Raptor CCD camera: ", 1);
        int ntab = 3;
        log_str = "Serial number: " + std::to_string(_serialNumber);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, log_str, ntab);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Build date (DD/MM/YY): " + _buildDate, ntab);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Build code: " + _buildCode, ntab);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Micro version: " + _microVersion, ntab);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "FPGA version: " + _FPGA_Version, ntab);

        logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "Try to set default mode ...", 1);

        setBinning(1, 1);
        setROI(1, 1, EAGLE_CAMERA_CCD_WIDTH, EAGLE_CAMERA_CCD_HEIGHT);
        setReadoutRate(EagleCamera::SlowRate);
        setReadoutMode(EagleCamera::NormalReadoutMode);
        setShutterState(EagleCamera::ShutterExp);

    } catch ( XCLIB_Exception &ex ) {
        lastError = ex.getError();
        logToFile(ex);
        logToFile(EagleCamera::LOG_IDENT_CAMERA_ERROR, "CANNOT INITIALIZE CAMERA");
        return false;
    } catch (EagleCamera_Exception &ex) {
        logToFile(EagleCamera::LOG_IDENT_CAMERA_ERROR, std::string(ex.what()));
        logToFile(EagleCamera::LOG_IDENT_CAMERA_ERROR, "CANNOT INITIALIZE CAMERA");
        return false;
    }


    logToFile(EagleCamera::LOG_IDENT_CAMERA_INFO, "INITIALIZATION COMPLETED SUCCESSFULLY");

    return true;
}



void EagleCamera::startExposure()
{

}


void EagleCamera::stopExposure()
{
}


void EagleCamera::setExposure(const double exp_time)
{
    if ( exp_time < 0 ) {
        std::string log_str = "Exposure time must be greater than 0! (trying to set to " + std::to_string(exp_time) + ")";
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
    }

    uint64_t counts = static_cast<uint64_t>(exp_time/2.5E-8); // in FPGA counts, 1 count = 25nsecs = 1/40MHz

    CameraLinkHandler::byte_array_t addr = {0xED, 0xEE, 0xEF, 0xF0, 0xF1};
    CameraLinkHandler::byte_array_t value;

    for ( size_t i = 4; i; --i ) {
        value.push_back(counts & (0xFF << i*8));
    }

    writeContinuousRegisters(addr, value);
}


double EagleCamera::getExposure()
{
    CameraLinkHandler::byte_array_t addr = {0xED, 0xEE, 0xEF, 0xF0, 0xF1};

    CameraLinkHandler::byte_array_t res = readContinuousRegisters(addr);

    uint64_t counts = 0;

    size_t i = addr.size()-1;
    for ( auto b: res) {
        counts += b << (i*8);
        --i;
    }

    return 2.5E-8*counts;
}


void EagleCamera::setBinning(const uint8_t xbin, const uint8_t ybin)
{
    std::string log_str;

    // binning value must be in range of [1,MAX_BIN]

    if ( (xbin < 1) || (xbin > EAGLE_CAMERA_MAX_XBIN) ) {
        log_str = "XBIN value must be within 1 and " + std::to_string(EAGLE_CAMERA_MAX_XBIN) +
                "! (trying to set to " + std::to_string(xbin) + ")";
        logToFile(EagleCamera::LOG_IDENT_CAMERA_ERROR, log_str);
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
    }
    if ( (ybin < 1) || (ybin > EAGLE_CAMERA_MAX_YBIN) ) {
        log_str = "YBIN value must be within 1 and " + std::to_string(EAGLE_CAMERA_MAX_YBIN) +
                "! (trying to set to " + std::to_string(ybin) + ")";
        logToFile(EagleCamera::LOG_IDENT_CAMERA_ERROR, log_str);
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
    }

    CameraLinkHandler::byte_array_t addr = {0xA1, 0xA2};
    CameraLinkHandler::byte_array_t vals = {xbin, ybin};
    writeContinuousRegisters(addr,vals);
}


void EagleCamera::getBinning(uint8_t *xbin, uint8_t *ybin)
{
    if ( !xbin && !ybin ) return;

    CameraLinkHandler::byte_array_t addr = {0xA1, 0xA2};
    CameraLinkHandler::byte_array_t vals = readContinuousRegisters(addr);

    if ( xbin ) *xbin = vals[0];
    if ( ybin ) *ybin = vals[1];
}


void EagleCamera::setROI(const uint16_t xstart, const uint16_t ystart, const uint16_t width, const uint16_t height)
{
    uint8_t xbin, ybin;
    std::string log_str;

    if ( (xstart < 1) || (xstart > EAGLE_CAMERA_CCD_WIDTH) ) {
        log_str = "XSTART of ROI must be within 1 and " + std::to_string(EAGLE_CAMERA_CCD_WIDTH) +
                  "! (trying to set to " + std::to_string(xstart) + ")";
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
    }

    if ( (ystart < 1) || (ystart > EAGLE_CAMERA_CCD_HEIGHT) ) {
        log_str = "YSTART of ROI must be within 1 and " + std::to_string(EAGLE_CAMERA_CCD_HEIGHT) +
                  "! (trying to set to " + std::to_string(xstart) + ")";
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
    }

    getBinning(&xbin, &ybin);

    uint16_t xsize= xbin*width;
    uint16_t xend = xstart + xsize-1;
    xsize = (xend <= EAGLE_CAMERA_CCD_WIDTH) ? xsize : EAGLE_CAMERA_CCD_WIDTH-xstart+1;

    uint16_t ysize= ybin*height;
    uint16_t yend = ystart + ysize-1;
    ysize = (yend <= EAGLE_CAMERA_CCD_HEIGHT) ? ysize : EAGLE_CAMERA_CCD_HEIGHT-ystart+1;



    CameraLinkHandler::byte_array_t addr = {0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB};
    CameraLinkHandler::byte_array_t bytes(addr.size());

    std::vector<uint16_t> values = {xsize, xstart, ysize, ystart};

    size_t i = 0;
    for ( auto val: values ) {
        bytes[2*i] = (val & 0x0F00) >> 8; // MM
        bytes[2*i+1] = val & 0xFF;        // LL
    }

    writeContinuousRegisters(addr, bytes);
}


void EagleCamera::getROI(uint16_t *xstart, uint16_t *ystart, uint16_t *width, uint16_t *height)
{
    CameraLinkHandler::byte_array_t addr = {0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB};

    CameraLinkHandler::byte_array_t value = readContinuousRegisters(addr);

    if ( width ) {
        *width = ((value[0] & 0x0F) << 8) + value[1];
    }

    if ( xstart ) {
        *xstart = ((value[2] & 0x0F) << 8) + value[3];
    }

    if ( height ) {
        *width = ((value[4] & 0x0F) << 8) + value[5];
    }

    if ( ystart ) {
        *ystart = ((value[6] & 0x0F) << 8) + value[7];
    }
}


void EagleCamera::setReadoutRate(const EagleCamera::ReadoutRate rate)
{
    CameraLinkHandler::byte_array_t addr = {0xA3, 0xA4};
    CameraLinkHandler::byte_array_t vals;

    switch (rate) {
        case EagleCamera::FastRate:
            vals = {CL_READOUT_CLOCK_RATE_A3_2MHZ, CL_READOUT_CLOCK_RATE_A4_2MHZ};
            break;
        case EagleCamera::SlowRate:
            vals = {CL_READOUT_CLOCK_RATE_A3_75KHZ, CL_READOUT_CLOCK_RATE_A4_75KHZ};
            break;
        default:
            std::string log_str = "Bad value for readout rate!";
            throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM,log_str);
            break;
    }

    writeContinuousRegisters(addr, vals);
}


EagleCamera::ReadoutRate EagleCamera::getReadoutRate()
{
    CameraLinkHandler::byte_array_t addr = {0xA3, 0xA4};
    CameraLinkHandler::byte_array_t vals = readContinuousRegisters(addr);

    if ( (vals[0] == CL_READOUT_CLOCK_RATE_A3_2MHZ) && (vals[1] == CL_READOUT_CLOCK_RATE_A4_2MHZ) ) return EagleCamera::FastRate;

    if ( (vals[0] == CL_READOUT_CLOCK_RATE_A3_75KHZ) && (vals[1] == CL_READOUT_CLOCK_RATE_A4_75KHZ) ) return EagleCamera::SlowRate;

    return EagleCamera::UnknownRate;
}


void EagleCamera::setGain(const EagleCamera::PreAmpGain gain)
{
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_WRITE_VALUE;

    unsigned char ctrlReg = getCtrlRegister();

    switch (gain) {
    case EagleCamera::HighGain:
        ctrlReg &= (0xFF ^ CL_FPGA_CTRL_REG_HIGH_GAIN);
        break;
    case EagleCamera::LowGain:
        ctrlReg |= CL_FPGA_CTRL_REG_HIGH_GAIN;
        break;
    default:
        std::string log_str = "Bad value for PreAmp gain!";
        throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM,log_str);
        break;
    }

    comm[4] = ctrlReg;

    cameralink_handler.exec(comm);
}


EagleCamera::PreAmpGain EagleCamera::getGain()
{
    unsigned char ctrlReg = getCtrlRegister();

    if ( (ctrlReg & CL_FPGA_CTRL_REG_HIGH_GAIN) == 0 ) {
        return EagleCamera::HighGain;
    } else return EagleCamera::LowGain;
}


void EagleCamera::enableTEC(const bool flag)
{
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_WRITE_VALUE;

    unsigned char ctrlReg = getCtrlRegister();
    bool isEnabled = ctrlReg & CL_FPGA_CTRL_REG_ENABLE_TEC;

    if ( flag ) {
        if ( !isEnabled ) { // turn on
            ctrlReg |= CL_FPGA_CTRL_REG_ENABLE_TEC;
        }
    } else {
        if ( isEnabled ) { // turn off
            ctrlReg ^= CL_FPGA_CTRL_REG_ENABLE_TEC;
        }
    }

    comm[4] = ctrlReg;;
    cameralink_handler.exec(comm);
}


bool EagleCamera::isTECEnabled()
{
    unsigned char ctrlReg = getCtrlRegister();
    return ctrlReg & CL_FPGA_CTRL_REG_ENABLE_TEC;
}


void EagleCamera::setCCD_Temperature(const double temp)
{
    CameraLinkHandler::byte_array_t addr = {0x03, 0x04};
    CameraLinkHandler::byte_array_t value(2);

    uint16_t counts = static_cast<uint16_t>(DAC_LinearCoeffs[1]*temp + DAC_LinearCoeffs[0]);

    value[0] = ( counts & 0x0F00) >> 8;
    value[1] = counts & 0x00FF;

    writeContinuousRegisters(addr,value);
}


double EagleCamera::getCCD_Temperature()
{
    //
    // according to Reference Manual there are unusual addressing for this command (extra 0x00 after address)
    //
    CameraLinkHandler::byte_array_t addr = {0x6E, 0x6F};
    CameraLinkHandler::byte_array_t comm_addr = {0x53, 0xE0, 0x02, 0x6E, 0x00};

    CameraLinkHandler::byte_array_t value = readContinuousRegisters(addr, comm_addr);

    uint16_t counts = ((value[0] & 0x0F) << 8) + value[1];

    return ADC_LinearCoeffs[0] + ADC_LinearCoeffs[1]*counts;
}


double EagleCamera::getPCB_Temperature()
{
    //
    // according to Reference Manual there are unusual addressing for this command (extra 0x00 after address)
    //
    CameraLinkHandler::byte_array_t addr = {0x70, 0x71};
    CameraLinkHandler::byte_array_t comm_addr = {0x53, 0xE0, 0x02, 0x6E, 0x00};

    CameraLinkHandler::byte_array_t value = readContinuousRegisters(addr, comm_addr);

    uint16_t counts = ((value[0] & 0x0F) << 8) + value[1];

    return counts/16.0;
}


void EagleCamera::setShutterState(const EagleCamera::ShutterState state)
{
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_WRITE_VALUE;

    comm[3] = 0xA5;
    comm[4] = 0x00;

    switch (state) {
        case EagleCamera::ShutterClosed:
            comm[4] |= CL_SHUTTER_CLOSED;
            break;
        case EagleCamera::ShutterOpen:
            comm[4] |= CL_SHUTTER_OPEN;
            break;
        case EagleCamera::ShutterExp:
            comm[4] |= CL_SHUTTER_EXP;
            break;
        default:
            std::string log_str = "Unknown shutter state! (trying to set to " + std::to_string(state) + ")";
            throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
            break;
    }

    cameralink_handler.exec(comm);
}


EagleCamera::ShutterState EagleCamera::getShutterState()
{
    CameraLinkHandler::byte_array_t comm_addr = CL_COMMAND_SET_ADDRESS;
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_READ_VALUE;
    CameraLinkHandler::byte_array_t value(1);

    comm_addr[3] = 0xA5;

    cameralink_handler.exec(comm,value);

    switch (value[0]) {
        case CL_SHUTTER_CLOSED:
            return EagleCamera::ShutterClosed;
        case CL_SHUTTER_OPEN:
            return EagleCamera::ShutterOpen;
        case CL_SHUTTER_EXP:
            return EagleCamera::ShutterExp;
        default:
            return EagleCamera::UnknownShutterState;
    }
}


void EagleCamera::setReadoutMode(const EagleCamera::ReadoutMode mode)
{
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_WRITE_VALUE;
    comm[3] = 0xF7;
    comm[4] = 0x00;

    switch (mode) {
        case EagleCamera::NormalReadoutMode:
            comm[4] |= CL_READOUT_MODE_NORMAL;
            break;
        case EagleCamera::TestReadoutMode:
            comm[4] |= CL_READOUT_MODE_TEST;
            break;
        default:
            std::string log_str = "Unknown readout mode! (trying to set to " + std::to_string(mode) + ")";
            throw EagleCamera_Exception(EagleCamera::ERROR_BAD_PARAM, log_str);
            break;
    }

    cameralink_handler.exec(comm);
}


EagleCamera::ReadoutMode EagleCamera::getReadoutMode()
{
    CameraLinkHandler::byte_array_t addr_comm = CL_COMMAND_SET_ADDRESS;
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_READ_VALUE;
    CameraLinkHandler::byte_array_t value(1);

    addr_comm[3] = 0xF7;

    cameralink_handler.exec(addr_comm);
    cameralink_handler.exec(comm, value);

    switch (value[0]) {
        case CL_READOUT_MODE_NORMAL:
            return EagleCamera::NormalReadoutMode;
        case CL_READOUT_MODE_TEST:
            return EagleCamera::TestReadoutMode;
        default:
            return EagleCamera::UnknownReadoutMode;
    }
}


void EagleCamera::logToFile(const EagleCamera::LogIdent ident, const std::string &log_str, const int indent_tabs)
{
    if ( !cameraLog ) return;

    if ( logLevel == EagleCamera::LOG_LEVEL_QUIET ) return;

    std::string tab;
    if ( indent_tabs > 0 ) tab.resize(indent_tabs*EAGLE_CAMERA_DEFAULT_LOG_TAB);

    std::string str = "[" + time_stamp() + "]";

    switch ( ident ) {
    case EagleCamera::LOG_IDENT_BLANK:
        *cameraLog  << tab << log_str << std::endl << std::flush;
        break;
    case EagleCamera::LOG_IDENT_CAMERA_INFO:
        str += "[CAMERA INFO] ";
        break;
    case EagleCamera::LOG_IDENT_CAMERA_ERROR:
        str += "[CAMERA ERROR] ";
        break;
    case EagleCamera::LOG_IDENT_XCLIB_INFO:
        str += "[XCLIB INFO] ";
        break;
    case EagleCamera::LOG_IDENT_XCLIB_ERROR:
        str += "[XCLIB ERROR] ";
        break;
    default:
        break;
    }


    *cameraLog << str << tab << log_str << std::endl << std::flush;
}


void EagleCamera::logToFile(const XCLIB_Exception &ex, const int indent_tabs)
{
    std::string log_str = ex.what();
    log_str += " [XCLIB ERROR CODE: " + std::to_string(ex.getError()) + "]";

    logToFile(EagleCamera::LOG_IDENT_XCLIB_ERROR, log_str, indent_tabs);
}


uint16_t EagleCamera::serialNumber() const
{
    return _serialNumber;
}


std::string EagleCamera::buildDate() const
{
    return _buildDate;
}


std::string EagleCamera::buildCode() const
{
    return _buildCode;
}


void EagleCamera::calibrationData(uint16_t *ADC_0, uint16_t *ADC_40, uint16_t *DAC_0, uint16_t *DAC_40)
{
    if ( ADC_0 ) *ADC_0 = ADC_Calibration[0];
    if ( ADC_40 ) *ADC_40 = ADC_Calibration[1];

    if ( DAC_0 ) *DAC_0 = DAC_Calibration[0];
    if ( DAC_40 ) *DAC_40 = DAC_Calibration[1];
}


std::string EagleCamera::microVersion() const
{
    return _microVersion;
}


std::string EagleCamera::FPGA_Version() const
{
    return _FPGA_Version;
}

                        /*  PROTECTED METHODS   */

unsigned char EagleCamera::getCtrlRegister()
{
    CameraLinkHandler::byte_array_t addr = CL_COMMAND_SET_ADDRESS;
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_READ_VALUE;
    CameraLinkHandler::byte_array_t value(1);

    cameralink_handler.exec(addr);
    cameralink_handler.exec(comm,value);

    return value[1];
}


unsigned char EagleCamera::getTriggerRegister()
{
    CameraLinkHandler::byte_array_t val(1);

    CameraLinkHandler::byte_array_t addr = CL_COMMAND_SET_ADDRESS;
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_READ_VALUE;

    addr[3] = 0xD4;
    cameralink_handler.exec(comm,val);

    return val[0];
}


void EagleCamera::getManufacturerData()
{
    cameralink_handler.setSystemState(true,true,false,true); // set FPGA_EPROM_COMMS bit

    CameraLinkHandler::byte_array_t comm = CL_COMMAND_GET_MANUFACTURER_DATA_1;
    cameralink_handler.exec(comm);

    comm = CL_COMMAND_GET_MANUFACTURER_DATA_2;
    CameraLinkHandler::byte_array_t value(18);
    cameralink_handler.exec(comm,value);

    _serialNumber = (value[0] << 8) + value[1];

    _buildDate = std::to_string(value[2]) + "/" + std::to_string(value[3]) + "/" + std::to_string(value[4]); // DD/MM/YY

    char buff[6];
    _buildCode = (char*)memcpy(buff, value.data() + 5, 5);

    ADC_Calibration[0] = (value[10] << 8) + value[11];
    ADC_Calibration[1] = (value[12] << 8) + value[13];

    // y = ADC_LinearCoeffs[0] + ADC_LinearCoeffs[1]*x
    ADC_LinearCoeffs[1] = (ADC_Calibration[1] - ADC_Calibration[0])/(ADC_CALIBRATION_POINT_2 - ADC_CALIBRATION_POINT_1);
    ADC_LinearCoeffs[0] = ADC_Calibration[0] - ADC_LinearCoeffs[1]*ADC_CALIBRATION_POINT_1;

    DAC_Calibration[0] = (value[14] << 8) + value[15];
    DAC_Calibration[1] = (value[16] << 8) + value[17];

    // y = DAC_LinearCoeffs[0] + DAC_LinearCoeffs[1]*x
    DAC_LinearCoeffs[1] = (DAC_Calibration[1] - DAC_Calibration[0])/(DAC_CALIBRATION_POINT_2 - DAC_CALIBRATION_POINT_1);
    DAC_LinearCoeffs[0] = DAC_Calibration[0] - DAC_LinearCoeffs[1]*DAC_CALIBRATION_POINT_1;

    cameralink_handler.setSystemState(true,true,false,false); // clear FPGA_EPROM_COMMS bit
}


void EagleCamera::getVersions()
{
    CameraLinkHandler::byte_array_t comm = {0x56};
    CameraLinkHandler::byte_array_t value(2);

    cameralink_handler.exec(comm,value); // get Micro version

    _microVersion = std::to_string(value[0]) + "." + std::to_string(value[1]);

    CameraLinkHandler::byte_array_t addr = { 0x7E,  0x7F };

    value = readContinuousRegisters(addr); // get FPGA version

    _FPGA_Version = std::to_string(value[0]) + "." + std::to_string(value[1]);
}

CameraLinkHandler::byte_array_t EagleCamera::readContinuousRegisters(const CameraLinkHandler::byte_array_t &addr,
                                                                     CameraLinkHandler::byte_array_t &addr_comm)
{
    CameraLinkHandler::byte_array_t res(addr.size());
//    CameraLinkHandler::byte_array_t addr_comm = CL_COMMAND_SET_ADDRESS;
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_READ_VALUE;
    CameraLinkHandler::byte_array_t value(1);

    size_t i = 0;
    for (auto address: addr ) {
        addr_comm[3] = address;
        cameralink_handler.exec(addr_comm);
        cameralink_handler.exec(comm,value);
        res[i++] = value[0];
    }

    return res;
}


void EagleCamera::writeContinuousRegisters(const CameraLinkHandler::byte_array_t &addr, const CameraLinkHandler::byte_array_t &values)
{
    CameraLinkHandler::byte_array_t comm = CL_COMMAND_WRITE_VALUE;

    size_t i = 0;
    for ( auto address: addr ) {
        comm[3] = address;
        comm[4] = values[i++];
        cameralink_handler.exec(comm);
    }
}


void EagleCamera::logXCLibCall(const std::string &log_str)
{
    logToFile(EagleCamera::LOG_IDENT_XCLIB_INFO, log_str);
}


