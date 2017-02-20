#include "cameralink_handler.h"
#include "eagle_camera.h"

#include <algorithm>
#include <cstdlib>
#include <sstream>

// implemented in eagle_camera.cpp
extern int XCLIB_API_CALL(int err_code, const char *context);
extern int XCLIB_API_CALL(int err_code, const std::string& context);
extern std::string pointer_to_str(void* ptr);

static char nill = '\0';

std::map<int,int> CameraLinkHandler::usedUnitmaps = std::map<int,int>();

CameraLinkHandler::CameraLinkHandler(const int unitmap):
    currentUnitmap(0), currentSpeed(CL_DEFAULT_BAUD_RATE),
    lastXCLibError(0), lastControllerError(CL_ETX),
    ACK_Bit(CL_DEFAULT_ACK_ENABLED), CK_SUM_Bit(CL_DEFAULT_CK_SUM_ENABLED),
    FPGAinRST_Bit(true), FPGA_EPROM_Bit(false),
//    rx_buff(std::unique_ptr<char[]>(&nill)), tx_buff(std::unique_ptr<char[]>(&nill)),
//    lastLogMessageUniqPtr(std::unique_ptr<char[]>(new char[CAMERALINK_HANDLER_LOG_MSG_LEN])),
    logFunc(nullptr)
{

//    setUnitmap(unitmap);
//    if ( lastXCLibError < 0 ) return;

//    lastLogMessage = lastLogMessageUniqPtr.get();

//    getMode();
//    if ( lastXCLibError < 0 ) return;

//    setMode(ACK_Bit, CK_SUM_Bit);
}


CameraLinkHandler::~CameraLinkHandler()
{
    --usedUnitmaps[currentUnitmap];
}


void CameraLinkHandler::setUnitmap(const int unitmap, const bool force)
{
    lastXCLibError = 0;
    lastControllerError = CL_ETX;

//    if ( unitmap <= 0 ) {
//        lastXCLibError = PXERBADPARM;
//        return;
//    }

    currentUnitmap = unitmap;
    ++usedUnitmaps[unitmap];

    // configure port only once per each unitmap
    if ( (usedUnitmaps[unitmap] == 1) || force ) lastXCLibError = config(CL_DEFAULT_BAUD_RATE, CL_DEFAULT_DATA_BITS, CL_DEFAULT_STOP_BIT);
    if ( lastXCLibError < 0 ) return;

//    reset();
    getMode();
    setMode(ACK_Bit, CK_SUM_Bit);
}


int CameraLinkHandler::getUnitMap() const
{
    return currentUnitmap;
}


int CameraLinkHandler::getLastXCLibError() const
{
    return lastXCLibError;
}


char CameraLinkHandler::getLastControllerError() const
{
    return lastControllerError;
}


bool CameraLinkHandler::isValid() const
{
    bool ok = ( (lastXCLibError >= 0) && (lastControllerError == CL_ETX) ) ? true : false;

    return ok;
}


const char* CameraLinkHandler::getLastCameraMessage() const
{
    return rx_buff.get();
}


const char* CameraLinkHandler::getLastHostMessage() const
{
    return tx_buff.get();
}


//
//  must be invoked AFTER pxd_PIXCIopen!!!
//
int CameraLinkHandler::config(const long speed, const int bits, const int stop_bits)
{
    lastControllerError = CL_ETX;
    lastXCLibError = 0;

    std::string log_str = "pxd_serialConfigure(" + std::to_string(currentUnitmap) + ", 0, " + std::to_string(speed) +
                          ", " + std::to_string(bits) + ", 0, " + std::to_string(stop_bits) + ", 0, 0, 0)";

    XCLIB_API_CALL( lastXCLibError = pxd_serialConfigure(currentUnitmap,0,speed,bits,0,stop_bits,0,0,0), log_str);

    return lastXCLibError;
}


int CameraLinkHandler::setMode(const bool ack_enabled, const bool ck_sum_enabled)
{
    if ( ack_enabled ) currentSystemState |= CL_SYSTEM_STATE_ACK;
    if ( ck_sum_enabled ) currentSystemState |= CL_SYSTEM_STATE_CK_SUM;

    byte_array_t comm = {0x4F, currentSystemState};
    byte_array_t ans;

    exec(comm,ans,CL_DEFAULT_TIMEOUT);

    return lastXCLibError;
}


int CameraLinkHandler::getMode()
{
    byte_array_t comm = {0x49}; // get system status

    byte_array_t status(1);

    exec(comm, status, CL_DEFAULT_TIMEOUT);

    if ( status[0] & CL_SYSTEM_STATE_CK_SUM ) CK_SUM_Bit = true; else CK_SUM_Bit = false;
    if ( status[0] & CL_SYSTEM_STATE_ACK ) ACK_Bit = true; else ACK_Bit = false;
    if ( status[0] & CL_SYSTEM_STATE_FPGA_RST_HOLD ) FPGAinRST_Bit = false; FPGAinRST_Bit = true; // here is opposite case!!!
    if ( status[0] & CL_SYSTEM_STATE_FPGA_EPROM_COMMS ) FPGA_EPROM_Bit = true; FPGA_EPROM_Bit = false;

    currentSystemState = status[0];

    return lastXCLibError;
}


int CameraLinkHandler::read(byte_array_t &msg, const long timeout)
{
    lastXCLibError = 0;
    lastControllerError = CL_ETX;

    std::string log_str_timeout = "timeout occured while reading operation";

//    std::string log_str = "pxd_serialRead(" + std::to_string(currentUnitmap) + ", 0, NULL, 0)";

    std::string log_str = formatLog("pxd_serialRead",NULL,0);

    // first, check for special invoking ...

    if ( timeout < 0 ) {
        lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0);
//        if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));
//        return XCLIB_API_CALL( lastXCLibError , log_str + " -> " + std::to_string(lastXCLibError));
        if ( logFunc ) logFunc(formatLog(log_str, lastXCLibError));
        return XCLIB_API_CALL( lastXCLibError , log_str);
    }

    // compute expected full length of camera UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;

    // nothing to read, return number of bytes in Rx-buffer
    if ( !UART_len ) {
        lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0);
//        if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));
//        return XCLIB_API_CALL( lastXCLibError , log_str + " -> " + std::to_string(lastXCLibError));
        if ( logFunc ) logFunc(formatLog(log_str, lastXCLibError));
        return XCLIB_API_CALL( lastXCLibError , log_str);
    }


    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    rx_buff = std::unique_ptr<char[]>( new char[UART_len+1]); // +1 to insert '\0' character
    char* buff_ptr = rx_buff.get();
    buff_ptr[UART_len] = '\0';

//    std::cout << "READ: reading " << UART_len << "-bytes ...\n";

    auto start = std::chrono::system_clock::now();

    XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str ); // how many bytes are available in Rx-buffer now
    if ( logFunc ) logFunc(formatLog(log_str, lastXCLibError));
//    if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));

    while ( lastXCLibError < UART_len ) { // wait for UART_len number of bytes in Rx-buffer
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() >= timeout_counts ) {
            throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));

        XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str ); // how many bytes are available in Rx-buffer now
        if ( logFunc ) logFunc(formatLog(log_str, lastXCLibError));
//        if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));
    }

//    log_str = "pxd_serialRead(" + std::to_string(currentUnitmap) + ", 0, " + pointer_to_str(buff_ptr) + ", " +
//              std::to_string(UART_len) + ")";

    log_str = formatLog("pxd_serialRead",buff_ptr,UART_len);

    XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, buff_ptr, UART_len), log_str );
    if ( logFunc ) {
        logFunc( formatLog("pxd_serialRead",buff_ptr,UART_len,lastXCLibError) );
//        logFunc(log_str + " -> " + std::to_string(lastXCLibError));
    }

    if ( msg.size() ) {
        for ( size_t i = 0; i < msg.size(); ++i ) msg[i] = buff_ptr[i];
    }
//    std::cout << "READ: UART [";
//    for ( size_t i = 0; i < UART_len; ++i ) printf("%#x ",buff_ptr[i]);
//    std::cout << "]\n";

    if ( ACK_Bit ) {
        lastControllerError = buff_ptr[msg.size()];
        if ( lastControllerError != CL_ETX ) {
            log_str = "Last serial read operation return '" + std::to_string(lastControllerError) + "' error code!";
            throw EagleCamera_Exception(EagleCamera::ERROR_CONTROLLER_ANSWER, log_str, lastControllerError);
        }
    }

    return lastXCLibError;
}


int CameraLinkHandler::write(const byte_array_t &msg, const long timeout)
{
    lastControllerError = CL_ETX;
    lastXCLibError = 0;

    std::string log_str_timeout = "timeout occured while writing operation";

//    std::string log_str = "pxd_serialWrite(" + std::to_string(currentUnitmap) + ", 0, NULL, 0)";
    std::string log_str = formatLog("pxd_serialWrite",NULL,0);

    // special cases

    // how many bytes are available in Tx-buffer now;
    if ( timeout < 0 ) {
        lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0);
//        return XCLIB_API_CALL( lastXCLibError, log_str + " -> " + std::to_string(lastXCLibError));
        return XCLIB_API_CALL( lastXCLibError, formatLog(log_str, lastXCLibError) );
    }

    // how many bytes are available in Tx-buffer now;
    if ( msg.size() == 0 ) {
        lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0);
//        return XCLIB_API_CALL( lastXCLibError, log_str + " -> " + std::to_string(lastXCLibError));
        return XCLIB_API_CALL( lastXCLibError, formatLog(log_str, lastXCLibError) );
    }

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    // compute expected full length of host UART mesage
    size_t UART_len = msg.size() + 1; // add mandatory for host message ETX byte
    if ( CK_SUM_Bit ) ++UART_len;

    tx_buff = std::unique_ptr<char[]>(new char[UART_len+1]); // +1 to insert '\0' character
    char* buff_ptr = tx_buff.get();
    buff_ptr[UART_len] = '\0';

    std::copy_n(msg.data(),msg.size(),buff_ptr);
    buff_ptr[msg.size()] = CL_ETX; // add ETX

    if ( CK_SUM_Bit) { // compute check sum (XOR for message bytes including ETX field)
        buff_ptr[UART_len-1] = msg[0];
        if ( CK_SUM_Bit ) {
            for ( size_t i = 1; i < msg.size(); ++i ) buff_ptr[UART_len-1] ^= msg[i];
        }
        buff_ptr[UART_len-1] ^= CL_ETX;
    }

    lastXCLibError = 0;

    auto start = std::chrono::system_clock::now();

//    std::cout << "WRITE UART: [";
//    for (int i = 0; i < UART_len; ++i) printf("%#x ", buff_ptr[i]);
//    std::cout << "]\n";

    // how many bytes are available in Tx-buffer now
    XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);
//    if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));
    if ( logFunc ) logFunc( formatLog(log_str, lastXCLibError) );

    while ( lastXCLibError < UART_len ) { // wait for free space in Tx-buffer
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() >= timeout_counts ) {
            throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));

        // how many bytes are available in Tx-buffer now
        XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);
//        if ( logFunc ) logFunc(log_str + " -> " + std::to_string(lastXCLibError));
        if ( logFunc ) logFunc( formatLog(log_str, lastXCLibError) );
    }

    // it should return immediately
//    log_str = "pxd_serialWrite(" + std::to_string(currentUnitmap) + ", 0, " + pointer_to_str(buff_ptr) +
//              ", " + std::to_string(UART_len) + ")";
    log_str = formatLog("pxd_serialWrite",buff_ptr,UART_len);

    XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, buff_ptr, UART_len), log_str);
    if ( logFunc ) {
//        logFunc(log_str + " -> " + std::to_string(lastXCLibError));
        log_str = formatLog("pxd_serialWrite",buff_ptr,UART_len, lastXCLibError);
        logFunc( log_str);
    }

    return lastXCLibError;
}


void CameraLinkHandler::exec(const byte_array_t &command, byte_array_t &ans, const long timeout, const long sleep)
{
//    std::cout << "EXEC: write \n";
    write(command, timeout);
//    std::cout << "EXEC: write OK\n";

    if ( sleep > 0 ) std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

//    std::cout << "EXEC: read\n";
    read(ans,timeout);
//    std::cout << "EXEC: read OK\n";
}


void CameraLinkHandler::exec(const byte_array_t &command, const long timeout, const long sleep)
{
    byte_array_t ans;

    exec(command, ans, timeout, sleep);
}


int CameraLinkHandler::reset(const long timeout)
{
    std::string log_str_timeout = "timeout occured while reset micro controller!";

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();

    lastXCLibError = 0;
    lastControllerError = CL_ETX;

    byte_array_t cam_msg;

    byte_array_t cmd  = {0x55, 0x99, 0x66, 0x11}; // reset micro

    write(cmd); // there is no camera response for this command!!!

    byte_array_t msg = {0x4F, 0x54};  // set 'FPGA in RST' bit to 0 (reset). also set ACK and CK_SUM bits
//    byte_array_t msg = {0x4F, 0x51};  // set 'FPGA in RST' bit to 0 (reset). also set ACK and CK_SUM bits

    ACK_Bit = true;
    CK_SUM_Bit = true;

    auto start = std::chrono::system_clock::now();

    while ( lastXCLibError < 2 ) { // poll camera with 'msg' and wait for camera response bytes in Rx-buffer
                                   // (according to Raptor Eagle-V 4240 Instruction manual) ...
        write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        lastXCLibError = read(cam_msg, -1);

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;

        if ( std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() >= timeout_counts ) {
            throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
        }
    }
    lastXCLibError = read(cam_msg); // read response

    getMode();

    return lastXCLibError;
}


int CameraLinkHandler::setSystemState(const bool ck_sum_bit, const bool ack_bit,
                                      const bool fpga_in_reset, const bool fpga_eprom)
{
    unsigned char mode = 0;

    if ( !fpga_in_reset ) mode |= CL_SYSTEM_STATE_FPGA_RST_HOLD; // 0 - reset, 1 - no reset
    if ( ck_sum_bit ) mode |= CL_SYSTEM_STATE_CK_SUM;
    if ( ack_bit ) mode |= CL_SYSTEM_STATE_ACK;
    if ( fpga_eprom ) mode |= CL_SYSTEM_STATE_FPGA_EPROM_COMMS;

    byte_array_t comm = {0x4F, mode};
    byte_array_t ans; // expect as answer only ACK + Chk_Sum

    exec(comm, ans, CL_DEFAULT_TIMEOUT);

    return lastXCLibError;
}


void CameraLinkHandler::setLogFunc(const log_func_t &func)
{
    logFunc = func;
}


std::string CameraLinkHandler::formatLog(const std::string &log_str, const int err_code)
{
    return log_str + " -> " + std::to_string(err_code);
}

std::string CameraLinkHandler::formatLog(const char *log_str, const int err_code)
{
    return formatLog(std::string(log_str), err_code);
}


std::string CameraLinkHandler::formatLog(const char *func_name, const char *UART_msg, const size_t UART_len)
{
    std::ostringstream st;

    st << func_name << "(" << currentUnitmap << ", 0, " << pointer_to_str((void*)UART_msg) << ", " << UART_len << ")";

    return st.str();
}

std::string CameraLinkHandler::formatLog(const char *func_name, const char *UART_msg,
                                         const size_t UART_len, const int err_code)
{
    std::ostringstream st;
    st.str(formatLog(formatLog(func_name, UART_msg, UART_len), err_code));

    if ( !UART_len ) return st.str();

    st << " [";
    size_t i;
    for (i = 0; i < (UART_len-1); ++i) st << std::nouppercase << "0x" << std::hex << std::uppercase << UART_msg[i] << ", ";
    st << std::nouppercase << "0x" << std::hex << std::uppercase << UART_msg[i];
    st << "]";

    return st.str();
}
