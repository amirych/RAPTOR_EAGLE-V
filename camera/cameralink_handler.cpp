#include "cameralink_handler.h"
#include "eagle_camera.h"

#include <algorithm>
#include <cstdlib>

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
    rx_buff(std::unique_ptr<char[]>(&nill)), tx_buff(std::unique_ptr<char[]>(&nill)),
    lastLogMessageUniqPtr(std::unique_ptr<char[]>(new char[CAMERALINK_HANDLER_LOG_MSG_LEN])),
    logFunc(nullptr)
{

    setUnitmap(unitmap);
    if ( lastXCLibError < 0 ) return;

    lastLogMessage = lastLogMessageUniqPtr.get();

    getMode();
    if ( lastXCLibError < 0 ) return;

    setMode(ACK_Bit, CK_SUM_Bit);
}


CameraLinkHandler::~CameraLinkHandler()
{
    --usedUnitmaps[currentUnitmap];
}


void CameraLinkHandler::setUnitmap(const int unitmap, const bool force)
{
    lastXCLibError = 0;
    lastControllerError = CL_ETX;

    if ( unitmap <= 0 ) {
        lastXCLibError = PXERBADPARM;
        return;
    }

    currentUnitmap = unitmap;
    ++usedUnitmaps[unitmap];

    // configure port only once per each unitmap
    if ( (usedUnitmaps[unitmap] == 1) || force ) lastXCLibError = config(CL_DEFAULT_BAUD_RATE, CL_DEFAULT_DATA_BITS, CL_DEFAULT_STOP_BIT);
    if ( lastXCLibError < 0 ) return;

    getMode();
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

    std::string log_str = "pxd_serialConfigure(" + std::to_string(currentUnitmap) + ", 0, " + std::to_string(speed) +
                          ", " + std::to_string(bits) + ", 0, " + std::to_string(stop_bits) + ", 0, 0, 0)";

    XCLIB_API_CALL( lastXCLibError = pxd_serialConfigure(currentUnitmap,0,speed,bits,0,stop_bits,0,0,0), log_str);

    return lastXCLibError;
}


int CameraLinkHandler::setMode(const bool ack_enabled, const bool ck_sum_enabled)
{
    if ( ack_enabled ) currentSystemState |= CL_SYSTEM_STATE_ACK;
    if ( ck_sum_enabled ) currentSystemState |= CL_SYSTEM_STATE_CK_SUM;

    byte_array_t comm = {currentSystemState};
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

    std::string log_str = "pxd_serialRead(" + std::to_string(currentUnitmap) + ", 0, NULL, 0)";

    // first, check for special invoking ...

    if ( logFunc ) logFunc(log_str);
    if ( timeout < 0 ) return XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str );

    // compute expected full length of camera UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;

    // nothing to read, return number of bytes in Rx-buffer
    if ( logFunc ) logFunc(log_str);
    if ( !UART_len ) return XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str );

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    rx_buff = std::unique_ptr<char[]>( new char[UART_len+1]); // +1 to insert '\0' character
    char* buff_ptr = rx_buff.get();
    buff_ptr[UART_len] = '\0';

    size_t n_received = 0;

    auto start = std::chrono::system_clock::now();

    if ( logFunc ) logFunc(log_str);
    XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str ); // how many bytes are available in Rx-buffer now

    while (lastXCLibError < msg.size()); { // wait for number of bytes at least equal to DATA field length
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
        if ( logFunc ) logFunc(log_str);
        XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, NULL, 0), log_str ); // how many bytes are available in Rx-buffer now
    }

    while ( n_received < UART_len ) {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
        }

        log_str = "pxd_serialRead(" + std::to_string(currentUnitmap) + ", 0, " + pointer_to_str(buff_ptr + n_received) + ", " +
                  std::to_string(UART_len - n_received) + ")";

        if ( logFunc ) logFunc(log_str);
        XCLIB_API_CALL( lastXCLibError = pxd_serialRead(currentUnitmap, 0, buff_ptr + n_received, UART_len - n_received), log_str );
        if ( lastXCLibError == 0 ) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
            continue; // wait for data
        }

        n_received += lastXCLibError;

        if ( n_received > msg.size() ) { // all DATA field bytes were read, look for ETX and Chk_Sum fields
                                         // Note: if n_received >= UART_len then Chk_Sum field was also read (do not check it)
            if ( ACK_Bit ) lastControllerError = buff_ptr[msg.size()];
        }
    }

    size_t k = n_received > msg.size() ? msg.size() : n_received;

    if ( k ) {
        for ( size_t i = 0; i < k; ++i ) msg[i] = buff_ptr[i]; // copy received DATA field (it may be not full!)
    } else {
        msg.clear(); // empty DATA field
    }

    if ( ACK_Bit && (lastControllerError != CL_ETX) ) {
        log_str = "Last serial read operation return '" + std::to_string(lastControllerError) + "' error code!";
        throw EagleCamera_Exception(EagleCamera::ERROR_CONTROLLER_ANSWER, log_str, lastControllerError);
    }

    return lastXCLibError;
}


int CameraLinkHandler::write(const byte_array_t &msg, const long timeout)
{
    lastControllerError = CL_ETX;
    lastXCLibError = 0;

    std::string log_str_timeout = "timeout occured while writing operation";

    std::string log_str = "pxd_serialWrite(" + std::to_string(currentUnitmap) + ", 0, NULL, 0)";

    // special cases

    // how many bytes are available in Tx-buffer now;
    if ( logFunc ) logFunc(log_str);
    if ( timeout < 0 ) return XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);

    // how many bytes are available in Tx-buffer now;
    if ( logFunc ) logFunc(log_str);
    if ( msg.size() == 0 ) return XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);


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

    size_t n_written = 0;

    lastXCLibError = 0;

    auto start = std::chrono::system_clock::now();

    while ( n_written < UART_len ) {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            return lastXCLibError = PXERTIMEOUT;
        }

        // how many bytes are available in Tx-buffer now
        if ( logFunc ) logFunc(log_str);
        XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);

        while ( !lastXCLibError ); { // wait for at least 1 byte in Tx-buffer
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end-start;
            if ( diff.count() >= timeout_counts ) {
                throw XCLIB_Exception(lastXCLibError = PXERTIMEOUT, log_str_timeout);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));

            // how many bytes are available in Tx-buffer now
            if ( logFunc ) logFunc(log_str);
            XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, NULL, 0), log_str);
        }

        // it should return immediately
        log_str = "pxd_serialWrite(" + std::to_string(currentUnitmap) + ", " + pointer_to_str(buff_ptr + n_written) +
                  ", " + std::to_string(lastXCLibError);
        if ( logFunc ) logFunc(log_str);
        XCLIB_API_CALL( lastXCLibError = pxd_serialWrite(currentUnitmap, 0, buff_ptr + n_written, lastXCLibError), log_str);

        n_written += lastXCLibError;
    }

    return lastXCLibError;
}


void CameraLinkHandler::exec(const byte_array_t &command, byte_array_t &ans, const long timeout, const long sleep)
{
    write(command, timeout);

    if ( sleep > 0 ) std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    read(ans,timeout);
}


void CameraLinkHandler::exec(const byte_array_t &command, const long timeout, const long sleep)
{
    byte_array_t ans;

    exec(command, ans, timeout, sleep);
}


int CameraLinkHandler::reset()
{
    lastXCLibError = 0;
    lastControllerError = CL_ETX;

    byte_array_t cam_msg;

    byte_array_t cmd  = {0x55, 0x99, 0x66, 0x11}; // reset FPGA

    write(cmd); // there is no camera response for this command!!!

    byte_array_t msg = {0x4F, 0x51};  // set 'FPGA in RST' bit to 0 (reset). also set ACK and CK_SUM bits
    write(msg);

    ACK_Bit = true;
    CK_SUM_Bit = true;

    while ( lastXCLibError < 2 ) { // poll camera with 'msg' and wait for camera response bytes in Rx-buffer
                                   // (according to Raptor Eagle-V 4240 Instruction manual) ...
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        lastXCLibError = read(cam_msg, -1);

        write(msg);
    }
    lastXCLibError = read(cam_msg); // read response

    return lastXCLibError;
}


int CameraLinkHandler::setSystemState(const bool ck_sum_bit, const bool ack_bit,
                                      const bool fpga_in_reset, const bool fpga_eprom)
{
    unsigned char mode = 0;

    if ( ck_sum_bit ) mode |= CL_SYSTEM_STATE_CK_SUM;
    if ( ack_bit ) mode |= CL_SYSTEM_STATE_ACK;
    if ( fpga_in_reset ) mode &= 0xFD; // 0 - reset, 1 - no reset
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
