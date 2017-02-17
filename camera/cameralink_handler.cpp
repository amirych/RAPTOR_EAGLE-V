#include "cameralink_handler.h"

#include <memory>
#include <algorithm>

std::map<int,int> CameraLinkHandler::usedUnitmaps = std::map<int,int>();

CameraLinkHandler::CameraLinkHandler(const int unitmap):
    currentUnitmap(unitmap), currentSpeed(CL_DEFAULT_BAUD_RATE),
    lastEPIXerror(0), lastETXerror(CL_ETX),
    ACK_Bit(CL_DEFAULT_ACK_ENABLED), CK_SUM_Bit(CL_DEFAULT_CK_SUM_ENABLED),
    FPGAinRST_Bit(true), FPGA_EPROM_Bit(false),
    lastCameraMessage(nullptr), lastHostMessage(nullptr)
{
    ++usedUnitmaps[unitmap];

    // configure port only once per each unitmap
    if ( usedUnitmaps[unitmap] == 1) lastEPIXerror = config(CL_DEFAULT_BAUD_RATE, CL_DEFAULT_DATA_BITS, CL_DEFAULT_STOP_BIT);
    if ( lastEPIXerror < 0 ) return;

    getMode();
    if ( lastEPIXerror < 0 ) return;

    setMode(ACK_Bit, CK_SUM_Bit);
}


CameraLinkHandler::~CameraLinkHandler()
{
    --usedUnitmaps[currentUnitmap];
}


void CameraLinkHandler::setUnitmap(const int unitmap)
{
    if ( unitmap <= 0 ) {
        lastEPIXerror = PXERBADPARM;
        return;
    }

    currentUnitmap = unitmap;
    ++usedUnitmaps[unitmap];

    // configure port only once per each unitmap
    if ( usedUnitmaps[unitmap] == 1) lastEPIXerror = config(CL_DEFAULT_BAUD_RATE, CL_DEFAULT_DATA_BITS, CL_DEFAULT_STOP_BIT);
    if ( lastEPIXerror < 0 ) return;

    getMode();
}


int CameraLinkHandler::getUnitMap() const
{
    return currentUnitmap;
}


int CameraLinkHandler::getLastEPIXError() const
{
    return lastEPIXerror;
}


int CameraLinkHandler::getLastETXError() const
{
    return lastETXerror;
}


bool CameraLinkHandler::isValid() const
{
    bool ok = ( (lastEPIXerror == 0) && (lastETXerror == CL_ETX) ) ? true : false;

    return ok;
}


const char* CameraLinkHandler::getLastCameraMessage() const
{
    return lastCameraMessage;
}


const char* CameraLinkHandler::getLastHostMessage() const
{
    return lastHostMessage;
}


//
//  must be invoked AFTER pxd_PIXCIopen!!!
//
int CameraLinkHandler::config(const long speed, const int bits, const int stop_bits)
{
    return lastEPIXerror = pxd_serialConfigure(currentUnitmap,0,speed,bits,0,stop_bits,0,0,0);
}


int CameraLinkHandler::setMode(const bool ack_enabled, const bool ck_sum_enabled)
{
    if ( ack_enabled ) currentSystemState |= CL_SYSTEM_STATE_ACK;
    if ( ck_sum_enabled ) currentSystemState |= CL_SYSTEM_STATE_CK_SUM;

    byte_array_t comm = {currentSystemState};
    byte_array_t ans;

    exec(comm,ans,CL_DEFAULT_TIMEOUT);

    return lastEPIXerror;
}


int CameraLinkHandler::getMode()
{
    byte_array_t comm = {0x49}; // get system status

    byte_array_t status(1);

    bool ok = exec(comm, status, CL_DEFAULT_TIMEOUT);

    if ( ok ) {
        if ( status[0] & CL_SYSTEM_STATE_CK_SUM ) CK_SUM_Bit = true; else CK_SUM_Bit = false;
        if ( status[0] & CL_SYSTEM_STATE_ACK ) ACK_Bit = true; else ACK_Bit = false;
        if ( status[0] & CL_SYSTEM_STATE_FPGA_RST_HOLD ) FPGAinRST_Bit = false; FPGAinRST_Bit = true; // here is opposite case!!!
        if ( status[0] & CL_SYSTEM_STATE_FPGA_EPROM_COMMS ) FPGA_EPROM_Bit = true; FPGA_EPROM_Bit = false;

        currentSystemState = status[0];
    }

    return lastEPIXerror;
}


int CameraLinkHandler::read(byte_array_t &msg, const long timeout)
{
    lastEPIXerror = 0;
    lastETXerror = CL_ETX;

    // first, check for special invoking ...

    if ( timeout < 0 ) return pxd_serialRead(currentUnitmap, 0, NULL, 0);

    // compute expected full length of camera UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;

    // nothing to read, return number of bytes in Rx-buffer
    if ( !UART_len ) return pxd_serialRead(currentUnitmap, 0, NULL, 0);

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    std::unique_ptr<char[]> buff = std::unique_ptr<char[]>( new char[UART_len]);
    char* buff_ptr = buff.get();

    size_t n_received = 0;
    int n_read;

    auto start = std::chrono::system_clock::now();

    n_read = pxd_serialRead(currentUnitmap, 0, NULL, 0); // how many bytes are available in Rx-buffer now

    while (n_read < msg.size()); { // wait for number of bytes at least equal to DATA field length
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            return lastEPIXerror = PXERTIMEOUT;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
        n_read = pxd_serialRead(currentUnitmap, 0, NULL, 0); // how many bytes are available in Rx-buffer now
    }

    while ( n_received < UART_len ) {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            lastEPIXerror = PXERTIMEOUT;
            break;
        }

        n_read = pxd_serialRead(currentUnitmap, 0, buff_ptr + n_received, UART_len - n_received);
        if ( n_read < 0 ) return lastEPIXerror = n_read;
        if ( n_read == 0 ) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
            continue; // wait for data
        }

        n_received += n_read;

        if ( n_received > msg.size() ) { // all DATA field bytes were read, look for ETX and Chk_Sum fields
                                         // Note: if n_received >= UART_len then Chk_Sum field was also read (do not check it)
            if ( ACK_Bit ) lastETXerror = buff_ptr[msg.size()];
        }
    }

    size_t k = n_received > msg.size() ? msg.size() : n_received;

    if ( k ) {
        for ( size_t i = 0; i < k; ++i ) msg[i] = buff_ptr[i]; // copy received DATA field (it may be not full!)
    } else {
        msg.clear(); // empty DATA field
    }

    lastCameraMessage = buff_ptr;

    return lastEPIXerror;
}


int CameraLinkHandler::write(const byte_array_t &msg, const long timeout)
{
    lastETXerror = CL_ETX;
    lastEPIXerror = 0;

    if ( timeout < 0 ) return pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now;

    if ( msg.size() == 0 ) return pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now;


    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    // compute expected full length of host UART mesage
    size_t UART_len = msg.size() + 1; // add mandatory for host message ETX byte
    if ( CK_SUM_Bit ) ++UART_len;

    std::unique_ptr<char[]> buff = std::unique_ptr<char[]>(new char[UART_len]);
    char* buff_ptr = buff.get();

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
    int n_bytes;

    lastEPIXerror = 0;

    auto start = std::chrono::system_clock::now();

    while ( n_written < UART_len ) {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= timeout_counts ) {
            return lastEPIXerror = PXERTIMEOUT;
        }

        n_bytes = pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now
        if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;

        while ( !n_bytes ); { // wait for at least 1 byte in Tx-buffer
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end-start;
            if ( diff.count() >= timeout_counts ) {
                return lastEPIXerror = PXERTIMEOUT;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
            n_bytes = pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now
            if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;
        }

        // it should return immediately
        n_bytes = pxd_serialWrite(currentUnitmap, 0, buff_ptr + n_written, n_bytes);
        if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;

        n_written += n_bytes;
    }

    lastHostMessage = buff_ptr;

    return lastEPIXerror;
}


bool CameraLinkHandler::exec(const byte_array_t &command, byte_array_t &ans, const long timeout, const long sleep)
{
    write(command, timeout);
    if ( !lastEPIXerror && (lastETXerror == CL_ETX) ) return true; else return false;

    if ( sleep > 0 ) std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    read(ans,timeout);

    if ( !lastEPIXerror && (lastETXerror == CL_ETX) ) return true; else return false;
}


int CameraLinkHandler::reset()
{
    lastEPIXerror = 0;
    byte_array_t cam_msg(2);

    byte_array_t cmd  = {0x55, 0x99, 0x66, 0x11}; // reset FPGA

    int ret = write(cmd); // there is no camera response for this command!!!
    if ( ret < 0 ) return lastEPIXerror = ret;

    byte_array_t msg = {0x4F, 0x51};  // set 'FPGA in RST' bit to 0 (reset)

    while ( !lastEPIXerror ) { // poll camera with 'msg' and wait for camera response ...
        ret = write(msg);
        if ( ret < 0 ) return lastEPIXerror = ret;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        lastEPIXerror = read(cam_msg);
        if ( lastETXerror != CL_ETX ) break;
    }

    return lastEPIXerror;
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

    return lastEPIXerror;
}
