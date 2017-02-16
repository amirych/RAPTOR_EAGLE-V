#include "cameralink_handler.h"

#include <memory>
#include <algorithm>

CameraLinkHandler::CameraLinkHandler(const int unitmap):
    currentUnitmap(unitmap), currentSpeed(CL_DEFAULT_BAUD_RATE),
    lastEPIXerror(0), lastETXerror(CL_ETX),
    ACK_Bit(CL_DEFAULT_ACK_ENABLED), CK_SUM_Bit(CL_DEFAULT_CK_SUM_ENABLED),
    FPGAinRST_Bit(true), FPGA_EPROM_Bit(false)
{
    lastEPIXerror = config(CL_DEFAULT_BAUD_RATE, CL_DEFAULT_DATA_BITS, CL_DEFAULT_STOP_BIT);
    if ( lastEPIXerror < 0 ) return;

    setMode(ACK_Bit, CK_SUM_Bit);
}


int CameraLinkHandler::getLastEPIXError() const
{
    return lastEPIXerror;
}


int CameraLinkHandler::getLastETXError() const
{
    return lastETXerror;
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
    return lastEPIXerror;
}

int CameraLinkHandler::read(byte_array_t &msg, const long timeout)
{
    if ( !msg.size() ) return lastEPIXerror = PXERBADPARM;

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    // compute expected full length of camera UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;

    std::unique_ptr<char[]> buff = std::unique_ptr<char[]>( new char[UART_len]);
    char* buff_ptr = buff.get();

    size_t n_received = 0;
    int n_read;

    lastEPIXerror = 0;
    lastETXerror = 0;

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
                                         // Note: if n_received > UART_len then Chk_Sum field was read (do not check it)
            if ( ACK_Bit ) lastETXerror = buff_ptr[msg.size()];
        }
    }

    size_t k = n_received > msg.size() ? msg.size() : n_received;

    for ( size_t i = 0; i < k; ++i ) msg[i] = buff_ptr[i]; // copy received DATA field (it may be not full!)

    if ( n_received > UART_len ) { // just check ...
        // it seems there is a mismatch between expected length of camera UART message
        // and real one!
        lastEPIXerror = PXERBADPARM;
    }

    return lastEPIXerror;
}


int CameraLinkHandler::write(const byte_array_t &msg, const long timeout)
{
    lastETXerror = 0;

    if ( msg.size() == 0 ) return 0;

    std::chrono::milliseconds readTimeout{timeout};
    size_t timeout_counts = readTimeout.count();
    size_t sleep_msecs = timeout_counts/10;

    // compute expected full length of host UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;

    std::unique_ptr<char[]> buff = std::unique_ptr<char[]>(new char[UART_len]);
    char* buff_ptr = buff.get();

    std::copy_n(msg.data(),msg.size(),buff_ptr);

    if ( ACK_Bit ) buff_ptr[msg.size()] = CL_ETX; // add ACK
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

        // it should return imediately
        n_bytes = pxd_serialWrite(currentUnitmap, 0, buff_ptr + n_written, n_bytes);
        if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;

        n_written += n_bytes;
    }
}

int CameraLinkHandler::reset()
{
    lastEPIXerror = 0;
    byte_array_t cam_msg(2);

    byte_array_t cmd  = {0x55, 0x99, 0x66, 0x11}; // reset FPGA

    int ret = write(cmd); // there is no camera response for this command!!!
    if ( ret < 0 ) return lastEPIXerror = ret;

    byte_array_t msg = {0x4F, 0x51};  // set 'FPGA in RST' bit to 0 (reset)

    ret = write(msg);
    if ( ret < 0 ) return lastEPIXerror = ret;

    for ( ;; ) { // wait for camera response ...
        lastEPIXerror = read(cam_msg);
        if ( !lastEPIXerror ) std::this_thread::sleep_for(std::chrono::milliseconds(500)); else break;
    }
}

//int CameraLinkHandler::reset()
//{
//    unsigned char cmd[] = {0x55, 0x99, 0x66, 0x11, 0x50, 0xEB}; // ACK = 1, CK_SUM = 1

//    lastEPIXerror = pxd_serialWrite(currentUnitmap, 0, (char*)cmd, 6); // there is no camera response for this command!!!
//    if ( lastEPIXerror < 0 ) return lastEPIXerror;

//    unsigned char msg[] = {0x4F, 0x51, 0x50, 0x4E}; // set FPGA in RST bit

//    unsigned char rx_buff[2] = {0,0};
//    int n_recieved = 0;

//    auto start = std::chrono::system_clock::now();
//    for (;;) { // poll with msg and wait for Rx bytes ...
//        lastEPIXerror = pxd_serialWrite(currentUnitmap, 0, (char*)msg, 6);
//        std::this_thread::sleep_for(std::chrono::milliseconds(500));
//        int n = pxd_serialRead(currentUnitmap, 0, (char*)rx_buff + n_recieved, 2 - n_recieved);
//        if ( n < 0 ) return n;
//        n_recieved += n;
//        if ( n_recieved < 2 ) {
//            auto end  = std::chrono::system_clock::now();
//            std::chrono::duration<double> diff = end-start;
//            if ( diff.count() >= OpTimeout.count() ) return PXERTIMEOUT;
//            continue;
//        }
//    }

//    lastETXerror = rx_buff[0];
//    if ( lastETXerror == 0x50 ) {

//    }

//    return lastEPIXerror;
//}



int CameraLinkHandler::setSystemState(const bool ck_sum_bit, const bool ack_bit,
                                      const bool fpga_in_reset, const bool fpga_eprom)
{
    unsigned char mode = 0;

    if ( ck_sum_bit ) mode |= CL_SYSTEM_STATE_CK_SUM;
    if ( ack_bit ) mode |= CL_SYSTEM_STATE_ACK;
    if ( fpga_in_reset ) mode |= CL_SYSTEM_STATE_FPGA_RST_HOLD;
    if ( fpga_eprom ) mode |= CL_SYSTEM_STATE_FPGA_EPROM_COMMS;

    unsigned char msg[] = {0x4F, mode};

    return lastEPIXerror;
}
