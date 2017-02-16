#include "cameralink_handler.h"

#include <memory>

CameraLinkHandler::CameraLinkHandler(const int unitmap):
    currentUnitmap(unitmap), currentSpeed(CL_DEFAULT_BAUD_RATE),
    lastEPIXerror(0), lastETXerror(CL_ETX),
    ACK_Bit(CL_DEFAULT_ACK_ENABLED), CK_SUM_Bit(CL_DEFAULT_CK_SUM_ENABLED),
    FPGAinRST_Bit(true), FPGA_EPROM_Bit(false),
    OpTimeout(std::chrono::milliseconds(CL_DEFAULT_TIMEOUT)), OpTimeoutMillisecs(CL_DEFAULT_TIMEOUT)
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


void CameraLinkHandler::setTimeout(const long timeout)
{
    OpTimeout = std::chrono::milliseconds(timeout);
}


//
//  must be invoked AFTER pxd_PIXCIopen!!!
//
int CameraLinkHandler::config(const long speed, const int bits, const int stop_bits)
{
    lastEPIXerror = pxd_serialConfigure(currentUnitmap,0,speed,bits,0,stop_bits,0,0,0);
}


int CameraLinkHandler::setMode(const bool ack_enabled, const bool ck_sum_enabled)
{

}

int CameraLinkHandler::read(byte_array_t &msg, const long timeout)
{
    if ( !msg.size() ) return lastEPIXerror = PXERBADPARM;

    std::chrono::milliseconds readTimeout{timeout};
    double timeout_counts;
    size_t sleep_msecs;
    if ( ACK_Bit ) {
        timeout_counts = OpTimeout.count();
        sleep_msecs = OpTimeoutMillisecs/10;
    } else {
        timeout_counts = readTimeout.count();
        sleep_msecs = timeout/10;
    }

    // compute expected full length of camera UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;

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

    for (;;) {
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

        if ( n_received > UART_len ) {
            // it seems there is a mismatch between expected length of camera UART message
            // and real one!
            return lastEPIXerror = PXERBADPARM;
        }

        if ( n_received >= msg.size() ) { // all DATA bytes were read
//            for ( size_t i = 0; i < msg.size(); ++i ) msg[i] = buff_ptr[i]; // copy DATA field

            if ( UART_len == msg.size() ) break; // all is done

            if ( ACK_Bit && (n_received >= (msg.size()+1)) ) { // ETX field is read
                lastETXerror = buff_ptr[msg.size()];
            } else continue; // wait for data bytes ..

            if ( ACK_Bit && CK_SUM_Bit && (n_received >= (msg.size()+2)) ) {  // ETX and Chk_Sum fields are read
                break;
            }

            continue;
        }


    }

    n_received = n_received > msg.size() ? msg.size() : n_received;

    for ( size_t i = 0; i < n_received; ++i ) msg[i] = buff_ptr[i]; // copy received DATA field (it may be not full!)

    return lastEPIXerror;
}


int CameraLinkHandler::write(const byte_array_t &msg)
{
    lastETXerror = 0;

    if ( msg.size() == 0 ) return 0;

    long sleep_msecs = OpTimeoutMillisecs/10;

    // compute expected full length of host UART mesage
    size_t UART_len = msg.size();
    if ( ACK_Bit ) ++UART_len;
    if ( CK_SUM_Bit ) ++UART_len;



    size_t n_written = 0;
    int n_bytes;

    bool data_was_written = false;

    lastEPIXerror = 0;

    char check_sum = msg[0];
    if ( CK_SUM_Bit ) { // compute check sum (XOR for message bytes including ETX field)
        for ( size_t i = 1; i < msg.size(); ++i ) check_sum ~= msg[i];
    }
    if ( ACK_Bit ) check_sum ~= CL_ETX;
    char tail[2] = {CL_ETX, check_sum};


    const char* buff_ptr = msg.data();

    auto start = std::chrono::system_clock::now();

    while ( n_written < UART_len ) {
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        if ( diff.count() >= OpTimeout.count() ) {
            return lastEPIXerror = PXERTIMEOUT;
        }

        n_bytes = pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now
        if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;

        while ( !n_bytes ); { // wait for number of bytes at least 1 byte
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end-start;
            if ( diff.count() >= OpTimeout.count() ) {
                return lastEPIXerror = PXERTIMEOUT;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msecs));
            n_bytes = pxd_serialWrite(currentUnitmap, 0, NULL, 0); // how many bytes are available in Tx-buffer now
            if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;
        }

        // it should return imediately
        if ( !data_was_written ) {
            n_bytes = pxd_serialWrite(currentUnitmap, 0, buff_ptr + n_written, n_bytes);
        } else {
            if ( CK_SUM_Bit )
            n_bytes = pxd_serialWrite(currentUnitmap, 0, buff_ptr + n_written, n_bytes);

        }
        if ( n_bytes < 0 ) return lastEPIXerror = n_bytes;

        n_written += n_bytes;

        if ( UART_len == msg.size() ) return 0; // all is done

        if ( n_written == msg.size() ) {
            data_was_written = true;
            continue;
        }
    }
}


int CameraLinkHandler::write(const char *packet, const size_t packet_len)
{

}


int CameraLinkHandler::reset()
{
    unsigned char cmd[] = {0x55, 0x99, 0x66, 0x11, 0x50, 0xEB}; // ACK = 1, CK_SUM = 1

    lastEPIXerror = pxd_serialWrite(currentUnitmap, 0, (char*)cmd, 6); // there is no camera response for this command!!!
    if ( lastEPIXerror < 0 ) return lastEPIXerror;

    unsigned char msg[] = {0x4F, 0x51, 0x50, 0x4E}; // set FPGA in RST bit

    unsigned char rx_buff[2] = {0,0};
    int n_recieved = 0;

    auto start = std::chrono::system_clock::now();
    for (;;) { // poll with msg and wait for Rx bytes ...
        lastEPIXerror = pxd_serialWrite(currentUnitmap, 0, (char*)msg, 6);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int n = pxd_serialRead(currentUnitmap, 0, (char*)rx_buff + n_recieved, 2 - n_recieved);
        if ( n < 0 ) return n;
        n_recieved += n;
        if ( n_recieved < 2 ) {
            auto end  = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end-start;
            if ( diff.count() >= OpTimeout.count() ) return PXERTIMEOUT;
            continue;
        }
    }

    lastETXerror = rx_buff[0];
    if ( lastETXerror == 0x50 ) {

    }

    return lastEPIXerror;
}



int CameraLinkHandler::setSystemState(const bool ck_sum_bit, const bool ack_bit,
                                      const bool fpga_in_reset, const bool fpga_eprom)
{
    unsigned char mode = 0;

    if ( ck_sum_bit ) mode |= CL_SYSTEM_STATE_CK_SUM;
    if ( ack_bit ) mode |= CL_SYSTEM_STATE_ACK;
    if ( fpga_in_reset ) mode |= CL_SYSTEM_STATE_FPGA_RST_HOLD;
    if ( fpga_eprom ) mode |= CL_SYSTEM_STATE_FPGA_EPROM_COMMS;

    unsigned char msg[] = {0x4F, mode};
}
