#ifndef CAMERALINK_HANDLER_H
#define CAMERALINK_HANDLER_H

#include <vector>
#include <chrono>
#include <thread>

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
    #include <windows.h>
#endif

#include <cameralink_defs.h>
#include <xcliball.h>

class CameraLinkHandler
{
public:
    typedef std::vector<unsigned char> byte_array_t;

    CameraLinkHandler(const int unitmap = 1);
    CameraLinkHandler(const long speed, const int bits = CL_DEFAULT_DATA_BITS);
    CameraLinkHandler(const long speed, const int bits, const int stop_bits = CL_DEFAULT_STOP_BIT);

    int config(const long speed, const int bits, const int stop_bits);
    int setMode(const bool ack_enabled, const bool ck_sum_enabled);

    int reset();

    int getLastEPIXError() const;
    int getLastETXError() const;

    int read(byte_array_t &msg, const long timeout = CL_DEFAULT_TIMEOUT);
                                 // reads from port msg.size() bytes.
                                 // If ACK bit enabled then also wait for ACK byte (ETX field).
                                 // If CK_SUM bit enabled then also wait for CK_SUM byte (Chk_Sum field).
                                 // if ACK and/or CK_SUM bits are not enabled then wait for data 'timeout' milliseconds, reads and returns.
                                 // the method returns in 'msg' only DATA field, so 'msg' will not contain possible ETX end Chk_Sum
                                 // fields of camera UART message.
                                 // the method sets lastETHError as a value of ETX field of camera UART message (if ACK bit is
                                 // enabled or lastETHError is 0)
                                 // the method's exit value is a result of the pxd_serialRead XCLIB function
                                 // if ACK and CK_SUM bytes will not received during a value of read/write operation timeout
                                 // then return PXERTIMEOUT

    int write(const byte_array_t &msg, const long timeout = CL_DEFAULT_TIMEOUT);
                                        // write 'msg' bytes. it is assumed that 'msg' contains only DATA field of
                                        // host UART mesage. the method add ETX field and computes check sum itself (if needed).
                                        // the method's return value is the result of pxd_serialWrite XCLIB function

    int setSpeed(const long speed);
    long getSpeed() const;

private:
    int currentUnitmap;
    long currentSpeed;

    int lastEPIXerror;
    int lastETXerror;

    bool ACK_Bit;
    bool CK_SUM_Bit;
    bool FPGAinRST_Bit;
    bool FPGA_EPROM_Bit;

    int setSystemState(const bool ck_sum_bit, const bool ack_bit, const bool fpga_in_reset, const bool fpga_eprom);
};

#endif // CAMERALINK_HANDLER_H
