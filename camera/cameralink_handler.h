#ifndef CAMERALINK_HANDLER_H
#define CAMERALINK_HANDLER_H

#include <vector>
#include <map>
#include <chrono>
#include <thread>
#include <memory>

#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
    #include <windows.h>
#endif

#include <cameralink_defs.h>
#include <xcliball.h>


#define CAMERALINK_HANDLER_LOG_MSG_LEN 200

class CameraLinkHandler
{
public:
    typedef std::vector<unsigned char> byte_array_t;

    // function type for extra-logging facility

    typedef std::function<void(const std::string&)> log_func_t;


    CameraLinkHandler(const int unitmap = 1);
//    CameraLinkHandler(const long speed, const int bits = CL_DEFAULT_DATA_BITS);
//    CameraLinkHandler(const long speed, const int bits, const int stop_bits = CL_DEFAULT_STOP_BIT);

    ~CameraLinkHandler();

    void setUnitmap(const int unitmap, const bool force = false);
                                 // set unitmap. If unitmap was not set previously then configure CameraLink serial port.
                                 // If force is true than configure CameraLink serial port for even it was done earlier.
                                 // If unitmap <= 0 then lastXCLibError = PXERBADPARM

    int getUnitMap() const;

    int config(const long speed, const int bits, const int stop_bits);  // configure CameraLink serial port
    int setMode(const bool ack_enabled, const bool ck_sum_enabled);
    int getMode();
    int setSystemState(const bool ck_sum_bit, const bool ack_bit, const bool fpga_in_reset, const bool fpga_eprom);

    int reset();  // reset camera firmware

    int getLastXCLibError() const;        // error returned by XCLIB PIXCI library functions
    char getLastControllerError() const;  // error returned by Raptor Eagle-V camera controller

    bool isValid() const;  // is last transmittion operation was successfull
                           // valid state is one with lastXCLibError >= 0 and lastControllerError = CL_ETX

    const char* getLastHostMessage() const;    // last full UART Tx-message (from host to camera) including possible ETX and Chk_Sum fields
    const char* getLastCameraMessage() const;  // last full UART Rx-message (from camera to host) including possible ETX and Chk_Sum fields

    int read(byte_array_t &msg, const long timeout = CL_DEFAULT_TIMEOUT);
                                 // reads from port msg.size() bytes.
                                 // If ACK bit enabled then also wait for ACK byte (ETX field).
                                 // If CK_SUM bit enabled then also wait for CK_SUM byte (Chk_Sum field).
                                 // if both ACK and CK_SUM bits are not enabled then wait atleast 'timeout' milliseconds for data bytes,
                                 // reads and returns.
                                 // the method returns in 'msg' only DATA field, so 'msg' will not contain possible ETX end Chk_Sum
                                 // fields of camera UART message.
                                 // the method sets lastControllerError as a value of ETX field of camera UART message (if ACK bit is
                                 // enabled or lastETHError is 0)
                                 // the method's exit value is a result of the pxd_serialRead XCLIB function
                                 // if ACK and CK_SUM bytes will not received during a value of read/write operation timeout
                                 // then return PXERTIMEOUT

    int write(const byte_array_t &msg, const long timeout = CL_DEFAULT_TIMEOUT);
                                        // write 'msg' bytes. it is assumed that 'msg' contains only DATA field of
                                        // host UART mesage. the method add ETX field and computes check sum itself (if needed).
                                        // if an error wil occured the method's return value is the result of pxd_serialWrite
                                        // XCLIB function, or 0 in success.
                                        // if msg.size() == 0, then return result of pxd_serialWrite(unitmap,0,NULL,0)

    void exec(const byte_array_t &command, byte_array_t &ans, const long timeout = CL_DEFAULT_TIMEOUT, const long sleep = 0);
                                        // execute a command. 'ans' will contain an camera answer.
                                        // the 'command' and 'ans' arguments follow to 'read' and 'write' methods description.
                                        // 'timeout' is timeout in milliseconds for read/write operations.
                                        // 'sleep' is a sleep period in milliseconds between 'write command' and
                                        // 'read answer' operations

    void exec(const byte_array_t &command, const long timeout = CL_DEFAULT_TIMEOUT, const long sleep = 0);

    int setSpeed(const long speed);
    long getSpeed() const;

    void setLogFunc(const log_func_t &func = nullptr);

private:
    int currentUnitmap;
    long currentSpeed;

    int lastXCLibError;         // error returned by XCLIB PIXCI library functions
    char lastControllerError;   // error returned by Raptor Eagle-V camera controller

    bool ACK_Bit;
    bool CK_SUM_Bit;
    bool FPGAinRST_Bit;
    bool FPGA_EPROM_Bit;

    std::unique_ptr<char[]>  tx_buff;     // full host UART message
    std::unique_ptr<char[]>  rx_buff;     // full camera UART message

    std::unique_ptr<char[]> lastLogMessageUniqPtr;
    char* lastLogMessage;

    unsigned char currentSystemState;

    log_func_t logFunc;

    static std::map<int,int> usedUnitmaps;
};

#endif // CAMERALINK_HANDLER_H
