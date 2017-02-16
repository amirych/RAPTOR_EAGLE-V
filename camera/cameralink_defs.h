#ifndef CAMERALINK_DEFS_H
#define CAMERALINK_DEFS_H

                /***************************************************
                *                                                  *
                *   DEFINITIONS FOR CAMERALINK PROTOCOL COMMANDS   *
                *       TO MANAGE RAPTOR EAGLE V 4240 CAMERA       *
                *                                                  *
                ****************************************************/


               /* from EAGLE V 4240 Instruction Manual Revision 1.1 */


// default port settings
// the values are ones for EB1 grabber card (see XCLIB Reference manual)

#define CL_DEFAULT_TIMEOUT 10000 // default read/write operation timeout in milliseconds (10 secs)

#define CL_DEFAULT_BAUD_RATE 115200
#define CL_DEFAULT_START_BIT 1
#define CL_DEFAULT_DATA_BITS 8
#define CL_DEFAULT_STOP_BIT 1

#define CL_DEFAULT_ACK_ENABLED 1
#define CL_DEFAULT_CK_SUM_ENABLED 1

// ETX/ERROR codes

#define CL_ETX 0x50
#define CL_ETX_SER_TIMEOUT 0x51
#define CL_ETX_CK_SUM_ERR  0x52
#define CL_ETXI2C_ERR      0x53
#define CL_ETX_UNKNOWN_CMD 0x54
#define CL_ETX_DONE_LOW    0x55


                    /*  BIT MASKS (0-7 bits) */

// system state

#define CL_SYSTEM_STATE_CK_SUM         0x40 // 6-th bit
#define CL_SYSTEM_STATE_ACK            0x10 // 4-th bit
#define CL_SYSTEM_STATE_FPGA_BOOT_OK   0x4 // 2-nd bit
#define CL_SYSTEM_STATE_FPGA_RST_HOLD     0x2 // 1-st bit
#define CL_SYSTEM_STATE_FPGA_EPROM_COMMS  0x1 // 0-th bit

// FPGA CTRL register

#define CL_FPGA_CTRL_REG_HIGH_GAIN     0x80 // 7-th bit (high pre-amp gain)
#define CL_FPGA_CTRL_REG_TMP_TRIP_RST  0x2  // 1-st bit
#define CL_FPGA_CTRL_REG_ENABLE_TEC    0x1  // 0-th bit

// trigger mode

#define CL_TRIGGER_MODE_ENABLE_RISING_EDGE  0x80 // 7-th bit
#define CL_TRIGGER_MODE_EXT_TRIGGER         0x40 // 6-th bit
#define CL_TRIGGER_MODE_ABORT_CURRENT_EXP   0x8  // 3-rd bit
#define CL_TRIGGER_MODE_CONTINUOUS_SEQ      0x4  // 2-nd bit
#define CL_TRIGGER_MODE_FIXED_FRAME_RATE    0x2  // 1-st bit
#define CL_TRIGGER_MODE_SNAPSHOT            0x1  // 0-th bit


// extract TEC set point value (sourcePtr[0] - LSB 8-bit long part, sourcePtr[1] - MSB right 4-bits )

#define CL_TEC_SET_POINT_GET_VALUE(sourcePtr) ( sourcePtr[0] + ((sourcePtr[1] & 0x0F) << 8) )
#define CL_TEC_SET_POINT_SET_VALUE(value) ()

            /*  SETUP CONTROL VALUES */

// shutter

#define CL_SUTTER_CLOSED  0x0
#define CL_SHUTTER_OPEN   0x1
#define CL_SHUTTER_EXP    0x2

// readout rate (registers vales)

#define CL_READOUT_CLOCK_RATE_A3_2MHZ  0x02
#define CL_READOUT_CLOCK_RATE_A4_2MHZ  0x02
#define CL_READOUT_CLOCK_RATE_A3_75KHZ  0x43
#define CL_READOUT_CLOCK_RATE_A4_75KHZ  0x80

// readout mode

#define CL_READOUT_MODE_NORMAL  0x01
#define CL_READOUT_MODE_TEST    0x04


#endif // CAMERALINK_DEFS_H
