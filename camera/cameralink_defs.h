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

#define CL_FPGA_CTRL_REG_HIGH_GAIN     0x80 // 7-th bit (0 if high pre-amp gain)
#define CL_FPGA_CTRL_REG_TMP_TRIP_RST  0x2  // 1-st bit
#define CL_FPGA_CTRL_REG_ENABLE_TEC    0x1  // 0-th bit

// trigger mode

#define CL_TRIGGER_MODE_ENABLE_RISING_EDGE  0x80 // 7-th bit
#define CL_TRIGGER_MODE_EXT_TRIGGER         0x40 // 6-th bit
#define CL_TRIGGER_MODE_ABORT_CURRENT_EXP   0x8  // 3-rd bit
#define CL_TRIGGER_MODE_CONTINUOUS_SEQ      0x4  // 2-nd bit
#define CL_TRIGGER_MODE_FIXED_FRAME_RATE    0x2  // 1-st bit
#define CL_TRIGGER_MODE_SNAPSHOT            0x1  // 0-th bit


            /*  SETUP CONTROL VALUES */

// shutter

#define CL_SHUTTER_CLOSED  0x0
#define CL_SHUTTER_OPEN    0x1
#define CL_SHUTTER_EXP     0x2

// readout rate (registers vales)

#define CL_READOUT_CLOCK_RATE_A3_2MHZ  0x02
#define CL_READOUT_CLOCK_RATE_A4_2MHZ  0x02
#define CL_READOUT_CLOCK_RATE_A3_75KHZ  0x43
#define CL_READOUT_CLOCK_RATE_A4_75KHZ  0x80

// readout mode

#define CL_READOUT_MODE_NORMAL  0x01
#define CL_READOUT_MODE_TEST    0x04


#define ADC_CALIBRATION_POINT_1 0.0   // 0 Celcius degree
#define ADC_CALIBRATION_POINT_2 40.0  // +40 Celcius degree

#define DAC_CALIBRATION_POINT_1 0.0   // 0 Celcius degree
#define DAC_CALIBRATION_POINT_2 40.0  // +40 Celcius degree

           /* COMMANDS  */

#define CL_COMMAND_SET_ADDRESS {0x53, 0xE0, 0x01, 0x00} // set address given by the last byte. the last byte should be set by user)
#define CL_COMMAND_READ_VALUE {0x53, 0xE1, 0x01}        // read a byte value

#define CL_COMMAND_WRITE_VALUE {0x53, 0xE0, 0x02, 0x00, 0x00 } // write a byte value given by the last byte
                                                               // at address given by 3-rd byte (starting from 0)

#define CL_COMMAND_GET_MANUFACTURER_DATA_1 {0x53, 0xAE, 0x05, 0x01, 0x00, 0x00, 0x02, 0x00} // 1st command to get manufacturer's data
#define CL_COMMAND_GET_MANUFACTURER_DATA_2 {0x53, 0xAF, 0x12} // the second one


#endif // CAMERALINK_DEFS_H
