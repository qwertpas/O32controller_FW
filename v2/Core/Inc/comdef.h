/*

comdef.h

* This file includes the definitions for the commands to send to the Ø32 motor controller.
  Each interaction with a motor controller consists of five 8-bit bytes:
    - 1:   Master sends command byte.
        - 1 bit header that indicates the start of the interaction (how others will know it’s a command byte)
        - 4 bit address (allowing for 16 motor controllers)
        - 3 bit command ID (8 unique)
            - control modes [voltage, speed, position, current]
            - data requests [position, speed, current, temperature]
    - 2-3: Master sends 2 command data bytes relevant to the command ID. Each byte contains:
        - 1 bit header that indicates in the middle of interaction (not a command byte)
        - 7 bit data (with 2 bytes, that’s a 14 bit number so 0-16383. That’s almost half of an int32, so I can do two addition operations at once)
    - 4-5: Motor controller is constantly receiving and checking for a command byte that contains its address. It responds with 2 response bytes, each consisting of:
        - 1 bit header saying it’s not a command byte
        - 7 bit data

 * Command byte from Teensy to particular Ø32 address
   Bits:      [   7  | 6 - 4 | 3 - 0 ]
   Purpose:   [   1  |  CMD  |  ADRR ]

 * Data byte between Teensy and Ø32 (both ways)
   Bits:      [   7  | 6 - 0 ]
   Purpose:   [   0  |  data ]

 * Usage:
   Command a set voltage to address 3 on Teensy: Serial2.write(CMD_SET_VTG | 3);
   Check if a byte is a voltage command on Ø32: if(p.uart_cmd[0] & CMD_MASK == CMD_SET_VOLTAGE)

*/

#ifndef INC_COMDEF_H_
#define INC_COMDEF_H_

#define CMD_SET_VOLTAGE (0b10000000)
#define CMD_SET_SPEED (0b10010000)
#define CMD_SET_POSITION (0b10100000)
#define CMD_SET_CURRENT (0b10110000)
#define CMD_GET_POSITION (0b11000000)
#define CMD_GET_SPEED (0b11010000)
#define CMD_GET_CURRENT (0b11100000)
#define CMD_GET_TEMPERATURE (0b11110000)

#define CMD_MASK (0xF0)

#endif /* INC_COMDEF_H_ */
