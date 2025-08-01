
#ifndef XIAOMI_CYBERGEAR_DEFS_H
#define XIAOMI_CYBERGEAR_DEFS_H
// https://openelab.io/blogs/learn/xiaomi-cybergear-motor-driver
#define CMD_DEVICE_ID                 0x00
#define CMD_CONTROL                   0x01
#define CMD_REQUEST                   0x02
#define CMD_ENABLE                    0x03
#define CMD_STOP                      0x04
#define CMD_SET_MECH_POSITION_TO_ZERO 0x06
#define CMD_SET_CAN_ID                0x07
#define CMD_RAM_READ                  0x11
#define CMD_RAM_WRITE                 0x12
#define CMD_FEEDBACK                  0x15
#define CMD_BAUD_RATE                 0x16

#define ADDR_RUN_MODE               0x7005
#define ADDR_I_REF                  0x7006
#define ADDR_SPEED_REF              0x700A
#define ADDR_LIMIT_TORQUE           0x700B
#define ADDR_CURRENT_KP             0x7010
#define ADDR_CURRENT_KI             0x7011
#define ADDR_CURRENT_FILTER_GAIN    0x7014
#define ADDR_POSITION_REF           0x7016
#define ADDR_LIMIT_SPEED            0x7017
#define ADDR_LIMIT_CURRENT          0x7018
#define ADDR_SPEED_KP               0x701F
#define ADDR_SPEED_KI               0x7020

#define MODE_MOTION                   0x00
#define MODE_POSITION                 0x01
#define MODE_SPEED                    0x02
#define MODE_CURRENT                  0x03

#define POS_MAX                    12.5f
#define POS_MAX                     2.0f
#define V_MAX                      30.0f
#define KP_MIN                      0.0f
#define KP_MAX                    500.0f
#define KI_MIN                      0.0f
#define KI_MAX                     10.0f
#define KD_MIN                      0.0f
#define KD_MAX                      5.0f
#define T_MAX                      12.0f
#define I_MAX                      27.0f
#define CURRENT_FILTER_GAIN_MIN     0.0f
#define CURRENT_FILTER_GAIN_MAX     1.0f

#define RET_CYBERGEAR_OK              0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL   0x01
#define RET_CYBERGEAR_INVALID_CAN_ID  0x02
#define RET_CYBERGEAR_INVALID_PACKET  0x03

#endif // XIAOMI_CYBERGEAR_DEFS_H
