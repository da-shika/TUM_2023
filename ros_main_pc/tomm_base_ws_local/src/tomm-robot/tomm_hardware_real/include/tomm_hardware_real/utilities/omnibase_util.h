#ifndef OMNIBASE_UTIL_H_
#define OMNIBASE_UTIL_H_

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

// Constants APM-SC05-ADK9 + 8" HD AndyMark wheels
#define ODOMETRY_CONSTANT 626594.7934  // in ticks/m
#define DRIVE_CONSTANT 626594.7934
#define ODOMETRY_CORRECTION 1.0
#define MAX_TICK_SPEED 833333          // ticks/s : 5000 rpm/ 60s * 10000 ticks/rev

// soem
#define EC_TIMEOUTMON 500

// Driver Regiser Map
#define PROFILE_DECELERATION 0x6084
#define PROFILE_ACCELERATION 0x6083
#define PROFILE_VELOCITY 0x6081
#define DIGITAL_OUTPUTS 0x60FE
#define TARGET_POSITION 0x607a
#define TARGET_VELOCITY 0x60ff
#define TARGET_TORQUE 0x6071
#define MAX_TORQUE 0x6072
#define CONTROL_WORD 0x6040
#define OPERATION_MODE 0x6060

#define POSITION 0x6064
#define DIGITAL_INPUTS 0x60FD
#define VELOCITY 0x606C
#define STATUS_WORD 0x6041
#define OPERATION_MODE_DISPLAY 0x6061
#define TORQUE 0x6077

#define PDO_OUTPUT_ASSIGNMENT 0x1c12
#define PDO_INPUT_ASSIGNMENT 0x1c13

#define PDO_RECEIVE_OBJ 0x16010001
#define PDO_TRANSMIT_OBJ 0x1a030001

#define LIMIT(x, l) ( (x>l) ? l : (x<-l) ? -l : x )
#define LIMIT_MIN_MAX(x, min, max) ( (x>max) ? max : (x<min) ? min : x )

namespace omnibase_util
{
/*
 * register read fnction overloads
 */
inline int read_register(uint16 slave, uint16 index, uint8 subindex, int32_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}
inline int read_register(uint16 slave, uint16 index, uint8 subindex, uint32_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}
inline int read_register(uint16 slave, uint16 index, uint8 subindex, int16_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}
inline int read_register(uint16 slave, uint16 index, uint8 subindex, uint16_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}
inline int read_register(uint16 slave, uint16 index, uint8 subindex, int8_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}
inline int read_register(uint16 slave, uint16 index, uint8 subindex, uint8_t& buf) {
   buf = 0;
   int __s = sizeof( buf );
   return ec_SDOread(slave, index, subindex, FALSE, &__s, &buf, EC_TIMEOUTRXM);
}

/*
 * register write fnction overloads
 */
inline int write_register(uint16 slave, uint16 index, uint8 subindex, int32_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}
inline int write_register(uint16 slave, uint16 index, uint8 subindex, uint32_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}
inline int write_register(uint16 slave, uint16 index, uint8 subindex, int16_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}
inline int write_register(uint16 slave, uint16 index, uint8 subindex, uint16_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}
inline int write_register(uint16 slave, uint16 index, uint8 subindex, int8_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}
inline int write_register(uint16 slave, uint16 index, uint8 subindex, uint8_t val) {
   int __s = sizeof( val );
   return ec_SDOwrite(slave, index, subindex, FALSE, __s, &val, EC_TIMEOUTRXM);
}

}

#endif