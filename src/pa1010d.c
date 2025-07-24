#include "gci_sensors/pa1010d.h"
#include <string.h> // memcpy

#define PA1010D_ADDRESS 0x10
#define LOOP_FAIL 5
#define MAX_NEMA_SIZE 82

static inline bool ascii_nema(const char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || c == '*' || c == ',' || c == '.';
}

pa1010d_io_t *pa1010d_i2c_init(uint32_t port) {
  uint8_t id = 0;
  int32_t ok;

  pa1010d_io_t *hw = (pa1010d_io_t *)calloc(1, sizeof(pa1010d_io_t));
  if (hw == NULL) return NULL;

  comm_interface_t *comm = comm_interface_init(port, PA1010D_ADDRESS, I2C_INTERFACE);
  if (comm == NULL) return NULL;

  hw->comm = comm;

  return hw;
}

int32_t pa1010d_write(pa1010d_io_t *hw, const uint8_t *command, uint16_t cmd_size) {
  comm_interface_t *comm = hw->comm;
  // return i2c_write_blocking(hw->i2c, hw->addr, command, cmd_size, false);
  // return writeRegister(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
  comm->write(comm->config, 0x00, command, cmd_size);
}

// Get message from GPS and return the message string
// with '\0' appended to end
int32_t pa1010d_read(pa1010d_io_t *hw, char buff[], const uint16_t buff_size) {
  size_t i               = 0;
  size_t j               = 0;
  char c                 = 0;
  int32_t ok             = 0;
  comm_interface_t *comm = hw->comm;

  // ok = i2c_read_blocking(hw->i2c, hw->addr, hw->buffer, MAX_NEMA_SIZE, false);
  comm->read(comm->config, 0x00, hw->buffer, MAX_NEMA_SIZE);
  if (ok < 0) return ok;

  // find start character $
  while (c != '$') {
    c = hw->buffer[i++];
    if (i == PA1010D_BUFFER_SIZE) return -1;
  }

  j         = 0;
  buff[j++] = c; // save

  // find end characters '\r' and '\n'
  while (j < buff_size - 1) {
    c = hw->buffer[i++];

    if (c == '$') return -1; // BAD
    else if (c == '\r') {    // end chars
      buff[j++] = c;
      c         = hw->buffer[i++];

      if (c != '\n') return -1;

      buff[j++] = c;
      buff[j]   = '\0';
      return j;
    }
    else if (ascii_nema(c)) buff[j++] = (char)c;
  }
  return -1; // how get here?
}

// int32_t pa1010d_write(pa1010d_i2c_t* hw, char command[], uint32_t cmd_size) {
//   return i2c_write_blocking(i2c, addr, (uint8_t*)command, cmd_size, false);
//   // return writeRegister(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
//   // return gci_i2c_write(hw->i2c, hw->addr, REG_ODR, &odr, 1);
// }

// pa1010d_i2c_t *pa1010d_i2c_init(uint32_t port, uint8_t addr) {
//   uint8_t id = 0;
//   int32_t ok;

//   pa1010d_i2c_t *hw = NULL;
//   hw                = (pa1010d_i2c_t *)calloc(1, sizeof(pa1010d_i2c_t));
//   if (hw == NULL) return NULL;
//   hw->i2c  = (port == 0) ? i2c0 : i2c1;
//   hw->addr = addr;
//   memset(hw->buffer, 0, I2C_BUFFER_SIZE);

//   return hw;
// }

// // int32_t pa1010d_write(pa1010d_i2c_t* hw, char command[], uint32_t cmd_size) {
// //   return i2c_write_blocking(i2c, addr, (uint8_t*)command, cmd_size, false);
// //   // return writeRegister(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
// //   // return gci_i2c_write(hw->i2c, hw->addr, REG_ODR, &odr, 1);
// // }

// int32_t pa1010d_write(pa1010d_i2c_t *hw, const uint8_t *command, size_t cmd_size) {
//   return i2c_write_blocking(hw->i2c, hw->addr, command, cmd_size, false);
//   // return writeRegister(addr, cmd_size, (uint8_t*)command) ? 0 : 1;
// }

// // Get message from GPS and return the message string
// // with '\0' appended to end
// int32_t pa1010d_read(pa1010d_i2c_t *hw, char buff[], const size_t buff_size) {
//   size_t i   = 0;
//   size_t j   = 0;
//   char c     = 0;
//   int32_t ok = 0;

//   ok = i2c_read_blocking(hw->i2c, hw->addr, hw->buffer, MAX_NEMA_SIZE, false);
//   if (ok < 0) return ok;

//   // find start character $
//   while (c != '$') {
//     c = hw->buffer[i++];
//     if (i == MAX_NEMA_SIZE) return -1;
//   }

//   j         = 0;
//   buff[j++] = c; // save

//   // find end characters '\r' and '\n'
//   while (j < buff_size - 1) {
//     c = hw->buffer[i++];

//     if (c == '$') return -1; // BAD
//     else if (c == '\r') {    // end chars
//       buff[j++] = c;
//       c         = hw->buffer[i++];

//       if (c != '\n') return -1;

//       buff[j++] = c;
//       buff[j]   = '\0';
//       return j;
//     }
//     else if (ascii_nema(c)) buff[j++] = (char)c;
//   }
//   return -1; // how get here?
// }

// uint32_t pa1010d_read(pa1010d_i2c_t *hw, char buff[], const size_t buff_size) {
//   uint32_t i  = 10000;
//   uint32_t iw = 0;
//   // uint32_t start = 0;
//   // uint32_t end = 0;
//   uint8_t loop_fail = 0;
//   char c            = 0;
//   int32_t ok;

//   while (c != '$') {
//     if (i >= I2C_BUFFER_SIZE) {
//       // if we do this too many time fail
//       if (loop_fail++ >= LOOP_FAIL) return 0;
//       i = 0;
//       memset(hw->buffer, 0, I2C_BUFFER_SIZE);
//       ok = i2c_read_blocking(hw->i2c, hw->addr, hw->buffer, I2C_BUFFER_SIZE, false);
//     }
//     c = hw->buffer[i++];
//   }
//   buff[iw++] = c; // save
//   loop_fail  = 0;

//   // find end char '\r' and '\n'
//   while (iw < buff_size - 1) {
//     if (i >= I2C_BUFFER_SIZE) {
//       // if we do this too many time fail
//       if (loop_fail++ >= LOOP_FAIL) return 0;
//       i = 0;
//       memset(hw->buffer, 0, I2C_BUFFER_SIZE);
//       i2c_read_blocking(hw->i2c, hw->addr, hw->buffer, I2C_BUFFER_SIZE, false);
//     }
//     c = hw->buffer[i++];

//     if (c == '$') return 0; // BAD
//     else if (c == '\r') {   // end chars
//       buff[iw++] = c;
//       c          = hw->buffer[i++];
//       if (c != '\n') {
//         return 0;
//       }
//       buff[iw++] = c;
//       buff[iw]   = '\0';
//       return iw;
//     }
//     else if (ascii_nema(c)) buff[iw++] = c;
//   }
//   return 0; // how get here?
// }
