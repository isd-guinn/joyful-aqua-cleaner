#ifndef SERIAL_RECEIVE_HPP 
#define SERIAL_RECEIVE_HPP

#include <stdint.h>
#include "uart_slave/MasterSerialProtocol.hpp"
#include "uart_slave/ControllerSerialProtocol.hpp"

namespace ByteUtil
{
  // reinterpret into float
  inline float reconFloat(uint8_t *packet, uint8_t pos, bool isSmallEndian = true)
  {
    if (isSmallEndian)
    {
      uint8_t ctn[4] = {packet[pos+3], packet[pos+2], packet[pos+1], packet[pos]};
      return *(float*)&ctn;
    }
    else
    {
      uint8_t ctn[4] = {packet[pos], packet[pos+1], packet[pos+2], packet[pos+3]};
      return *(float*)&ctn;
    }              
  }
}

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    uint8_t buf[C2M_PACKET_SIZE];       /* message raw buffer */

    float MotorVolt_L;                     /* left foc angle */
    float MotorVolt_R;                    /* right foc angle */
} C2Mraw_t;

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    uint8_t buf[S2M_PACKET_SIZE];       /* message raw buffer */

    uint8_t debug_code;                 /* debug code */
    float foc_left;                     /* left foc angle */
    float foc_right;                    /* right foc angle */

} S2Mraw_t;

/* check start bit & start retrieving data from serial port
* args:   uint8_t *Rx_buffer      IO    Rx Buffer for storing
          int &num_bytes          IO    number of bytes to be updated
          const int uart_fd_      I     serial port
*/
bool check_start(const int uart_fd_);

/* Get raw data from Rx buffer
* args:     raw_t *raw              IO  receiver raw data control struct -> raw will be updated
*           uint8_t *Rx_buffer      I   stream data (1 byte)
*           int num_bytes           I   number of bytes received    
* return:   status (-1: error message, 0: no message, 1: input data successfully)
*/
int serial_input(S2Mraw_t *raw, uint8_t *Rx_buffer, const int num_bytes);
int serial_input(C2Mraw_t *raw, uint8_t *Rx_buffer, const int num_bytes);
#endif