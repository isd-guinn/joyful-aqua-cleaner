#define SMALL_ENDIAN
// #define BIG_ENDIAN

#include "uart_slave/MasterSerialProtocol.hpp"
#include "uart_slave/ControllerSerialProtocol.hpp"
#include "uart_slave/serial_receive.hpp"

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>

/*
bool check_start(const int uart_fd_)
{
    static uint8_t first_bit[1] = {0};
    static int n = 0;

    while (first_bit[0] != START_BIT)
        {
            // std::cout << "last first_bit = " << std::hex << first_bit[0] << std::endl;
            n = read(uart_fd_, first_bit, 1); // here n should = 1
            while (n != 1)
            {
                if (n == 0){
                    // std::cout << std::dec << uart_fd_ << std::endl;
                    std::cout << "Controller: No data available." << std::endl;
                }
                else {
                    std::cout << "Controller Error!!!!!!!" << std::endl;
                }
                return false; 
            }
        }
    return true;
}
// */

static int parse_data(S2Mraw_t *raw)
{
    raw->debug_code = raw->buf[BYTE_POS_S2M_DEBUGCODE];
    raw->foc_left = ByteUtil::reconFloat(raw->buf, BYTE_POS_S2M_LEFTFOCANGLE);
    raw->foc_right = ByteUtil::reconFloat(raw->buf, BYTE_POS_S2M_RIGHTFOCANGLE);
    return 1; // parsed successfully
}

static int parse_data(C2Mraw_t *raw)
{
    raw->MotorVolt_L = ByteUtil::reconFloat(raw->buf, BYTE_POS_C2M_LWHEELVOLTAGE);
    raw->MotorVolt_R = ByteUtil::reconFloat(raw->buf, BYTE_POS_C2M_RWHEELVOLTAGE);
    return 1; // parsed successfully
}

int serial_input(S2Mraw_t *raw, uint8_t *Rx_buffer, const int num_bytes)
{
    // store the bytes into the raw struct
    raw->buf[raw->nbyte++] = Rx_buffer[0]; // store the first byte into the raw struct's buffer
    for (int i = 1; i < num_bytes; i++){
        raw->buf[raw->nbyte++] = Rx_buffer[i];
    }
    // printf("nbyte = %d\n", raw->nbyte); // shd be equal to num_bytes aka S2M_PACKET_SIZE

    // check if the message is complete / corrupted
    if (raw->buf[raw->nbyte - 1] != END_BIT) // buf[13]
    {
        printf("End bit error\n");
        raw->nbyte = 0; // reset the nbyte
        return -1;
    }
    if (raw->nbyte != S2M_PACKET_SIZE)
    {
        printf("Length error\n");
        raw->nbyte = 0; // reset the nbyte
        return -1;
    }

    // check the checksum
    uint8_t checksum = 0;
    for (int i = 0; i < BYTE_POS_S2M_CHECKSUM; i++)
    { 
        checksum += raw->buf[i];
    }
    if (checksum != raw->buf[BYTE_POS_S2M_CHECKSUM])
    {
        printf("Checksum error\n");
        return -1;
    }

    raw->nbyte = 0; // reset the nbyte
    // start parsing the data
    return parse_data(raw);
}

int serial_input(C2Mraw_t *raw, uint8_t *Rx_buffer, const int num_bytes)
{
    // store the bytes into the raw struct
    raw->buf[raw->nbyte++] = Rx_buffer[0]; // store the first byte into the raw struct's buffer
    for (int i = 1; i < num_bytes; i++){
        raw->buf[raw->nbyte] = Rx_buffer[i];
        raw->nbyte++;
    }
    // printf("nbyte = %d\n", raw->nbyte); // shd be equal to num_bytes aka C2M_PACKET_SIZE

    // check if the message is complete / corrupted
    if (raw->buf[raw->nbyte - 1] != END_BIT) // buf[13]
    {
        printf("End bit error\n");
        raw->nbyte = 0; // reset the nbyte
        return -1;
    }
    if (raw->nbyte != C2M_PACKET_SIZE)
    {
        printf("Length error\n");
        raw->nbyte = 0; // reset the nbyte
        return -1;
    }

    // check the checksum
    uint8_t checksum = 0;
    for (int i = 0; i < BYTE_POS_C2M_CHECKSUM; i++)
    { 
        checksum += raw->buf[i];
    }
    if (checksum != raw->buf[BYTE_POS_C2M_CHECKSUM])
    {
        printf("Checksum error\n");
        return -1;
    }

    raw->nbyte = 0; // reset the nbyte
    // start parsing the data
    return parse_data(raw);
}