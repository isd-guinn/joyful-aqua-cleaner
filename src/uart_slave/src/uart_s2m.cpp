// logic to be updated

#include "rclcpp/rclcpp.hpp"

#include <unistd.h>   // File IO
#include <fcntl.h>    // File Control & Access Modes
#include <errno.h>
#include <termios.h>  // for terminal operation

#include <stdio.h>   
#include <cstdint> 
#include <chrono>
#include <memory>

#include <iostream>
#include <iomanip>

#define SMALL_ENDIAN
// #define BIG_ENDIAN

#include "uart_slave/MasterSerialProtocol.hpp"
#include "uart_slave/serial_receive.hpp"
#include "uart_slave/msg/foc_angle.hpp"

#ifdef __cplusplus
extern "C"{
#endif
#define BAUD                (B115200)
#define SLAVE_SERIAL     ("/dev/ttyAMA10") // if On-board UART: "/dev/ttyAMA10" equals to "/dev/serial0"
// #define SLAVE_SERIAL        ("/dev/ttyAMA0") // if GPIO UART: "/dev/ttyAMA0"
#define DEG_TO_RAD          (0.01745329)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static S2Mraw_t raw; // struct for storing the raw data from the serial port

class UartReceiver : public rclcpp::Node
{
public:
    int uart_fd_ = 0;
    uint8_t Rx_buffer[S2M_PACKET_SIZE] = {0};
    UartReceiver()
        : Node("Uart_receiver")
    {
        uart_fd_ = open_serial();
        uart_pub_focangle_ = this->create_publisher<uart_slave::msg::FocAngle>("/FOC_angle", 10);
        timer_ = this->create_wall_timer(18ms, std::bind(&UartReceiver::timer_callback, this));
    }
    ~UartReceiver()
    {
        if (uart_fd_ != -1)
        {
            close(uart_fd_);
        }
    }

private:
    rclcpp::Publisher<uart_slave::msg::FocAngle>::SharedPtr uart_pub_focangle_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int num_bytes;

    void timer_callback(){
        auto foc_angle = uart_slave::msg::FocAngle();
        raw.nbyte = 0;
        num_bytes = 0;
        
        uint8_t first_bit[1] = {0};
        std::cout << "Slave's uart_fd_ = " << uart_fd_ << std::endl;

        // if the serial is not connected at start, connect now:
        if (uart_fd_ == -1){
            std::cout << "Slave not connected. Reconnecting......" << std::endl;
            close(uart_fd_);
            uart_fd_ = open_serial();
            if (uart_fd_ == -1){
                std::cout << "Slave still not connected. Wait for next timer_callback." << std::endl;
                std::cout << "----------------------" << std::endl;
                return;
            }
        }

        // check the bytes one by one until get the start bit
        while (first_bit[0] != START_BIT)
        {
            // std::cout << "last first_bit = " << std::hex << first_bit[0] << std::endl;
            num_bytes = read(uart_fd_, first_bit, 1); // here num_bytes should = 1
            while (num_bytes != 1)
            {
                if (num_bytes == 0){
                    // std::cout << std::dec << uart_fd_ << std::endl;
                    std::cout << "Slave: No data available." << std::endl;
                }
                else {
                    std::cout << "Slave Error!!!!!!!" << std::endl;
                }
                return; 
            }
        }
        Rx_buffer[0] = first_bit[0];

        // read the rest of the data
        num_bytes += read(uart_fd_, &Rx_buffer[1], S2M_PACKET_SIZE-1);
        // std::cout << std::dec << "num_bytes after read: " << num_bytes << std::endl;

        // store the raw data into the raw struct
        int rev = serial_input(&raw, Rx_buffer, num_bytes);

        if (rev){
            // successfully decoded the data
            // still in Radian
            foc_angle.left = raw.foc_left;
            foc_angle.right = raw.foc_right;

            std::cout << "---------------------------" << std::endl;
            std::cout << "Slave Decoded Data: " << std::endl;
            std::cout << "foc angle = " << raw.foc_left << " " << raw.foc_right << std::endl;

            uart_pub_focangle_->publish(foc_angle);

            std::cout << "Slave data received: ";
            for (int i=0; i < S2M_PACKET_SIZE; i++)
            {
                std::cout << std::hex << static_cast<int>(Rx_buffer[i]) << " ";
            }
            std::cout << std::endl;
        }
        else {
            std::cout << "No data is received." << std::endl;
        }
        std::cout << "-------------------------------------" << std::endl;

        // for preparing to receive the next data
		memset(Rx_buffer,0,sizeof(Rx_buffer));
    }

    int open_serial(void)
		{
			struct termios options;

			int fd = open(SLAVE_SERIAL, O_RDWR | O_NOCTTY);
			if(fd == -1)
			{
				std::cout   << "cannot open slave serial port: " 
                            << SLAVE_SERIAL << std::endl;
				exit(0);
			}
			
            tcgetattr(uart_fd_, &options);
			memset(&options, 0, sizeof(options));

			options.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
			options.c_iflag = IGNPAR;
			options.c_oflag = 0;
			options.c_lflag = 0;
			options.c_cc[VTIME] = 0;
			options.c_cc[VMIN] = 0;
			tcflush(fd, TCIFLUSH);
			tcsetattr(fd, TCSANOW, &options);

			return fd;
		}
};

int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UartReceiver>());
	rclcpp::shutdown();
	return 0;
}