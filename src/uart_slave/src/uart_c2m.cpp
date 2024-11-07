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

#include "uart_slave/ControllerSerialProtocol.hpp"
#include "uart_slave/serial_receive.hpp"
#include "uart_slave/msg/foc_angle.hpp"

#ifdef __cplusplus
extern "C"{
#endif
#define BAUD                (B115200)
#define CONTROLLER_SERIAL   ("/dev/ttyACM0")
#define CONTROLLER_SERIAL_2 ("/dev/ttyACM1")
#define DEG_TO_RAD          (0.01745329)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static C2Mraw_t raw; // struct for storing the raw data from the serial port

class UartControllerReceiver : public rclcpp::Node
{
public:
    int uart_fd_ = 0;
    uint8_t Rx_buffer[C2M_PACKET_SIZE] = {0};
    int freq_adjust = 1;

    UartControllerReceiver()
        : Node("Uart_receiver_controller")
    {
        uart_fd_ = open_serial();
        uart_pub_motorvoltage_ = this->create_publisher<uart_slave::msg::FocAngle>("/Motor_voltage", 10);
        // same frequency required?
        timer_ = this->create_wall_timer(100ms, std::bind(&UartControllerReceiver::timer_callback, this));
    }
    ~UartControllerReceiver()
    {
        if (uart_fd_ != -1)
        {
            close(uart_fd_);
        }
    }

private:
    rclcpp::Publisher<uart_slave::msg::FocAngle>::SharedPtr uart_pub_motorvoltage_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int num_bytes;

    void timer_callback(){
        auto motor_voltage = uart_slave::msg::FocAngle();
        raw.nbyte = 0;
        num_bytes = 0;

        // /*
        uint8_t first_bit[1] = {0};

        // check the bytes one by one until get the start bit
        while (first_bit[0] != START_BIT)
        {
            // std::cout << "last first_bit = " << std::hex << first_bit[0] << std::endl;
            num_bytes = read(uart_fd_, first_bit, 1); // here num_bytes should = 1
            while (num_bytes != 1)
            {
                if (num_bytes == 0){
                    // std::cout << std::dec << uart_fd_ << std::endl;
                    std::cout << "Controller: No data available." << std::endl;
                }
                else {
                    std::cout << "Controller Error!!!!!!!" << std::endl;
                }
                return; 
            }
        }
        Rx_buffer[0] = first_bit[0];
        // */

        /*
        if (check_start(uart_fd_) == true){
            num_bytes = 1;
            Rx_buffer[0] = START_BIT;
        }
        else return;
        // */

        // read the rest of the data
        num_bytes += read(uart_fd_, &Rx_buffer[1], C2M_PACKET_SIZE-1);
        // std::cout << std::dec << "num_bytes after read: " << num_bytes << std::endl;

        // store the raw data into the raw struct
        int rev = serial_input(&raw, Rx_buffer, num_bytes);
        
        // rev = 1 means the data is successfully decoded
        if (rev){
            // successfully decoded the data
            motor_voltage.left = raw.MotorVolt_L;
            motor_voltage.right = raw.MotorVolt_R;
            uart_pub_motorvoltage_->publish(motor_voltage);

            // std::cout << "Controller data received: ";
            // for (int i=0; i < C2M_PACKET_SIZE; i++)
            // {
            //     std::cout << std::hex << static_cast<int>(Rx_buffer[i]) << " ";
            // }
            // std::cout << std::endl;

            // std::cout << "Controller Decoded Data: " << std::endl;
            std::cout << "motor voltage = " << raw.MotorVolt_L << " " << raw.MotorVolt_R << std::endl;
        }
        else {
            std::cout << "No data from Controller." << std::endl;
        }
        std::cout << "-------------------------------------" << std::endl;

        // for preparing to receive the next data
		memset(Rx_buffer,0,sizeof(Rx_buffer));
    }

    int open_serial(void)
		{
			struct termios options;

			int fd;
            fd = open(CONTROLLER_SERIAL, O_RDWR | O_NOCTTY);
			if(fd == -1)
			{
                std::cout   << "cannot open controller serial port: " 
                            << CONTROLLER_SERIAL << std::endl;

                fd = open(CONTROLLER_SERIAL_2, O_RDWR | O_NOCTTY);
                if (fd == -1)
                {
                    std::cout << "cannot open controller serial port: "
                              << CONTROLLER_SERIAL_2 << std::endl;
                    exit(0);    
                }
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
	rclcpp::spin(std::make_shared<UartControllerReceiver>());
	rclcpp::shutdown();
	return 0;
}