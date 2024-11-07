#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"
#include "serial_imu/msg/euler_angle.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#ifdef __cplusplus
extern "C"{
#endif

#include "ch_serial.h"

#define IMU_SERIAL  ("/dev/ttyUSB0")
#define IMU_SERIAL_2  ("/dev/ttyUSB1")
#define BAUD        (B115200)
#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)
#define BUF_SIZE    (1024)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static raw_t raw;

// create the node class "IMUPublisher" by inheriting from "rclcpp::Node"
class IMUPublisher : public rclcpp::Node
{
	public:
		int fd = 0;
		uint8_t buf[BUF_SIZE] = {0};
		// name the node as "IMU_publisher"
		IMUPublisher() : Node("IMU_publisher")	
		{
			fd = open_serial();
			// message type = Imu, topic name = "/Imu_data"
			imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/Imu_data", 20);
			imu_pub_euler_ = this->create_publisher<serial_imu::msg::EulerAngle>("/Imu_euler_angle", 20);
			// timer_callback function to be init every 2ms -> 500ms for testing purpose
			timer_ = this->create_wall_timer(500ms, std::bind(&IMUPublisher::timer_callback, this));
		}

		~IMUPublisher()
		{
			close(fd);
		}

	private: 
		// where the msg data is set and actually published
		void timer_callback()
		{
			// message is named "imu_data"
			auto imu_data = sensor_msgs::msg::Imu();
			auto imu_euler = serial_imu::msg::EulerAngle();
			int n = read(fd, buf, sizeof(buf));

			for(int i = 0; i < n; i++)
			{
				int rev = ch_serial_input(&raw, buf[i]);
				
				if(raw.item_code[raw.nitem_code - 1] != KItemGWSOL)
				{
					if(rev)
					{
						// for the to-be-pub message "imu_data"
						imu_data.orientation.w = raw.imu[raw.nimu - 1].quat[0];
						imu_data.orientation.x = raw.imu[raw.nimu - 1].quat[1];	
						imu_data.orientation.y = raw.imu[raw.nimu - 1].quat[2];
						imu_data.orientation.z = raw.imu[raw.nimu - 1].quat[3];
						imu_data.angular_velocity.x = raw.imu[raw.nimu - 1].gyr[0] * DEG_TO_RAD;
						imu_data.angular_velocity.y = raw.imu[raw.nimu - 1].gyr[1] * DEG_TO_RAD;
						imu_data.angular_velocity.z = raw.imu[raw.nimu - 1].gyr[2] * DEG_TO_RAD;
						imu_data.linear_acceleration.x = raw.imu[raw.nimu - 1].acc[0] * GRA_ACC;
						imu_data.linear_acceleration.y = raw.imu[raw.nimu - 1].acc[1] * GRA_ACC;
						imu_data.linear_acceleration.z = raw.imu[raw.nimu - 1].acc[2] * GRA_ACC;

						imu_euler.pitch_x 	= raw.imu[raw.nimu - 1].eul[0] * DEG_TO_RAD;
						imu_euler.roll_y 	= raw.imu[raw.nimu - 1].eul[1] * DEG_TO_RAD;
						imu_euler.yaw_z 	= raw.imu[raw.nimu - 1].eul[2] * DEG_TO_RAD;

						imu_data.header.stamp = rclcpp::Clock().now();
						imu_data.header.frame_id = "base_link";
						imu_pub_->publish(imu_data);
						imu_pub_euler_->publish(imu_euler);
					}
				}
			}

			// for preparing to receive the next data
			memset(buf,0,sizeof(buf));
		}

		int open_serial(void)
		{
			struct termios options;

			int fd = open(IMU_SERIAL, O_RDWR | O_NOCTTY);
			if(fd == -1)
			{
                std::cout   << "cannot open controller serial port: " 
                            << IMU_SERIAL << std::endl;

                fd = open(IMU_SERIAL_2, O_RDWR | O_NOCTTY);
                if (fd == -1)
                {
                    std::cout << "cannot open controller serial port: "
                              << IMU_SERIAL_2 << std::endl;
                    exit(0);    
                }
			}

			tcgetattr(fd,&options);
			
			if(fcntl(fd, F_SETFL, 0) < 0)
				cout << "fcntl failed" << "\n" << endl;
			else
				fcntl(fd, F_SETFL, 0);

			if(isatty(STDIN_FILENO) == 0)
				cout << "standard input is not a terminal device" << "\n" << endl;
			else
				cout << "isatty success!" << "\n" << endl;

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

		// declaration of timer & publisher
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
		rclcpp::Publisher<serial_imu::msg::EulerAngle>::SharedPtr imu_pub_euler_;
};


int main(int argc, const char * argv[])
{
	// init ROS2
	rclcpp::init(argc, argv);
	// starts processing data from the node, including callbacks from the timer
	rclcpp::spin(std::make_shared<IMUPublisher>());
	rclcpp::shutdown();

	return 0;
}
