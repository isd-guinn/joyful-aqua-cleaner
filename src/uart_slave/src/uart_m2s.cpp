#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "std_msgs/msg/u_int8.hpp"

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
#include "serial_imu/msg/euler_angle.hpp"
#include "uart_slave/msg/foc_angle.hpp"

#ifdef __cplusplus
extern "C"{
#endif

#define BAUD                (B115200)
#define SLAVE_SERIAL     ("/dev/ttyAMA10") // if on-board UART: "/dev/ttyAMA10" equals to "/dev/serial0" - debug UART port
// #define SLAVE_SERIAL        ("/dev/ttyAMA0") // if GPIO UART: "/dev/ttyAMA0"
#define DEG_TO_RAD          (0.01745329)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;

// Store all the required states for updates the modules
// all angle in radian
struct RobotState
{ 
  bool v_estop;
  control_mode_t control_mode;
  float speed_target; // motor voltage Left
  float speed_current;
  float angle_target; // motor voltage Right
  float angle_current;  
  float angular_speed_target;
  float angular_speed_current;
  float vacuum_voltage;
  action_t action;
  bool  foc_engaged;
};

struct RobotState rs{ false, MANUAL_CONTROL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, STOP, false };

class UartPublisher : public rclcpp::Node
{
public:
    int uart_fd_ = 0;
    float temp_speed_current, temp_angle_current, temp_angular_speed_current = 0;
    float temp_motorvolt_left, temp_motorvolt_right = 0;
    action_t temp_action = STOP;

    UartPublisher()
        : Node("Uart_sender")
    {
        // uart_fd_ = open_serial();
        uart_sub_imuraw_ = this->create_subscription<sensor_msgs::msg::Imu>("Imu_data", 10, std::bind(&UartPublisher::imuraw_callback, this, std::placeholders::_1));
        // uart_sub_imuprocessed_ = this->create_subscription<???>("Imu_processed", 10, imuprocessed_callback);
        uart_sub_euler_ = this->create_subscription<serial_imu::msg::EulerAngle>("Imu_euler_angle", 10, std::bind(&UartPublisher::euler_callback, this, std::placeholders::_1));
        uart_sub_motorvolt_ = this->create_subscription<uart_slave::msg::FocAngle>("Motor_voltage", 10, std::bind(&UartPublisher::motorvolt_callback, this, std::placeholders::_1));
        uart_sub_action_ = this->create_subscription<std_msgs::msg::UInt8>("Robot_action", 10, std::bind(&UartPublisher::algo_callback, this, std::placeholders::_1));
        
        // send data to slave every 1s
        timer_ = this->create_wall_timer(18ms, std::bind(&UartPublisher::timer_callback, this));
    }

private:
  int open_serial(void)
  {
    struct termios options;

    // Open Serial Port
    int uart_fd_ = open(SLAVE_SERIAL, O_RDWR | O_NOCTTY);
    if (uart_fd_ == -1)
    {
      perror("unable to open serial port");
      exit(0);
    }

    tcgetattr(uart_fd_, &options);
    memset(&options, 0, sizeof(options));

    // Configure UART
    // baud rate | character size = 8 bits | ignore modem control lines | enable receiver
    options.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR; // ignore framing & parity errors
    options.c_oflag = 0; // no special output processing
    options.c_lflag = 0; // no special local processing (including echo)
    tcflush(uart_fd_, TCIFLUSH); // flush data that're received but not read
    tcsetattr(uart_fd_, TCSANOW, &options); // changes occur immediately

    return uart_fd_;
  }

  void imuraw_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    // using info of x for testing purpose
    temp_speed_current = msg->linear_acceleration.x;
    temp_angular_speed_current = msg->angular_velocity.x; // in radian
  }

  void euler_callback(const serial_imu::msg::EulerAngle::SharedPtr msg){
    temp_angle_current = msg->yaw_z; // in radian
  }

  void motorvolt_callback(const uart_slave::msg::FocAngle::SharedPtr msg){
    temp_motorvolt_left = msg->left; 
    temp_motorvolt_right = msg->right; 
  }

  void algo_callback(const std_msgs::msg::UInt8::SharedPtr msg){
    switch(msg->data)
    {
      case 0: temp_action = STOP; break;
      case 1: temp_action = FORWARD; break;
      case 2: temp_action = BACKWARD; break;
      case 3: temp_action = ANTI_CLOCKWISE; break;
      case 4: temp_action = CLOCKWISE; break;
      default: temp_action = STOP; break;
    }
  }

  void timer_callback()
  {
    uint8_t data[M2S_PACKET_SIZE];
    uart_fd_ = open_serial();

    // dummy data for testing
    // float dummy_speed_target = 12.0f;
    // float dummy_angle_target = 2.0f;
    float dummy_angular_speed_target = 4.0f;
    float dummy_v_pump = 3.0f;

    rs.speed_target = temp_motorvolt_left;
    rs.angle_target = temp_motorvolt_right;
    rs.angular_speed_target = dummy_angular_speed_target;
    rs.vacuum_voltage = dummy_v_pump;

    rs.speed_current = temp_speed_current;
    rs.angle_current = temp_angle_current;
    rs.angular_speed_current = temp_angular_speed_current;
    rs.action = temp_action;

    std::cout << "speed_target (LWheelVolt)= " << rs.speed_target << std::endl;
    // std::cout << "speed_current = " << rs.speed_current << std::endl;
    std::cout << "angle_target (RWheelVolt)= " << rs.angle_target << std::endl;
    // std::cout << "angle_current = " << rs.angle_current << std::endl;
    // std::cout << "angular_speed_target = " << rs.angular_speed_target << std::endl;
    // std::cout << "angular_speed_current = " << rs.angular_speed_current << std::endl;
    // std::cout << "vacuum_voltage = " << rs.vacuum_voltage << std::endl;
    // std::cout << "action = " << rs.action << std::endl;
    // std::cout << std::endl;

    // prepare the PACKET
    data[BYTE_POS_M2S_STARTBIT] = START_BIT;

    if (rs.v_estop == true) data[BYTE_POS_M2S_VESTOP] = V_ESTOP_EN_CODE;
    else data[BYTE_POS_M2S_VESTOP] = V_ESTOP_DIS_CODE;

    data[BYTE_POS_M2S_CONTROLMODE] = rs.control_mode; // fixed atm

    for (int i = BYTE_POS_M2S_TARGETSPEED; i < BYTE_POS_M2S_CURRENTSPEED; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target, i-BYTE_POS_M2S_TARGETSPEED);
    }

    for (int i = BYTE_POS_M2S_CURRENTSPEED; i < BYTE_POS_M2S_TARGETANGLE; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_current, i-BYTE_POS_M2S_CURRENTSPEED);
    }

    for (int i = BYTE_POS_M2S_TARGETANGLE; i < BYTE_POS_M2S_CURRENTANGLE; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_target, i-BYTE_POS_M2S_TARGETANGLE);
    }

    for (int i = BYTE_POS_M2S_CURRENTANGLE; i < BYTE_POS_M2S_TARANGSPEED; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_current, i-BYTE_POS_M2S_CURRENTANGLE);
    }

    for (int i = BYTE_POS_M2S_TARANGSPEED; i < BYTE_POS_M2S_CURANGSPEED; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angular_speed_target, i-BYTE_POS_M2S_TARANGSPEED);
    }

    for (int i = BYTE_POS_M2S_CURANGSPEED; i < BYTE_POS_M2S_VACUUMVOLTAGE; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angular_speed_current, i-BYTE_POS_M2S_CURANGSPEED);
    }

    for (int i = BYTE_POS_M2S_VACUUMVOLTAGE; i < BYTE_POS_M2S_FOCMODE; i++) {
      data[i] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.vacuum_voltage, i-BYTE_POS_M2S_VACUUMVOLTAGE);
    }

    data[BYTE_POS_M2S_ACTION] = rs.action;

    if (rs.foc_engaged == true) data[BYTE_POS_M2S_FOCMODE] = FOC_EN_CODE;
    else  data[BYTE_POS_M2S_FOCMODE] = FOC_DIS_CODE; // FOCMode

    uint8_t checksum = 0;
    // Calculate checksum
    for (int i = 0; i < BYTE_POS_M2S_CHECKSUM; i++) {
        checksum += data[i];
    }
    data[BYTE_POS_M2S_CHECKSUM] = checksum;

    data[BYTE_POS_M2S_ENDBIT] = END_BIT;

    // Write to Serial Port
    ssize_t bytes_written = write(uart_fd_, data, sizeof(data));

    if (bytes_written == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Wrote %ld bytes to serial port", bytes_written);
      // for debug: display the bytes
      // std::cout << "Bytes of the data sent: " << std::endl;
      for (int i = 0; i < M2S_PACKET_SIZE; i++)
      {
        std::cout << std::hex << static_cast<int>(data[i]) << " ";
      }
      std::cout << std::endl;
      // std::cout << "---------------------------------------" << std::endl;
    }
  }

  // void imuprocessed_callback(const ??? msg){
  //   // get current speed
  //   // get current angular speed
  // }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr uart_sub_imuraw_;
  rclcpp::Subscription<serial_imu::msg::EulerAngle>::SharedPtr uart_sub_euler_;
  rclcpp::Subscription<uart_slave::msg::FocAngle>::SharedPtr uart_sub_motorvolt_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr uart_sub_action_;
  // rclcpp::Subscription<???>::SharedPtr uart_sub_imuprocessed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartPublisher>());
  rclcpp::shutdown();
  return 0;
}