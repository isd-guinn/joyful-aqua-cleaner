#include <unistd.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>

// create a dummy share pointer node from rclcpp::Node
rclcpp::Node::SharedPtr nh = nullptr;
using namespace std;

// receive msg data published over the topic
// then write to console using cout
void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

	cout << "header:" << "\n";
	// cout << "	" << "stamp:"<< "\n";
	cout << "	  " << "secs:" << msg->header.stamp.sec<< "\n";
	cout << "	  " << "nanosecs:" << msg->header.stamp.nanosec << "\n";
	// cout << "	" << "frame_id:" << msg->header.frame_id << "\n" ;

	cout << "orientation:";
	cout << "\n";
	cout << "  x: " << fixed << setprecision(18) << msg->orientation.x;
	// cout << "\n";
	cout << " y: " << fixed << setprecision(18) << msg->orientation.y;
	// cout << "\n";
	cout << " z: " << fixed << setprecision(18) << msg->orientation.z;
	// cout << "\n";
	cout << " w: " << fixed << setprecision(18) << msg->orientation.w << "\n";
	// cout  << "orientation_covariance: [ " << fixed << setprecision(1) << msg->orientation_covariance[0];
	// cout  << ", " << msg->orientation_covariance[1];
	// cout  << ", " << msg->orientation_covariance[2];
	// cout  << ", " << msg->orientation_covariance[3];
	// cout  << ", " << msg->orientation_covariance[4];
	// cout  << ", " << msg->orientation_covariance[5];
	// cout  << ", " << msg->orientation_covariance[6];
	// cout  << ", " << msg->orientation_covariance[7];
	// cout  << ", " << msg->orientation_covariance[8] << "]" << "\n";

	cout  << "angular_velocity: ";
	cout << "\n";
	cout  << "  x: " << fixed << setprecision(18) << msg->angular_velocity.x;
	// cout << "\n";
	cout  << " y: " << fixed << setprecision(18) << msg->angular_velocity.y;
	// cout << "\n";
	cout  << " z: " << fixed << setprecision(18) << msg->angular_velocity.z << "\n";
	// cout  << "angular_velocity_covariance: [ " << fixed << setprecision(1) << msg->angular_velocity_covariance[0];
	// cout  << ", " << msg->angular_velocity_covariance[1];
	// cout  << ", " << msg->angular_velocity_covariance[2];
	// cout  << ", " << msg->angular_velocity_covariance[3];
	// cout  << ", " << msg->angular_velocity_covariance[4];
	// cout  << ", " << msg->angular_velocity_covariance[5];
	// cout  << ", " << msg->angular_velocity_covariance[6];
	// cout  << ", " << msg->angular_velocity_covariance[7];
	// cout  << ", " << msg->angular_velocity_covariance[8] << "]" << "\n";

	cout  << "linear_acceleration:";
	cout << "\n";
	cout  << "  x: " << fixed << setprecision(18) << msg->linear_acceleration.x;
	// cout << "\n" ;
	cout  << " y: " << fixed << setprecision(18) << msg->linear_acceleration.y;
	// cout << "\n" ;
	cout  << " z: " << fixed << setprecision(18) << msg->linear_acceleration.z << "\n" ;
	// cout  << "linear_acceleration_covariance: [ " << fixed << setprecision(1) << msg->linear_acceleration_covariance[0];
	// cout  << ", " << msg->linear_acceleration_covariance[1] ;
	// cout  << ", " << msg->linear_acceleration_covariance[2] ;
	// cout  << ", " << msg->linear_acceleration_covariance[3] ;
	// cout  << ", " << msg->linear_acceleration_covariance[4] ;
	// cout  << ", " << msg->linear_acceleration_covariance[5] ;
	// cout  << ", " << msg->linear_acceleration_covariance[6] ;
	// cout  << ", " << msg->linear_acceleration_covariance[7] ;
	// cout  << ", " << msg->linear_acceleration_covariance[8] << "]" << "\n" << "---" << endl;
}


int main(int argc,const char* argv[])
{
	rclcpp::init(argc, argv);
	// create a node named "imu_sub" and link it to the share pointer node
	nh = std::make_shared<rclcpp::Node>("IMU_subscriber");
	// declare & init a subscription
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ ;
	imu_sub_ = nh->create_subscription<sensor_msgs::msg::Imu>("Imu_data", 10,topic_callback);
	// start processing data from the node
	rclcpp::spin(nh);
	rclcpp::shutdown();

	return 0;
}
