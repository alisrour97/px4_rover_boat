#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPublisher : public rclcpp::Node {
public:
	
    TrajectoryPublisher();
	void arm();
	void disarm();
	void takeoff();

private:

    void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void load_trajectory();
    void publisher();

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	float takeoff_altitude_;
	float takeoff_yaw_;

    int traj_cnt_, offboard_setpoint_counter_;
    TrajectorySetpoint traj_sp_{};

    int trajectory_setpoints_;
    Eigen::VectorXf traj_time_, des_yaw_, des_yaw_rate_;
	Eigen::Matrix<float, Eigen::Dynamic, 3> des_pos_, des_vel_, des_acc_;

	
};