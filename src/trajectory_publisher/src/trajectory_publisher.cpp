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

class TrajPub : public rclcpp::Node {
public:
	TrajPub() : Node("trajectory_publisher") {

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 0);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 0);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 0);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};


/**
 * @brief Send a command to Arm the vehicle
 */
void TrajPub::arm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void TrajPub::disarm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        We will send position, velocity and acceleration
 */
void TrajPub::publish_offboard_control_mode(){
	OffboardControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void TrajPub::publish_trajectory_setpoint(){
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void TrajPub::publish_vehicle_command(uint16_t command, float param1,
					      float param2){
	VehicleCommand msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting trajectory_publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	Eigen::VectorXf traj_time, des_yaw, des_yaw_rate;
	Eigen::Matrix<float, Eigen::Dynamic, 3> des_pos, des_vel, des_acc;

	std::vector<std::vector<std::string>> content;
	std::vector<std::string> row;
	std::string line, word;
	std::fstream traj_file ("csv_file/drone_data.csv", std::ios::in);
	if(traj_file.is_open())
	{
		while(std::getline(traj_file, line))
		{
			row.clear();
 
			std::stringstream str(line);
 
			while(std::getline(str, word, ','))
				row.push_back(word);

			content.push_back(row);
		}
	}
	else
		std::cout<<"Could not open the file\n";
 
	// Avoid the first row of titles
	for(int i=1; i < int( content.size() ) ;i++)
	{
		traj_time.resize(i);
		des_pos.conservativeResize(des_pos.rows()+1, des_pos.cols());
		// des_vel.resize(i);
		// des_acc.resize(i);
		// des_yaw.resize(i);
		// des_yaw_rate.resize(i);

		traj_time(i-1) = (float)std::atof(content[i][0].c_str());
		des_pos.row(i-1) << (float)std::atof(content[i][1].c_str()), (float)std::atof(content[i][2].c_str()), (float)std::atof(content[i][3].c_str());
		// des_vel(i-1) = (float)std::atof(content[i][1].c_str());
		// des_acc(i-1) = (float)std::atof(content[i][1].c_str());
		// des_yaw(i-1) = (float)std::atof(content[i][1].c_str());

		std::cout << des_pos.row(i-1) << std::endl;
		
	}

	rclcpp::spin(std::make_shared<TrajPub>());

	rclcpp::shutdown();
	return 0;
}
