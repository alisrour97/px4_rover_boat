#include "trajectory_publisher/trajectory_publisher.hpp"

TrajectoryPublisher::TrajectoryPublisher() : Node("trajectory_publisher") {

	offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 0);
	trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 0);
	vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 0);

	takeoff_altitude_ = -2.0;
	takeoff_yaw_ = 1.57;

	offboard_setpoint_counter_ = 0;
	traj_cnt_ = 0;

	load_trajectory();

	publisher();
}


/**
 * @brief Send a command to Arm the vehicle
 */
void TrajectoryPublisher::arm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void TrajectoryPublisher::disarm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        We will send position, velocity and acceleration
 */
void TrajectoryPublisher::publish_offboard_control_mode(){
	OffboardControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

void TrajectoryPublisher::takeoff(){
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, takeoff_altitude_};
	msg.yaw = takeoff_yaw_; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void TrajectoryPublisher::publish_vehicle_command(uint16_t command, float param1,
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

void TrajectoryPublisher::load_trajectory(){

	
	std::vector<std::vector<std::string>> content;
	std::vector<std::string> row;
	std::string line, word;
	std::fstream traj_file ("src/trajectory_publisher/csv_file/drone_data4.csv", std::ios::in);
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

		trajectory_setpoints_ = content.size();

		// Avoid the first row of titles
		for(int i=1; i < trajectory_setpoints_ ;i++)
		{
			traj_time_.conservativeResize(traj_time_.size()+1);
			des_pos_.conservativeResize(des_pos_.rows()+1, des_pos_.cols());
			des_vel_.conservativeResize(des_vel_.rows()+1, des_vel_.cols());
			des_acc_.conservativeResize(des_acc_.rows()+1, des_acc_.cols());
			des_yaw_.conservativeResize(des_yaw_.size()+1);
			des_yaw_rate_.conservativeResize(des_yaw_rate_.size()+1);

			traj_time_(i-1) = (float)std::atof(content[i][0].c_str());
			des_pos_.row(i-1) << (float)std::atof(content[i][1].c_str()), (float)std::atof(content[i][2].c_str()), (float)std::atof(content[i][3].c_str());
			des_yaw_(i-1) = (float)std::atof(content[i][4].c_str());
			des_vel_.row(i-1) << (float)std::atof(content[i][5].c_str()), (float)std::atof(content[i][6].c_str()), (float)std::atof(content[i][7].c_str());
			des_yaw_rate_(i-1) = (float)std::atof(content[i][8].c_str());
			des_acc_.row(i-1) << (float)std::atof(content[i][9].c_str()), (float)std::atof(content[i][10].c_str()), (float)std::atof(content[i][11].c_str());
			
			// std::cout << traj_time(i-1) << ") " << des_pos.row(i-1) << " " << des_yaw(i-1) << " " << des_vel.row(i-1) << std::endl;
		}
	}
	else{
		std::cout<<"Could not open the file\n";
		exit(0);
	}

}

void TrajectoryPublisher::publisher(){

	auto timer_callback = [this]() -> void {

		if (offboard_setpoint_counter_ == 10) {
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}
		else if (offboard_setpoint_counter_ >= 300){

			if (traj_cnt_ < trajectory_setpoints_-1){

				traj_sp_.position = {des_pos_(traj_cnt_,0), des_pos_(traj_cnt_,1), des_pos_(traj_cnt_,2) + takeoff_altitude_};
				traj_sp_.velocity = {des_vel_(traj_cnt_,0), des_vel_(traj_cnt_,1), des_vel_(traj_cnt_,2)};
				traj_sp_.acceleration = {des_acc_(traj_cnt_,0), des_acc_(traj_cnt_,1), des_acc_(traj_cnt_,2)};
				traj_sp_.yaw = des_yaw_(traj_cnt_) + takeoff_yaw_;
				traj_sp_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

				publish_offboard_control_mode();
				trajectory_setpoint_publisher_->publish(traj_sp_);

				traj_cnt_++;

			}
		}
		
		// stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 301) {
			offboard_setpoint_counter_++;

			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			publish_offboard_control_mode();
			this->takeoff();
		}
	};
	timer_ = this->create_wall_timer(20ms, timer_callback); // 50 Hz
}

int main(int argc, char* argv[]) {
	std::cout << "Starting trajectory_publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<TrajectoryPublisher>());

	rclcpp::shutdown();
	return 0;
}
