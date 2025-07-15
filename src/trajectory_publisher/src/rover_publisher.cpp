#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using std::placeholders::_1;
using px4_msgs::msg::VehicleCommand;

class RoverManualControl : public rclcpp::Node
{
public:
    RoverManualControl() : Node("rover_manual_control")
    {
        manual_control_pub_ = this->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
            "/fmu/in/manual_control_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            std::bind(&RoverManualControl::status_callback, this, _1));

        is_armed_ = false;
        in_offboard_ = false;

        publisher();  // start the control loop
    }

private:
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    // Subscriber
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    bool is_armed_;
    bool in_offboard_;

    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        is_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        in_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arming command sent");
    }

    void disarm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }

    void set_offboard_mode()
    {
        publish_offboard_control_mode();  // Always publish control mode first
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        publish_offboard_control_mode();  // Immediately again after command
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        // msg.position = false;
        // msg.velocity = false;
        // msg.acceleration = false;
        // msg.attitude = false;
        // msg.body_rate = false;
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;  // Enable actuator control if needed

        offboard_control_mode_pub_->publish(msg);
    }

    void send_trajectory_setpoint()
    {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position = {5.0, 5.0, 0.0};  // Go forward 1m in NED
    msg.velocity = {2.0, 2.0, 0.0};
    msg.acceleration = {0.0, 0.0, 0.0};
    msg.jerk = {0.0, 0.0, 0.0};
    msg.yaw = 1.0;
    msg.yawspeed = 0.5;

    trajectory_setpoint_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent trajectory setpoint");
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void send_manual_control_setpoint()
    {
        px4_msgs::msg::ManualControlSetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.pitch = 0.7f;
        msg.roll = 0.0f;
        msg.yaw = 0.30f;
        msg.throttle = 1.0f;

        manual_control_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent manual control command (pitch=0.7)");
    }

    void publisher()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() -> void
            {
                publish_offboard_control_mode();  // Always keep publishing it at high rate

                if (!is_armed_) {
                    arm();
                    return;
                }

                if (!in_offboard_) {
                    set_offboard_mode();
                    return;
                }

                // send_manual_control_setpoint();
                publish_offboard_control_mode();
                send_trajectory_setpoint();
            });
    }
};

// Main
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverManualControl>());
    rclcpp::shutdown();
    return 0;
}
