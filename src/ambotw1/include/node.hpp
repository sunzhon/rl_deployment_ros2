// ambotw1 robot node code
// author: Tao Sun
// Date: 2024-11-14
//
//
#include <chrono>
#include <memory>
#include <cassert>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ambot_msgs/msg/state.hpp"
#include "ambot_msgs/msg/action.hpp"

#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "can_usb.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include "yesense.hpp"


using namespace std::chrono_literals;
using namespace yesense;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


class Robot
{
	public:
		Robot();
		~Robot(void);
		void init_robot(std::vector<std::string> devices);
		void move_motor(const std::shared_ptr<ambot_msgs::msg::Action>& action, std::shared_ptr<ambot_msgs::msg::State>& state);
		void get_motor_cmds(const std::shared_ptr<ambot_msgs::msg::Action>& action, std::shared_ptr<ambot_msgs::msg::Action>& motor_cmds);
		void get_joint_state(const std::shared_ptr<ambot_msgs::msg::State>& motor_fdbk, std::shared_ptr<ambot_msgs::msg::State>& state);
		void read_imu(std::shared_ptr<ambot_msgs::msg::State>& state);
		bool motor_enabled;
		bool imu_enabled;
		int motor_num;

		std::shared_ptr<yesense::YesenseDriver> imu;
		std::shared_ptr<CyberGearCan> cybergear;


		std::vector<std::vector<float>> realrobot_joint_limits;
	std::vector<std::vector<float>> simrobot_joint_limits;
	std::vector<std::vector<float>> torealrobot_params;
	std::vector<std::vector<float>> tosimrobot_params;

	private:
		// motors and imu objects
		std::shared_ptr<SerialPort> motors;

		// motors
		std::shared_ptr<ambot_msgs::msg::State> motor_fdbk;
		std::shared_ptr<ambot_msgs::msg::Action> motor_cmds;
		// imu
		protocol_info_t imu_data;

};

class RobotRosNode : public rclcpp::Node, public Robot
{
	public:
		RobotRosNode();
		void set_state( ambot_msgs::msg::State data);
		void get_action();
	private:
		void state_timer_callback(); // for peroidic publish data
		void action_timer_callback(); // for peroidic publish data
		void sensor_timer_callback(); // for peroidic publish data
		void action_callback(ambot_msgs::msg::Action::SharedPtr data) const;
		rclcpp::TimerBase::SharedPtr action_timer;
		rclcpp::TimerBase::SharedPtr state_timer;
		rclcpp::TimerBase::SharedPtr sensor_timer;
		rclcpp::Publisher<ambot_msgs::msg::State>::SharedPtr state_publisher;
		rclcpp::Subscription<ambot_msgs::msg::Action>::SharedPtr action_subscription;

		std::shared_ptr<ambot_msgs::msg::State> state_ptr;
		std::shared_ptr<ambot_msgs::msg::Action> action_ptr;


		std::vector<std::string> devices;

};
