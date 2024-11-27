// ambotw1 robot node code
// author: Tao Sun
// Date: 2024-11-14
//
//
#include "node.hpp"
using namespace std::chrono_literals;
//using std::placeholders::_1;
using namespace yesense;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

RobotRosNode::RobotRosNode(): Node("robot_ros_node")
{

	// publishe robot state of robot
	state_publisher = this->create_publisher<ambot_msgs::msg::State>("state", 1);

	// subscribe action from controller
	rclcpp::QoS qos_profile(1);
	qos_profile.reliable(); 
	action_subscription = this->create_subscription<ambot_msgs::msg::Action>("action", qos_profile, std::bind(&RobotRosNode::action_callback, this, std::placeholders::_1));

	// timer tasks of action and state transition
	action_timer = this->create_wall_timer(5ms, std::bind(&RobotRosNode::action_timer_callback, this));
	state_timer = this->create_wall_timer(5ms, std::bind(&RobotRosNode::state_timer_callback, this));
	sensor_timer = this->create_wall_timer(5ms, std::bind(&RobotRosNode::sensor_timer_callback, this));

	// rosparameter
	this->declare_parameter<std::string>("motor_device", "/dev/M1080");
	this->declare_parameter<std::string>("imu_device", "/dev/YIS106");
	this->declare_parameter<int>("motor_num", 12);


	action_ptr = std::make_shared<ambot_msgs::msg::Action>();
	state_ptr = std::make_shared<ambot_msgs::msg::State>();

	// init state and action msg buffer
	this->get_parameter("motor_num", this->motor_num);
	action_ptr->motor_num = this->motor_num;
	action_ptr->motor_action.resize(this->motor_num);
	state_ptr->motor_num = this->motor_num;
	state_ptr->motor_state.resize(this->motor_num);

	// open motor and imu devices
	devices.resize(2);
	this->get_parameter("motor_device", devices[0]);
	this->get_parameter("imu_device", devices[1]);
	this->init_robot(devices[0], devices[1]);
}


void RobotRosNode::state_timer_callback() // for peroidic publish data
{

	// publish robot state
	if(this->motor_enabled && this->imu_enabled){
		state_publisher->publish(*state_ptr);
		// for testing
		RCLCPP_DEBUG(this->get_logger(), "Publishing state");
	}
}

void RobotRosNode::sensor_timer_callback() // for peroidic publish data
{

	this->imu->fetchImuData();
	RCLCPP_DEBUG(this->get_logger(), "fetch imu data");
}

void RobotRosNode::action_timer_callback() // for peroidic publish data
{

	this->move_motor(action_ptr, state_ptr);
	this->read_imu(state_ptr);
	RCLCPP_DEBUG(this->get_logger(), "Moving motors");
}


void RobotRosNode::action_callback(ambot_msgs::msg::Action::SharedPtr data) const{
	//std::cout<<motor_num<<std::endl;
	assert(this->motor_num==data->motor_num);
	for (int i = 0; i < data->motor_num; i++){
		action_ptr->motor_action[i].id = data->motor_action[i].id;
		action_ptr->motor_action[i].mode = data->motor_action[i].mode;
		action_ptr->motor_action[i].q = data->motor_action[i].q;
		action_ptr->motor_action[i].dq = data->motor_action[i].dq;
		action_ptr->motor_action[i].tau = data->motor_action[i].tau;
		action_ptr->motor_action[i].kp = data->motor_action[i].kp;
		action_ptr->motor_action[i].kd = data->motor_action[i].kd;
	}
	RCLCPP_DEBUG(this->get_logger(), "receiving action");
}


// Robot class
Robot::Robot(void){
	motor_enabled = false;
	imu_enabled = false;
}

void Robot::init_robot(std::string motor_device, std::string imu_device){
	
	// open motor device
	if(!motor_device.empty()){
		try {
			if(access(motor_device.c_str(),0)!=F_OK){
				throw std::runtime_error("Error: " + motor_device + " does not exist.");
			}
			motor_port = std::make_shared<SerialPort>(motor_device.c_str());
			std::cout <<  "Sucessfully open motor device"<<  std::endl;
		} catch (const std::exception& e) {
			std::cerr << e.what() << std::endl;
		}
	}else{
		std::cout <<  "do not open motor device"<<  std::endl;
	}

	// open imu device
	if(!imu_device.empty()){
		imu = std::make_shared<yesense::YesenseDriver>(imu_device,460800);
	}else{
		std::cout <<  "do not open imu device"<<  std::endl;
	}
}

void Robot::move_motor(const std::shared_ptr<ambot_msgs::msg::Action>& action,  std::shared_ptr<ambot_msgs::msg::State>& state)
{

	MotorCmd cmd;
	MotorData data;
	assert(motor_num==action->motor_num);

	std::cout <<  " only enable a motor"<<std::endl;
	//NOTE. replace this
	//for(uint8_t idx=0;idx<action->motor_num;idx++){
	for(uint8_t idx=0;idx<1;idx++){
		//fill and send cmd to motors
		data.motorType = MotorType::GO_M8010_6;
		cmd.motorType = MotorType::GO_M8010_6;
		cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
		if(action->motor_action[idx].id>0){
			cmd.id   = action->motor_action[idx].id;
			cmd.kp   = 0.01*action->motor_action[idx].kp; // NOTE
			cmd.kd   = 0.1*action->motor_action[idx].kd;
			cmd.q    = action->motor_action[idx].q * queryGearRatio(MotorType::GO_M8010_6);
			cmd.dq   = action->motor_action[idx].dq * queryGearRatio(MotorType::GO_M8010_6);
			cmd.tau  = action->motor_action[idx].tau/queryGearRatio(MotorType::GO_M8010_6);
		}else{
			cmd.id = idx+1;
			cmd.kp = 0.0;
			cmd.kd = -0.05;
			cmd.q = 0.0;
			cmd.dq = 0.0;
			cmd.tau = 0.0;
		}
		std::cout <<  "cmd.id: "<<  cmd.id <<std::endl;
		std::cout <<  "cmd.q: "<<  cmd.q<<std::endl;
		std::cout <<  "cmd.dq: "<<  cmd.dq<<std::endl;
		std::cout <<  "cmd.kp: "<<  cmd.kp<<std::endl;
		std::cout <<  "cmd.kd: "<<  cmd.kd<<std::endl;
		std::cout <<  "cmd.tau: "<<  cmd.tau<<std::endl;
		std::cout <<  "cmd.mode: "<<  cmd.mode<<std::endl;

		motor_port->sendRecv(&cmd, &data);

		// fetch motor feedbacks
		state->motor_state[idx].id = data.motor_id;
		state->motor_state[idx].q = data.q/queryGearRatio(MotorType::GO_M8010_6);
		state->motor_state[idx].dq = data.dq/queryGearRatio(MotorType::GO_M8010_6);
		state->motor_state[idx].tau = data.tau*queryGearRatio(MotorType::GO_M8010_6);
		state->motor_state[idx].temp = data.temp;
		state->motor_state[idx].merror = data.merror;
		motor_enabled = true;
	}
}




void Robot::read_imu(std::shared_ptr<ambot_msgs::msg::State>& state){

	// process imu data
	imu->processImuData();

	// read imu data
	imu_data = imu->readImuData();

	// fill imu state msg
	state->imu_state.gyroscope.x = imu_data.angle_x;
	state->imu_state.gyroscope.y = imu_data.angle_y;
	state->imu_state.gyroscope.z = imu_data.angle_z;

	state->imu_state.acceleration.x = imu_data.accel_x;
	state->imu_state.acceleration.y = imu_data.accel_y;
	state->imu_state.acceleration.z = imu_data.accel_z;

	state->imu_state.quaternion.w= imu_data.quaternion_data0;
	state->imu_state.quaternion.x= imu_data.quaternion_data1;
	state->imu_state.quaternion.y= imu_data.quaternion_data2;
	state->imu_state.quaternion.z= imu_data.quaternion_data3;
	imu_enabled = true;
}





Robot::~Robot(void){

	// disable motors
	MotorCmd cmd;
	MotorData data;
	for(uint8_t idx=0; idx<motor_num;idx++){
		cmd.motorType = MotorType::GO_M8010_6;
		data.motorType = MotorType::GO_M8010_6;
		cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::BRAKE);
		cmd.id   = idx+1; // NOTE: this should be changed 
		cmd.kp = 0;
		cmd.kd = 0;
		motor_port->sendRecv(&cmd, &data);
	}

	std::cout <<  " stop motors " <<  std::endl;

}



