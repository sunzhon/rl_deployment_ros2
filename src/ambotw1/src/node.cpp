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
	this->declare_parameter<std::string>("cyber_device", "/dev/ttyACM0");
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
	devices.resize(3);
	this->get_parameter("motor_device", devices[0]);
	this->get_parameter("imu_device", devices[1]);
	this->get_parameter("cyber_device", devices[2]);
	this->init_robot(devices);
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
	motor_cmds = std::make_shared<ambot_msgs::msg::Action>();
	motor_fdbk = std::make_shared<ambot_msgs::msg::State>();
}

void Robot::init_robot(std::vector<std::string> devices){

	// open unitree motor device
	if(!devices[0].empty()){
		try {
			if(access(devices[0].c_str(),0)!=F_OK){
				throw std::runtime_error("Error: " + devices[0] + " does not exist.");
			}
			motors = std::make_shared<SerialPort>(devices[0].c_str());
			std::cout <<  "Sucessfully open motor device"<<  std::endl;

			// scan motors
			// /*
			//  Do something to scan all connected motors 
			//
			// */

			motor_cmds->motor_num=0;
			for(uint8_t id=1;id<20;id++){
				MotorCmd cmd;
				MotorData data;
				data.motorType = MotorType::GO_M8010_6;
				cmd.motorType = MotorType::GO_M8010_6;
				cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
				cmd.id = id;
				motors->sendRecv(&cmd, &data);
				if(data.motor_id==id){
					motor_cmds->motor_num++;
				}else{
					break;
				}
			}

			motor_cmds->motor_action.resize(motor_cmds->motor_num);
			motor_fdbk->motor_num = motor_cmds->motor_num;
			motor_fdbk->motor_state.resize(motor_cmds->motor_num);

			printf("Found %i motors\n",motor_cmds->motor_num);

		} catch (const std::exception& e) {
			std::cerr << e.what() << std::endl;
		}
	}else{
		std::cout <<  "do not open motor device"<<  std::endl;
	}

	// open imu device
	if(!devices[1].empty()){
		imu = std::make_shared<yesense::YesenseDriver>(devices[1],460800);
	}else{
		std::cout <<  "do not open imu device"<<  std::endl;
	}

	// open cyber motor device
	if(!devices[2].empty()){
		//imu = std::make_shared<yesense::YesenseDriver>(devices[1],460800);
		std::vector<int> ids;
		ids.resize(1);
		ids[0]=0x7f;
		//cybergear = std::make_shared<CyberGearCan>(devices[2], 115200, ids);
	//move(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel){
	//cybergear->move(ids[0], 1.0,  0.02,  0.0, 0.5, 0.0);
	//sleep(3);// move 3 seconds
	//cybergear->stop();

	}else{
		std::cout <<  "do not open cyber device"<<  std::endl;
	}

}

void Robot::move_motor(const std::shared_ptr<ambot_msgs::msg::Action>& action,  std::shared_ptr<ambot_msgs::msg::State>& state)
{

	//1) assert
	MotorCmd cmd;
	MotorData data;
	assert(motor_num==action->motor_num);

	//2) interpret joint action to be motor commands
	get_motor_cmds(action, motor_cmds);

	//3) send motor cmds and get motor feedback
	for(uint8_t idx=0;idx<motor_cmds->motor_num;idx++){
		//fill and send cmd to motors
		data.motorType = MotorType::GO_M8010_6;
		cmd.motorType = MotorType::GO_M8010_6;
		cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
		if(motor_cmds->motor_action[idx].id>0){
			cmd.id   = motor_cmds->motor_action[idx].id;
			cmd.kp   = motor_cmds->motor_action[idx].kp; 
			cmd.kd   = motor_cmds->motor_action[idx].kd;
			cmd.q    = motor_cmds->motor_action[idx].q * queryGearRatio(MotorType::GO_M8010_6);
			cmd.dq   = motor_cmds->motor_action[idx].dq * queryGearRatio(MotorType::GO_M8010_6);
			cmd.tau  = motor_cmds->motor_action[idx].tau/queryGearRatio(MotorType::GO_M8010_6);
		}else{
			cmd.id = idx+1;
			cmd.kp = 0.0;
			cmd.kd = -0.05;
			cmd.q = 0.0;
			cmd.dq = 0.0;
			cmd.tau = 0.0;
		}
		//std::cout <<  "cmd.id: "<<  cmd.id <<std::endl;
		//std::cout <<  "cmd.q: "<<  cmd.q<<std::endl;
		//std::cout <<  "cmd.dq: "<<  cmd.dq<<std::endl;
		//std::cout <<  "cmd.kp: "<<  cmd.kp<<std::endl;
		//std::cout <<  "cmd.kd: "<<  cmd.kd<<std::endl;
		//std::cout <<  "cmd.tau: "<<  cmd.tau<<std::endl;
		//std::cout <<  "cmd.mode: "<<  cmd.mode<<std::endl;

		motors->sendRecv(&cmd, &data);

		// fetch motor feedbacks
		motor_fdbk->motor_state[idx].id = data.motor_id;
		motor_fdbk->motor_state[idx].q = data.q/queryGearRatio(MotorType::GO_M8010_6);
		motor_fdbk->motor_state[idx].dq = data.dq/queryGearRatio(MotorType::GO_M8010_6);
		motor_fdbk->motor_state[idx].tau = data.tau*queryGearRatio(MotorType::GO_M8010_6);
		motor_fdbk->motor_state[idx].temp = data.temp;
		motor_fdbk->motor_state[idx].merror = data.merror;
		motor_enabled = true;


	}

	//4) get joint state
	get_joint_state(motor_fdbk, state);
}

void Robot::get_motor_cmds(const std::shared_ptr<ambot_msgs::msg::Action>& action,  std::shared_ptr<ambot_msgs::msg::Action>& motor_cmds){

	for(uint8_t idx=0;idx<motor_cmds->motor_num;idx++){
		motor_cmds->motor_action[idx].id = action->motor_action[idx].id;
		motor_cmds->motor_action[idx].q = action->motor_action[idx].q;
		motor_cmds->motor_action[idx].dq = action->motor_action[idx].dq;
		motor_cmds->motor_action[idx].kp = 0.01*action->motor_action[idx].kp; //NOTE
		motor_cmds->motor_action[idx].kd = 0.1*action->motor_action[idx].kd;
		motor_cmds->motor_action[idx].tau = action->motor_action[idx].tau;
	}

}

void Robot::get_joint_state(const std::shared_ptr<ambot_msgs::msg::State>& motor_fdbk,  std::shared_ptr<ambot_msgs::msg::State>& state){

	for(uint8_t idx=0;idx<motor_cmds->motor_num;idx++){
		state->motor_state[idx].id = motor_fdbk->motor_state[idx].id;
		state->motor_state[idx].q = motor_fdbk->motor_state[idx].q;
		state->motor_state[idx].dq = motor_fdbk->motor_state[idx].dq;
		state->motor_state[idx].tau = motor_fdbk->motor_state[idx].tau; 
		state->motor_state[idx].temp = motor_fdbk->motor_state[idx].temp;
		state->motor_state[idx].merror = motor_fdbk->motor_state[idx].merror;
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
	for(uint8_t idx=0; idx<motor_cmds->motor_num;idx++){
		cmd.motorType = MotorType::GO_M8010_6;
		data.motorType = MotorType::GO_M8010_6;
		cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::BRAKE);
		cmd.id   = idx+1; // NOTE: this should be changed 
		cmd.kp = 0;
		cmd.kd = 0;
		motors->sendRecv(&cmd, &data);
	}

	std::cout <<  " stop motors " <<  std::endl;

}



