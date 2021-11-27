#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include "std_msgs/msg/string.hpp"
#include <string>
#include <cstdlib>


#define _USE_MATH_DEFINES

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;
using namespace rclcpp;
using namespace std;

double x_coord, y_coord, z_coord = 0.0;
double error_x, error_y, error_z = 0.0;
double gyro_1, gyro_2, gyro_3 = 0.0;
bool isQPBallDetected, goToDeliverySpot, isLandingSpotDetected, BoxDelivered, ReturnHome, LandCommand = false;
bool isBoxDetected, BoxTaken = false;
int control_box, control_ls, i, j = 0;
double actual_height, actual_x, actual_y = 0.0;


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);

#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		trajectory_setpoint_publisher_ =
		 	this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");

#endif

		//publishers and subscribers initialization

		subscription_z = this->create_subscription<std_msgs::msg::String>("z_coord", 1, std::bind(&OffboardControl::topic_callback_z_coord, this, _1)); 
		subscription_x = this->create_subscription<std_msgs::msg::String>("x_coord", 1, std::bind(&OffboardControl::topic_callback_x_coord, this, _1));
		subscription_y = this->create_subscription<std_msgs::msg::String>("y_coord", 1, std::bind(&OffboardControl::topic_callback_y_coord, this, _1));
		subscription_box_detected = this->create_subscription<std_msgs::msg::String>("isBoxDetected", 1, std::bind(&OffboardControl::topic_callback_is_box_detected, this, _1));
		subscription_landing_spot_detected = this->create_subscription<std_msgs::msg::String>("isLandingSpotDetected", 1, std::bind(&OffboardControl::topic_callback_is_landing_spot_detected, this, _1));
		subscription_qp_ball_detected = this->create_subscription<std_msgs::msg::String>("qp_BallDetected", 1, std::bind(&OffboardControl::topic_callback_qp_ball_detected, this, _1));

		sensor_subscription = this->create_subscription<px4_msgs::msg::SensorCombined>(
			"SensorCombined_PubSubTopic",
		#ifdef ROS_DEFAULT_API
            10,
		#endif
			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			gyro_1 = msg -> gyro_rad[0];
			gyro_2 = msg -> gyro_rad[1];
			gyro_3 = msg -> gyro_rad[2];
		});

		odometry_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"VehicleOdometry_PubSubTopic",
		#ifdef ROS_DEFAULT_API
            10,
		#endif
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			actual_height = msg -> z;
			//actual_x = abs(roundf(msg -> x * 10)/10);
			//actual_y = abs(roundf(msg -> y * 10)/10);
		});

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			//instructs the drone to come back to its home position
			else if (ReturnHome == true){
				this -> publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 1.0);
				ReturnHome = false;
				cout << "Returning home..." << endl;
			}


			// offboard_control_mode needs to be paired with trajectory_setpoint
				publish_offboard_control_mode();
				publish_trajectory_setpoint();

				 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				 offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(1000ms, timer_callback);
	}

	void arm() const;
	void disarm() const;
	void land() const;


	//callback functions that receive data about the presence or the position of the box, the landing spot and the balls from the vision node
	private:
    	void topic_callback_z_coord(const std_msgs::msg::String::SharedPtr msg) const
    	{
		  z_coord = stod(msg->data);
    	}


	private:
    	void topic_callback_x_coord(const std_msgs::msg::String::SharedPtr msg) const
    	{
		  x_coord = stod(msg->data);
    	}
    	

	private:
    	void topic_callback_y_coord(const std_msgs::msg::String::SharedPtr msg) const
    	{
		  y_coord = stod(msg->data);
    	}
    	

	private:
    	void topic_callback_is_box_detected(const std_msgs::msg::String::SharedPtr msg) const
    	{
			  if (strcmp(msg -> data.c_str(), "true") == 0 && BoxTaken == false){
				  isBoxDetected = true;
			  }

			  else {
				  isBoxDetected = false;
			  }
			
    	}


	private:
	void topic_callback_is_landing_spot_detected(const std_msgs::msg::String::SharedPtr msg) const
	{
		if (strcmp(msg -> data.c_str(), "true") == 0){
				  isLandingSpotDetected = true;
		}

	}

	private:
		void topic_callback_qp_ball_detected(const std_msgs::msg::String::SharedPtr msg) const
		{
			if (stoi(msg->data) == 1){
				isQPBallDetected = true;
			}
		}

//Publishers and Subscribers definition
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_x;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_y;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_z;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_box_detected;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_landing_spot_detected;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_qp_ball_detected;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
};

//this function arms the drone
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

//this function disarms the drone
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

//settings for the offboard control mode
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = true;

	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() const {

// The position where the drone moves is relative to where it takes off, so all positions are relative to the spawn point, which will have coordinates 0 0 0
//msg.x is considered with the minus sign because the reference system is flipped

	TrajectorySetpoint msg{};

	//definition of the most important points to reach
	double home_x = 0.0;
	double home_y = 0.0;
	double home_z = -7.0;

	double delivery_x = 10.0;
	double delivery_y = -87.0;
	double delivery_z = -18.0;
	double delivery_yaw = -1.8; //in radians

	//CASE 1: the drone starts to fly and looks for the box
	if (isBoxDetected == false && BoxTaken == false){

		msg.timestamp = timestamp_.load();
		msg.x = home_x;
		msg.y = home_y;
		msg.z = home_z; //the drone takes off to 7 meters until the box is detected 
		cout << "I'm looking for the box..." << endl;

		trajectory_setpoint_publisher_->publish(msg);
		
	}


	//CASE 2: the drone finds the box
	else if ((control_box++)<1 && isBoxDetected == true) { 
		//for getting the first distance point to adjust the position from, the drone doesn't move anymore until that point is reached
		msg.timestamp = timestamp_.load();
		msg.x = -x_coord;
		error_x = -x_coord;
		msg.y = y_coord;
		error_y = y_coord;
		msg.z = -5.0; //the drone's altitude is locked to 5 m if the box is detected
		trajectory_setpoint_publisher_->publish(msg);
		cout << "I found the box!" << endl;
	}

	//CASE 3: the drone has altrady grabbed the box and goes to the delivery spot while avoiding obstacles
	else if(goToDeliverySpot == true && isLandingSpotDetected == false){

		double z = delivery_z;

		//the drone's height change if a "dangerous ball" is spotted. 
		if(isQPBallDetected == true){
			z = actual_height - 2.0;
			//When the new altitude (with a precision of 0.5 m) is reached,
			//the altitude's value is restored to the delivery spot's one unless another ball is spotted
			if (actual_height > z + 0.5){
				isQPBallDetected = false;
			}
			cout << "Ball detected, avoiding..." << endl;
		}

		//the value of z is rewritten according to the presence of a "dangerous ball"
		msg.timestamp = timestamp_.load();
		msg.z = z;
		msg.x = delivery_x;
		msg.y = delivery_y;
		msg.yaw = delivery_yaw;

		trajectory_setpoint_publisher_->publish(msg);
	}


	//CASE 4: the drone spots the delivery spot and gets the first distance coordinates where to start the alignment from
	else if(isLandingSpotDetected == true && (control_ls++)<1){
		//for getting the first distance point to adjust the position from, the drone doesn't move anymore until that point is reached
		msg.timestamp = timestamp_.load();
		msg.x = -x_coord + delivery_x;
		error_x = -x_coord + delivery_x;
		msg.y = y_coord + delivery_y;
		error_y = y_coord + delivery_y;
		msg.z = delivery_z;
		goToDeliverySpot = false;
		trajectory_setpoint_publisher_->publish(msg);
	}

	//CASE 5: the drone adjust its position dinamically to align with the delivery spot, finally it delivers the box
	else if(BoxDelivered == false && isLandingSpotDetected == true  && (gyro_1 < 0.005 && gyro_1 > -0.005) && (gyro_2 < 0.005 && gyro_2 > -0.005) && (gyro_3 < 0.005 && gyro_3 > -0.005)){
		//position correction if the drone is not oscillating too much

		error_z = delivery_z;
		if(x_coord < -0.08 || x_coord > 0.08 || y_coord < -0.08 || y_coord > 0.08){
			cout << "I'm aligning with the delivery spot..." << endl;
			//the drone keeps aligning until the error for both x and y coordinates is below 8 cm.
			if(x_coord < -0.08 || x_coord > 0.08){
				error_x = error_x - x_coord;
			}
			if (y_coord < -0.08 || y_coord > 0.08){
				error_y = error_y + y_coord;
			}
		}

		//if the drone's horizontal position is aligned with the delivery spot, then the delivery phase takes places. This procedure is executed just once
		else if(x_coord > -0.08 && x_coord < 0.08 && y_coord > -0.08 && y_coord < 0.08 && z_coord > 0.7 && j == 0){
			error_z = actual_height + z_coord;
			j++;
			BoxDelivered = true;
			cout << "I'm deliverying the box..." << endl;
		}

		//the setpoint is given to the autopilot differently according to the simulation's phase (alignment or delivery)
		msg.timestamp = timestamp_.load();
		msg.z = error_z;
		msg.x = error_x;
		msg.y = error_y;

		trajectory_setpoint_publisher_->publish(msg);
		
	}

	//CASE 6: the drone delivers the box, hovers for 10 seconds over the box, then reaches another preparatory setpoint
	else if (BoxDelivered == true && j <= 11){
		
		msg.timestamp = timestamp_.load();

		//preparatory setpoint reached for just 1 second, the ReturnHome variable changes the drone's flight mode
		//from "Offboard" to "Return"
		if (j > 10){
			msg.z = delivery_z - 8.0;
			msg.x = delivery_x;
			msg.y = delivery_y; 
			ReturnHome = true;
			cout << "Preparing to return home..." << endl;
			j++;
		}

		else{
			msg.z = error_z;
			msg.x = error_x;
			msg.y = error_y; 
			cout << "I delivered the box!" << endl;
			j++;
		}
		trajectory_setpoint_publisher_->publish(msg);
	}

	//CASE 7: the drone has grabbed the box so it gets to a preparatory setpoint
	else if (BoxTaken == true && goToDeliverySpot == false && isLandingSpotDetected == false){

		msg.timestamp = timestamp_.load();

		if (i == 20){
			goToDeliverySpot = true;
		}

		else if(i > 10){
			//another preparatory setpoint
			msg.x = 0.0;
			msg.y = 0.0;
			msg.z = -13.0; 
			cout << "I'm looking for the delivery spot..." << endl;
			i++;
		}

		else{
			msg.z = error_z;
			msg.x = error_x;
			msg.y = error_y; 
			cout << "I got the box!" << endl;
			i++;
		}

		trajectory_setpoint_publisher_->publish(msg);
	}

	//CASE 8: the drone adjust its position dinamically to align with the box, finally it grabs it
	else if(isBoxDetected == true  && (gyro_1 < 0.005 && gyro_1 > -0.005) && (gyro_2 < 0.005 && gyro_2 > -0.005) && (gyro_3 < 0.005 && gyro_3 > -0.005)){

		//position correction if the drone is not oscillating too much

		error_z = -5.0;

		//the drone keeps aligning until the error for both x and y coordinates is below 8 cm.
		if(x_coord < -0.08 || x_coord > 0.08 || y_coord < -0.08 || y_coord > 0.08){
			cout << "I'm aligning with the box..." << endl;
			if(x_coord < -0.08 || x_coord > 0.08){
				error_x = error_x - x_coord;
			}
			if (y_coord < -0.08 || y_coord > 0.08){
				error_y = error_y + y_coord;
			}
		}

		else if(x_coord > -0.08 && x_coord < 0.08 && y_coord > -0.08 && y_coord < 0.08 && z_coord > 0.7 && i == 0){
			error_z = actual_height + z_coord;
			i++;
			BoxTaken = true;
			cout << "I'm approaching the box..." << endl;
		}

		//the setpoint is given to the autopilot differently according to the simulation's phase (alignment or grabbing)
		msg.timestamp = timestamp_.load();
		msg.z = error_z;
		msg.x = error_x;
		msg.y = error_y;

		trajectory_setpoint_publisher_->publish(msg);
		
	}

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
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
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}