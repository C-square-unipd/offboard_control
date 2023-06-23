/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "spline.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace Eigen;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;
		debug_ = declare_parameter<bool>("debug", true);
		TTakeOff_ = declare_parameter<double>("TTakeOff", 15.0);
		TLand_ = declare_parameter<double>("TLand", 10.0);
		TimePoints_ = declare_parameter<std::vector<double>>("TimePoints", {0.0});
		PointsX_ = declare_parameter<std::vector<double>>("PointsX", {0.0});
		PointsY_ = declare_parameter<std::vector<double>>("PointsY", {0.0});
		PointsZ_ = declare_parameter<std::vector<double>>("PointsZ", {0.0});

		initTrajVector();

		auto timer_callback = [this]() -> void {

			// auto begin = std::chrono::high_resolution_clock::now();

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			if (offboard_setpoint_counter_ < (TimePoints_.back()+TLand_)/0.1){
				publish_offboard_control_mode();
				publish_trajectory_setpoint(offboard_setpoint_counter_);
			}

			// Publish land command
			// if (offboard_setpoint_counter_ == landTime_/0.1)
			// 	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, traj_[traj_.size()-1].yaw);				
					
			if(debug_)
			{
				if((offboard_setpoint_counter_%10 == 0) && (offboard_setpoint_counter_ != 0))
				{

					if (offboard_setpoint_counter_/10 == TimePoints_[0]){
						std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << " --- Start Motion" << std::endl;
					} else if (offboard_setpoint_counter_/10 == TimePoints_.back()){
						std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << " --- Landing Phase" << std::endl;
					}else{
						std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << std::endl;	
					}			
				}	
			}
			
			offboard_setpoint_counter_++;

   			// auto end = std::chrono::high_resolution_clock::now();
    		// auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    
    		// printf("Time measured: %.5f seconds.\n", elapsed.count() * 1e-9);


		};

		// Exectues periodically the callback function
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	// END OF THE CONSTRUCTOR

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	std::vector<TrajectorySetpoint> traj_ = {};
	MatrixXd wayPoints_;
    VectorXd timePoints_;	

	int count_;
	bool debug_;
	double TTakeOff_;
	double TLand_;
	std::vector<double> TimePoints_;
	std::vector<double> PointsX_;
	std::vector<double> PointsY_;
	std::vector<double> PointsZ_;

	void initTrajVector();
	void publish_trajectory_setpoint(uint64_t counter);

	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 */
void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2, float param3, float param4)  {
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	vehicle_command_publisher_->publish(msg);
}

// MY FUNCTIONS

/**
 * @brief Initilize the trajectory
 */
void OffboardControl::initTrajVector() {

	for (double& element : TimePoints_) {
        element += TTakeOff_;
    }

	std::vector<double> TimeLand = {TimePoints_.back(), TimePoints_.back()+TLand_/2.0, TimePoints_.back()+TLand_};
	std::vector<double> PointLand = {PointsZ_.back(), PointsZ_.back()/2.0, 0.0};

	tk::spline sx(TimePoints_,PointsX_,tk::spline::cspline,false,tk::spline::first_deriv,0.0, tk::spline::first_deriv,0.0);

	tk::spline sy(TimePoints_,PointsY_,tk::spline::cspline,false,tk::spline::first_deriv,0.0, tk::spline::first_deriv,0.0);

	tk::spline sz(TimePoints_,PointsZ_,tk::spline::cspline,false,tk::spline::first_deriv,0.0, tk::spline::first_deriv,0.0);

	tk::spline sLand(TimeLand,PointLand,tk::spline::cspline,false,tk::spline::first_deriv,0.0, tk::spline::first_deriv,0.0);
	

	// sx.set_boundary(tk::spline::first_deriv, 0.0,
    //                tk::spline::first_deriv, 0.0);
	
	// sy.set_boundary(tk::spline::first_deriv, 0.0,
    //                tk::spline::first_deriv, 0.0);

	// sz.set_boundary(tk::spline::first_deriv, 0.0,
    //                tk::spline::first_deriv, 0.0);

	// sLand.set_boundary(tk::spline::first_deriv, 0.0,
    //                tk::spline::first_deriv, 0.0);

	double Ts = 0.1;
	double x,y,z, xnew, ynew, yaw;
	TrajectorySetpoint setpoint;
	traj_.clear();
	traj_.push_back(setpoint);

	std::ofstream myfile("trajectory.txt");
	myfile << "data = [";
   	
	for(double t = 0.0; t<=TimePoints_.back()+TLand_; t=t+Ts){
		if(t < TimePoints_[0])
		{
			x = PointsX_[0];
			y = PointsY_[0];
			z = PointsZ_[0];
			yaw =atan2(sy(TimePoints_[0]+Ts)-sy(TimePoints_[0]),sx(TimePoints_[0]+Ts)-sx(TimePoints_[0]));
		}else if((TimePoints_[0] <= t) && (t < TimePoints_.back())){
			x = sx(t);
        	y = sy(t);
        	z = sz(t);

        	xnew = sx(t+Ts);
        	ynew = sy(t+Ts);
			yaw = atan2(ynew-y,xnew-x);
		}else{
			x = PointsX_.back();
			y = PointsY_.back();
			z = sLand(t);
		}

		setpoint.position = {(float) x, (float) y, (float) z};
		setpoint.yaw = (float) yaw;

		traj_.push_back(setpoint);

		//std::cout << t << " " << x << " " << y << " " << z << " " << yaw << ";" << std::endl;

		myfile << t << " " << x << " " << y << " " << z << " " << yaw << ";" << std::endl;
    }

   	myfile << "];\n\n";
	
	myfile << "TimePoints = [";
	for (double& element : TimePoints_) {
        myfile << " " << element;
    }
	myfile <<"];\n";

	myfile << "PointsX = [";
	for (double& element : PointsX_) {
        myfile << " " << element;
    }
	myfile <<"];\n";

	myfile << "PointsY = [";
	for (double& element : PointsY_) {
        myfile << " " << element;
    }
	myfile <<"];\n";

	myfile << "PointsZ = [";
	for (double& element : PointsZ_) {
        myfile << " " << element;
    }
	myfile <<"];\n";

	myfile.close();
}

/**
 * @brief Publish a trajectory setpoint message
 */
void OffboardControl::publish_trajectory_setpoint(uint64_t counter) {
 	
	TrajectorySetpoint msg{};

	if (counter < traj_.size()) {
 		msg = traj_[counter];
 	}
 	else if (traj_.size() > 0)
 	{
 		msg = traj_[traj_.size()-1];
 	}
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
 	trajectory_setpoint_publisher_->publish(msg);
}

//------------------------------//

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}