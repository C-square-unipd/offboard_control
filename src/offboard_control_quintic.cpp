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
#include <vector>
#include <eigen3/Eigen/Dense>

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
		phase_ = 0;
		count_ = 1;

		debug_ = declare_parameter<bool>("debug", true);
		test_ = declare_parameter<int>("test", 0.0);
		MotionStart_ = declare_parameter<double>("MotionStart", 15.0);
		Tmotion_ = declare_parameter<double>("Tmotion", 10.0);
		Trest_ = declare_parameter<double>("Trest", 2.0);
		std::vector<double> StartingPoint = declare_parameter<std::vector<double>>("StartingPoint", {1.0,1.0,-1.0});
		StartingPoint_ = {StartingPoint[0], StartingPoint[1], StartingPoint[2]};
		yaw_ = declare_parameter<double>("yaw", -M_PI);
		deltaX_ = declare_parameter<double>("deltaX", 2.0);
		deltaY_ = declare_parameter<double>("deltaY", 2.0);
		deltaZ_ = declare_parameter<double>("deltaZ", 0.25);

		initTrajVars();
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
			if (offboard_setpoint_counter_ < landTime_/0.1){
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

				
					if (offboard_setpoint_counter_/10 == timePoints_(phase_))
					{
						if(phase_%2 == 0)
						{
							if(phase_ == timePoints_.size()-1)
								std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << " --- Landing Phase" << std::endl;
							else{
							std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << " --- Start Phase " << count_  <<std::endl;
							count_++;
							}
						}
						else
						{
							std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << " --- Rest" << std::endl;
						}
						if(phase_+1 < timePoints_.size())
							phase_++;
					}
					else
						std::cout << "Time: " << offboard_setpoint_counter_/10 << " s" << std::endl;				
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
	double landTime_;

	int phase_;
	int count_;
	bool debug_;
	int test_;
	double MotionStart_;
	double Tmotion_;
	double Trest_;
	Vector3d StartingPoint_;
	double yaw_;
	double deltaX_;
	double deltaY_;
	double deltaZ_;

	RowVectorXd generateQuinticCoeffs(VectorXd posPts, double finalTime);
	MatrixXd quinticPolyTraj(VectorXd timeLine);
	void initTrajVars();
	void initTrajVector();
	void publish_trajectory_setpoint(uint64_t counter);

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
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
 * @brief Helper function for coefficients generation
 * @param posPts	Starting and finishing positions
 * @param finalTime	Time required for motion 
 */
RowVectorXd OffboardControl::generateQuinticCoeffs(VectorXd posPts, double finalTime){

    double x0 = posPts(0);
    double dx0 = 0;
    double ddx0 = 0;

    double xT = posPts(1);
    double dxT = 0;
    double ddxT = 0;

    VectorXd coeffVec(6);
    coeffVec << x0, dx0, ddx0/2, 0, 0, 0;

    MatrixXd TMat0(3,3); 
    TMat0 << 1, finalTime, pow(finalTime,2), 0, 1, 2*finalTime, 0, 0, 2;

    Vector3d tmp;
    tmp << xT, dxT, ddxT;
    Vector3d B = tmp - TMat0*coeffVec(seq(0,2));

    Matrix3d invTMatF;
    invTMatF << 10/pow(finalTime,3),  -4/pow(finalTime,2),   1/(2*finalTime), -15/pow(finalTime,4),  7/pow(finalTime,3),  -1/pow(finalTime,2), 6/pow(finalTime,5), -3/pow(finalTime,4),  1/(2*pow(finalTime,3));
    coeffVec(seq(3,5)) = invTMatF * B;

    return coeffVec.transpose();
}

/**
 * @brief Creation of matrix composed by sampled position divided into cartesian components
 * @param timeLine	Collection of time points where the trajectory has to be computed
 */
MatrixXd OffboardControl::quinticPolyTraj(VectorXd timeLine){

    MatrixXd q(3,timeLine.size());

    for (int i=0; i < timeLine.size(); i++)
        {
        double t = timeLine(i);
        
        double finalTime;
        double timeVar;
        RowVectorXd coeff(6);

        for(int j=0; j < timePoints_.size()-1; j++){
            if (t < timePoints_(0))
                q.col(i) = wayPoints_.col(0);
            else if ((timePoints_(j) <= t) && (t < timePoints_(j+1))){
                finalTime = timePoints_(j+1) - timePoints_(j);
                timeVar = t - timePoints_(j);


                for (int k=0; k<3; k++){
                    coeff = generateQuinticCoeffs(wayPoints_(k,seq(j,j+1)), finalTime);
                    VectorXd varVect(6);
                    varVect << 1, timeVar, pow(timeVar,2), pow(timeVar,3), pow(timeVar,4), pow(timeVar,5);
                    q(k,i) = coeff*varVect;
                }
                }
            else if (t >= timePoints_(last))
                q.col(i) = wayPoints_.rightCols(1);   
            }
        }
    return q;
}

/**
 * @brief Initilize waypoints and timepoints
 */
void OffboardControl::initTrajVars() {

    switch(test_) 	// Horizontal square trajectory
    {
	case 0:
		{
        Vector3d vrt;
        Vector3d pointA = StartingPoint_;
        vrt = {-deltaX_, 0, 0};
        Vector3d pointB = pointA + vrt;
        vrt = {0, -deltaY_, 0};
        Vector3d pointC = pointB + vrt;
        vrt = {deltaX_, 0, 0};
        Vector3d pointD = pointC + vrt;
		Vector3d pointLand = pointA;
		pointLand[2] = 0.0;

        MatrixXd wp(3,11);
		wayPoints_ = wp;
        wayPoints_ << pointA, pointB, pointB, pointC, pointC, pointD, pointD, pointA, pointA, pointLand, pointLand;
		break;
		}
	case 1:	// Vertical steps trajectory
        {
		Vector3d vrt = {0, 0, -deltaZ_};
        Vector3d pointA = StartingPoint_;
        Vector3d pointB = pointA + vrt;
        Vector3d pointC = pointB + vrt;
        Vector3d pointD = pointC + vrt;
		Vector3d pointLand = pointA;
		pointLand[2] = 0.0;

        MatrixXd wp(3,15);
		wayPoints_ = wp;
        wayPoints_ << pointA, pointB, pointB, pointC, pointC, pointD, pointD, pointC, pointC, pointB, pointB, pointA, pointA, pointLand, pointLand;       
		break;
		}
	case 2: // Hovering trajectory
	    {
		Vector3d vrt = {0, 0, -deltaZ_};
		Vector3d pointA = StartingPoint_;
        Vector3d pointB = pointA + vrt;
		Vector3d pointLand = pointA;
		pointLand[2] = 0.0;

        MatrixXd wp(3,5);
		wayPoints_ = wp;
        wayPoints_ << pointA, pointB, pointB, pointLand, pointLand;       
		break;
		}
	}

    VectorXd tp(wayPoints_.cols());
	timePoints_ = tp;
    for (int i=0; i< timePoints_.size(); i++){
        if (i == 0)
                timePoints_(i) = MotionStart_;
        else if (i%2 == 1)
                timePoints_(i) = timePoints_(i-1) + Tmotion_;
        else
                timePoints_(i) = timePoints_(i-1) + Trest_;
        }

    landTime_    = timePoints_(last);

}

/**
 * @brief Initilize the trajectory
 */
void OffboardControl::initTrajVector() {

	TrajectorySetpoint setpoint;

	// Time vector creation
	double Ts = 0.1;
    int n = landTime_/Ts;
    double t = 0;
    VectorXd timeLine(n+1);
    for(int i = 0; i < timeLine.size(); i++){
        timeLine(i) = t;
        t = t + Ts;
    }

	// Trajectory vector creation
	MatrixXd q = quinticPolyTraj(timeLine);	

	traj_.clear();
	traj_.push_back(setpoint);

	for(int i= 0; i<q.cols();i++)
    {
		setpoint.position = {(float) q(0,i), (float) q(1,i), (float) q(2,i)};
		setpoint.yaw = yaw_; 

		traj_.push_back(setpoint);
    }
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