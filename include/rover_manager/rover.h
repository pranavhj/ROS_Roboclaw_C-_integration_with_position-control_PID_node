#pragma once
#include "ros/ros.h"
#include "roboclaw/RoboclawMotorVelocity.h"
#include "roboclaw/RoboclawEncoderSteps.h"
#include "motor_controller/position.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <iostream>
#include "sensor_msgs/Joy.h"
#include "remote.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>



#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <typeinfo>
#include <fstream>
#include <tf/transform_listener.h>

#include <exception>

// #include "helper.h"
#include "../../src/tf_tools.cpp"
#include<cmath>

class Rover{

	private:

		float Kp1=1.5,  ki1=0.000000,  kd1=1,prev_e1=0,max_speed=1500,total_error_1=0;
		float Kp2=1.5,  ki2=0.000000, kd2=1,prev_e2=0,max_speed_2=1500,total_error_2=0;
		float Kp3=1.5,  ki3=0.000000,  kd3=1,prev_e3=0,max_speed_3=1500,total_error_3=0;

		ros::Publisher velocity_publisher;
		ros::Subscriber pose_subscriber;
		ros::Subscriber position_subscriber;
		
		ros::Subscriber cmd_vel_subscriber;

		
		float targets[3];
		float interpolation_speed=50;
		float prev_speed=0,prev_speed_1=0,prev_speed_2=0;
		float prev_x=0,prev_y=0,prev_theta=0;geometry_msgs::Pose2D prevState;
		ros::Publisher velocity_publisher_1;
		ros::Subscriber pose_subscriber_1;
		ros::Publisher DonePublisher;
		std_msgs::String DoneString;
		ros::Publisher ForwardKinematicsPositionPublisher;
		ros::Subscriber inv_kinematics_subscriber;
		ros::Publisher inv_kinematics_publisher;

		ros::Publisher reset_enc_publisher;

		roboclaw::RoboclawEncoderSteps pose;
		roboclaw::RoboclawEncoderSteps pose_1;
		motor_controller::position position;
		float full_speed=5000;
		geometry_msgs::Pose2D FKPose;
		ros::NodeHandle n;
		geometry_msgs::Pose2D IKpose;
		
		geometry_msgs::Twist cmd_vel;

		Remote *remote;

		float w1_prev=0;
		float w2_prev=0;
		float w3_prev=0;

		Eigen::MatrixXd B;
		Eigen::MatrixXd A;
		Eigen::Vector3d W_prev;

		Eigen::Vector3d Xn_1;


		Eigen::MatrixXd Pk_1;

		Eigen::MatrixXd H;
		Eigen::MatrixXd M;

		Eigen::MatrixXd Q;
		Eigen::MatrixXd R;

		Eigen::MatrixXd I3x3;

		Eigen::Vector3d Y;
		Eigen::Vector3d D;

		
		bool FKVROriginFrameflag=true;
		tf::TransformListener transformListener;
		double angle_correction_factor=90.0/115.0;


		TF *TF_;
		int kf_counter=0;

		geometry_msgs::Pose lastObsPose,prev_obs_pose;
		double last_FK_call=0;



	public:

	Rover(ros::NodeHandle *n);
	void poseCallback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message);
	void pose_1Callback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message);
	
	void positionCallback(const motor_controller::position::ConstPtr& position_message);
	void IKCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message);
	void CMDVELCallback(const geometry_msgs::Twist::ConstPtr& pose_message);
	
	
	void ExecuteCMDVEL();
	void SetRemote(Remote *c){remote=c;}



	void PIDcontroller(float goal1,float goal2,float goal3);
	void ForwardKinematics();
	void ExecuteIK();
	float* matrixCalculation(float x, float y, float w);
	void IK();
	geometry_msgs::Pose2D ConvertPosition(geometry_msgs::Pose2D remoteState);
	void GotoPosition(geometry_msgs::Pose2D remoteState);

	void FK();
	void InitializeMatrices();
	void KalmanFilter();

	// geometry_msgs::Pose getInFrame(geometry_msgs::Pose pose,std::string pose_frame_id, std::string op_frame_id);

	void FKVROriginFrame();




	

	geometry_msgs::Quaternion EulerToQuaternion(double x,double y,double z);


	double EulerDistance(geometry_msgs::Pose p1,geometry_msgs::Pose p2){

	return double(sqrt(pow((p1.position.x-p2.position.x),2) + pow((p1.position.y-p2.position.y),2) + pow((p1.position.z-p2.position.z),2)));

	}



	


	};
