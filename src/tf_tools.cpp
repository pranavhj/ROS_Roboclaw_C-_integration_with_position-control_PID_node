#pragma once


#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

//#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include <algorithm>
#include <vector>
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
#include <Eigen/Dense>
#include <Eigen/Geometry>


// #include "helper.h"
using namespace std;

class TF{
private:

	tf2_ros::TransformBroadcaster *odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;

public:

	TF(){
		odom_broadcaster=new tf2_ros::TransformBroadcaster();
	}


	vector<float> QuaterniontoEuler(geometry_msgs::Pose odom_){   //in rad
	    vector<float> temp;
	    geometry_msgs::Quaternion orientation=odom_.orientation;
	    tf2::Quaternion q(
	    orientation.x,
	    orientation.y,
	    orientation.z,
	    orientation.w);   
	    //ROS_INFO_STREAM(q);
	    tf2::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
	    temp.push_back((roll));
	    temp.push_back((pitch));
	    temp.push_back((yaw));
	    return temp;
	}



	void publishFrame(geometry_msgs::Pose pose,string frame_id, std::string parent_frame_id){


		geometry_msgs::PoseStamped poseStamped;
		poseStamped.pose=pose;


	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = ros::Time::now();
	    odom_trans.header.frame_id = parent_frame_id;
	    odom_trans.child_frame_id = frame_id;

	    odom_trans.transform.translation.x = poseStamped.pose.position.x;
	    odom_trans.transform.translation.y = poseStamped.pose.position.y;
	    odom_trans.transform.translation.z = poseStamped.pose.position.z;
	    odom_trans.transform.rotation = poseStamped.pose.orientation;
	   
	       //send the transform
	    odom_broadcaster->sendTransform(odom_trans);

	    // tf_publisher_counter++;
	    // cout<<"tf  "<<tf_publisher_counter<<endl;


	}


	



	


	geometry_msgs::Pose getInFrame(tf::TransformListener &transformListener,geometry_msgs::Pose pose,std::string pose_frame_id, std::string op_frame_id){

	    tf::StampedTransform camera_tf_transform_;

	    transformListener.waitForTransform(pose_frame_id, op_frame_id, ros::Time(0),
	                                             ros::Duration(3));

	    transformListener.lookupTransform(pose_frame_id, op_frame_id, ros::Time(0),
	                                    camera_tf_transform_);	

	    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	    StampedPose_in.header.frame_id = pose_frame_id;
	    StampedPose_in.pose = pose;

	    transformListener.transformPose(op_frame_id,StampedPose_in,StampedPose_out);

	    return StampedPose_out.pose;
	    
	}






	geometry_msgs::Pose getFramePose(tf::TransformListener &transformListener,std::string pose_frame_id, std::string op_frame_id){

	    tf::StampedTransform camera_tf_transform_;

	    transformListener.waitForTransform(op_frame_id,pose_frame_id, ros::Time(0),
	                                             ros::Duration(3));

	    transformListener.lookupTransform(op_frame_id,pose_frame_id, ros::Time(0),
	                                    camera_tf_transform_);	

	    geometry_msgs::Pose pose;

	    pose.position.x=camera_tf_transform_.getOrigin().x();
	    pose.position.y=camera_tf_transform_.getOrigin().y();
	    pose.position.z=camera_tf_transform_.getOrigin().z();

	    pose.orientation.x=camera_tf_transform_.getRotation().x();
	    pose.orientation.y=camera_tf_transform_.getRotation().y();
	    pose.orientation.z=camera_tf_transform_.getRotation().z();
	    pose.orientation.w=camera_tf_transform_.getRotation().w();
	    return pose;
	}



	static geometry_msgs::Quaternion EulerToQuaternion(double x, double y, double  z ){
		tf2::Quaternion myQuaternion;
	    myQuaternion.setRPY( x, y, z );  // Create this quaternion from roll/pitch/yaw (in radians)
	    myQuaternion.normalize();
	    // ROS_INFO_STREAM(myQuaternion);
	    geometry_msgs::Quaternion geom_quat = tf2::toMsg(myQuaternion);

	    return geom_quat;
	}





	void PublishStaticTransform(std::string frame_id, std::string parent_frame_id,geometry_msgs::Pose pose){
		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	    geometry_msgs::TransformStamped static_transformStamped;

	    static_transformStamped.header.stamp = ros::Time::now();
	    static_transformStamped.header.frame_id = parent_frame_id;
	    static_transformStamped.child_frame_id = frame_id;
	    static_transformStamped.transform.translation.x = pose.position.x;
	    static_transformStamped.transform.translation.y = pose.position.y;
	    static_transformStamped.transform.translation.z = pose.position.z;


	    static_transformStamped.transform.rotation.x = pose.orientation.x;
	    static_transformStamped.transform.rotation.y = pose.orientation.y;
	    static_transformStamped.transform.rotation.z = pose.orientation.z;
	    static_transformStamped.transform.rotation.w = pose.orientation.w;
	    static_broadcaster.sendTransform(static_transformStamped);
	}




	static geometry_msgs::Pose MakeGeometryMsgsPose(double px,double py,double pz,double rx,double ry,double rz,double rw){
		geometry_msgs::Pose p;
		p.position.x=px;
		p.position.y=py;
		p.position.z=pz;

		p.orientation.x=rx;
		p.orientation.y=ry;
		p.orientation.z=rz;
		p.orientation.w=rw;
		return p;
	}


	static geometry_msgs::Pose2D MakeGeometryMsgsPose2D(double px,double py,double pt){
		geometry_msgs::Pose2D p;
		p.x=px;
		p.y=py;
		p.theta=pt;
		return p;
	}



	geometry_msgs::Pose ConvertVectorToPose(std::vector<double> v){  //input is x y theta, input in m

		auto quat=EulerToQuaternion(0,0,v[2]);
	    geometry_msgs :: Pose pose;
	    pose.position.x=v[0];pose.position.y=v[1];

	    pose.orientation=quat;
	    return pose;
	}




	geometry_msgs::Pose ConvertPose2DToPose(geometry_msgs::Pose2D p){
		auto quat=EulerToQuaternion(0,0,p.theta);
	    geometry_msgs :: Pose pose;
	    pose.position.x=p.x;pose.position.y=p.y;

	    pose.orientation=quat;
	    return pose;
	}




	Eigen::Vector3d PosetoEigenVector3d(geometry_msgs::Pose pose){
		Eigen::Vector3d Xn_1;
		Xn_1.resize(3);
		auto eul = QuaterniontoEuler(pose);// in rad
		Xn_1<<pose.position.x, pose.position.y , eul[2];


		return Xn_1;
	}




};




