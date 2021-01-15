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


tf2_ros::TransformBroadcaster *odom_broadcaster;
geometry_msgs::Pose2D goal_in_origin_2d;
geometry_msgs::Pose2D goal_in_origin_2d_dash;

float kp_x_=0.6*1000,ki_x_=0,kd_x_=0,prev_error_x_=0,total_error_x_=0;
float kp_theta_=0.0080,ki_theta_=0.00000,kd_theta_=0.002,prev_error_theta_=0,total_error_theta_=0;
float kp_y_=0.6*1000,ki_y_=0,kd_y_=0,prev_error_y_=0,total_error_y_=0;


using namespace std;

geometry_msgs::Pose getInFrame(tf::TransformListener &transformListener,geometry_msgs::Pose pose,std::string pose_frame_id, std::string op_frame_id){

    // tf::TransformListener transformListener1;
    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    StampedPose_in.header.frame_id = pose_frame_id;
    
    StampedPose_in.pose = pose;
    //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    transformListener.transformPose(op_frame_id,StampedPose_in,StampedPose_out);


    // cout<<"ROBOT IN TRACKER "<<endl;
    // ROS_INFO_STREAM(StampedPose_out);
    

    return StampedPose_out.pose;
    
}


geometry_msgs::Pose MakeGeometryMsgsPose(double px,double py,double pz,double rx,double ry,double rz,double rw){
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


geometry_msgs::Pose2D MakeGeometryMsgsPose2D(double px,double py,double pt){
	geometry_msgs::Pose2D p;
	p.x=px;
	p.y=py;
	p.theta=pt;
	return p;
}


float wrapto360(double theta){
	
	while(theta>=360){
	if (theta>=360)
		theta=theta-360;
	}

	return theta;


}


void GoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& pose){
 

 goal_in_origin_2d=*pose;



 goal_in_origin_2d_dash=goal_in_origin_2d;
 goal_in_origin_2d_dash.theta+=180;

 goal_in_origin_2d_dash.theta=wrapto360(goal_in_origin_2d_dash.theta);

}

vector<float> QuaterniontoEuler(geometry_msgs::Quaternion odom_){
    vector<float> temp;
    geometry_msgs::Quaternion orientation=odom_;
    tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);   
    //ROS_INFO_STREAM(q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    temp.push_back(180+(roll*180/3.14159));
    temp.push_back(180+(pitch*180/3.14159));
    temp.push_back(180+(yaw*180/3.14159));                 ////////for conversion to 0-360
    return temp;
}


geometry_msgs::Pose2D getPose2DInRobot(tf::TransformListener &transformListener){
	
	tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, goal_in_origin_2d.theta );  // Create this quaternion from roll/pitch/yaw (in radians)
    myQuaternion.normalize();
    // ROS_INFO_STREAM("normalize");
    geometry_msgs::Quaternion geom_quat = tf2::toMsg(myQuaternion);

    // ROS_INFO_STREAM("tomsg");



	auto goal_pose_in_origin=MakeGeometryMsgsPose(goal_in_origin_2d.x,goal_in_origin_2d.y,0,geom_quat.x,geom_quat.y,geom_quat.z,geom_quat.w);

	auto goal_pose_in_robot=getInFrame(transformListener,goal_pose_in_origin,"/origin", "/robot_frame");
	// ROS_INFO_STREAM("getinfrmae");
	auto eul=QuaterniontoEuler(goal_pose_in_robot.orientation);
	// ROS_INFO_STREAM("QuaterniontoEuler");


	// ROS_INFO_STREAM(goal_pose_in_robot.position);
	// cout<<eul[0]<<" "<<eul[1]<<" "<<eul[2]<<endl;
	return MakeGeometryMsgsPose2D(goal_pose_in_robot.position.x,goal_pose_in_robot.position.y,eul[2]);
	
}






float PIDTheta(float goal_theta_,float euler_angle){
    float error_theta=goal_theta_-euler_angle;
    // if (abs(error_theta)>300){
    //     error_theta=(error_theta/abs(error_theta))*(360-abs(error_theta));
    // }

    float raw_pid_theta=kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_);
    raw_pid_theta=-raw_pid_theta;
    if (raw_pid_theta>0){
        raw_pid_theta=min(float(0.7),raw_pid_theta);

    }
    else{
        raw_pid_theta=max(float(-0.7),raw_pid_theta);
    }
    // if(abs(error_theta)>300){
    //     raw_pid_theta=-raw_pid_theta/2.5;
        
    // }
    total_error_theta_=total_error_theta_+error_theta;
    prev_error_theta_=error_theta;
    return raw_pid_theta;
}


float PIDX(float goal_x){
    float error_x=goal_x;//-odom_.position.x;

    float raw_pid_x=kp_x_*error_x+ki_x_*error_x+kd_x_*(error_x-prev_error_x_);
    raw_pid_x=raw_pid_x;
    if (raw_pid_x>0){
        raw_pid_x=min(float(0.5*1000),raw_pid_x);

    }
    else{
        raw_pid_x=max(float(-0.5*1000),raw_pid_x);
    }
    
    return raw_pid_x;
}


float PIDY(float goal_y){
    float error_y=goal_y;//-odom_.position.y;

    float raw_pid_y=kp_y_*error_y+ki_y_*error_y+kd_y_*(error_y-prev_error_y_);
    raw_pid_y=raw_pid_y;
    if (raw_pid_y>0){
        raw_pid_y=min(float(0.5*1000),raw_pid_y);

    }
    else{
        raw_pid_y=max(float(-0.5*1000),raw_pid_y);
    }
    
    return raw_pid_y;
}





int main(int argc, char **argv)
{   
    

	cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"follower");
    ros::NodeHandle n_("~");     /////~ for arguments
    // goal_pose_in_origin.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;

    odom_broadcaster=new tf2_ros::TransformBroadcaster();
    tf::TransformListener transformListener;


    goal_in_origin_2d_dash.theta+=180;

    ros::Subscriber pose_subscriber_ = n_.subscribe("/goal_pose", 10000, GoalPoseCallback);
    ros::Publisher velocity_publisher=n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::spinOnce();

    
    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z=0;


    

    while(n_.ok()){

    	tf::StampedTransform camera_tf_transform_;

	    transformListener.waitForTransform("/origin", "/robot_frame", ros::Time(0),
	                                             ros::Duration(3));

	    transformListener.lookupTransform("/origin", "/robot_frame", ros::Time(0),
	                                    camera_tf_transform_);




	    // ROS_INFO_STREAM("Get 2d in robot");
    	auto goal2dinRobot=getPose2DInRobot(transformListener);         /// point in pose2d where we want to be angle in 0 360

    	// ROS_INFO_STREAM("get in frame ");
    	auto robotpose=getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
    										"/robot_frame", "/origin");        
    								//Pose of robot in origin



    	// ROS_INFO_STREAM("EL   ");
    	auto el=QuaterniontoEuler(robotpose.orientation);


    	float raw_pid_theta;
        float raw_pid_x;
        float raw_pid_y;
        if(abs(goal_in_origin_2d_dash.theta-el[2])<10){
            raw_pid_theta=PIDTheta(goal_in_origin_2d_dash.theta,el[2]); 

            cmd_vel_msg.angular.z=-raw_pid_theta;

            raw_pid_x=PIDX(goal2dinRobot.x);
            cmd_vel_msg.linear.x=raw_pid_x;

            raw_pid_y=PIDY(goal2dinRobot.y);
            cmd_vel_msg.linear.y=raw_pid_y;
        }
        else{
            raw_pid_theta=PIDTheta(goal_in_origin_2d_dash.theta,el[2]);        
            cmd_vel_msg.angular.z=-raw_pid_theta;

            cmd_vel_msg.linear.y=0;
            cmd_vel_msg.linear.x=0;

        }

        
            // std::cout<<raw_pid_x<<" "<<raw_pid_y<<std::endl;
        

        // cout<<raw_pid_theta<<" "<<goal_in_origin_2d.theta<<" "<<el[2]<<endl; 
        // ROS_INFO_STREAM("error"<<goal_theta_-euler_angles[2],raw_pid_theta);//rpy
        // cout<<"error "<<goal_theta_-euler_angles[2]<<"raw_pid_theta "<<raw_pid_theta<<endl;
        cout<<"err x "<<goal_in_origin_2d.x-robotpose.position.x<<"  x  "<<raw_pid_x<<"  err y "<<goal_in_origin_2d.y-robotpose.position.y<<"  y  "<<raw_pid_y<<"  errtheta  "<<abs(goal_in_origin_2d.theta-el[2])<<"  theta "<<raw_pid_theta<<endl;
        // cout<<position_wrt_robot[0]<<"       "<<position_wrt_robot[1]<<"   "<<goal_theta_-euler_angles[2]<<endl;

        velocity_publisher.publish(cmd_vel_msg);


    	ros::spinOnce();

    }


}