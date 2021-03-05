


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

float kp_x_=(0.6+0.2)*1000,ki_x_=0,kd_x_=150,prev_error_x_=0,total_error_x_=0;
float kp_theta_=0.025,ki_theta_=0.00000,kd_theta_=0.002,prev_error_theta_=0,total_error_theta_=0;  //kptheta=0.025
float kp_y_=(0.6+0.2)*1000,ki_y_=0,kd_y_=150,prev_error_y_=0,total_error_y_=0;

float speed_value=0.75;


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

    //////////////////////////////////////////////////////////////////////////
	//float raw_pid_theta=(kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_))*3.14159/180
    /////////////////////////////////////////////////////////////

    float raw_pid_theta=(kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_));
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
        raw_pid_x=min(float(speed_value*1000),raw_pid_x);

    }
    else{
        raw_pid_x=max(float(-speed_value*1000),raw_pid_x);
    }
    
    return raw_pid_x;
}


float PIDY(float goal_y){
    float error_y=goal_y;//-odom_.position.y;

    float raw_pid_y=kp_y_*error_y+ki_y_*error_y+kd_y_*(error_y-prev_error_y_);
    raw_pid_y=raw_pid_y;
    if (raw_pid_y>0){
        raw_pid_y=min(float(speed_value*1000),raw_pid_y);

    }
    else{
        raw_pid_y=max(float(-speed_value*1000),raw_pid_y);
    }
    
    return raw_pid_y;
}


vector<double> Interpolator(double x, double y){
    double max_value=speed_value;

    auto e_current=x;
    auto e_current_1=y;
    // auto e_current_2=theta;
    double speed_x, speed_y;
    // speed_theta;


    if (abs(e_current) >= abs(e_current_1)     )
    {
    speed_x=(max_value*e_current/abs(e_current));
    speed_y=(max_value*abs(e_current_1)/abs(e_current)*e_current_1/abs(e_current_1));
    // speed_theta=(max_value*abs(e_current_2)/abs(e_current)*e_current_2/abs(e_current_2));

    }

    else if (abs(e_current_1) >= abs(e_current)   )
        {//ROS_INFO_STREAM("22222222");
         
         speed_y=(max_value*e_current_1/abs(e_current_1));
         speed_x=(max_value*abs(e_current)/abs(e_current_1)*e_current/abs(e_current));
         // speed_theta=(max_value*abs(e_current_2)/abs(e_current_1)*e_current_2/abs(e_current_2));
        }

    // else if (abs(e_current_2) >= abs(e_current)   and  abs(e_current_2) >= abs(e_current_1))
    //     {
    //      //ROS_INFO_STREAM("3333333");
    //      //std::cout<<"errors "<<e_current<<" "<<e_current_1<<" "<<e_current_2<<" "<<max_value<<std::endl;

    //      speed_theta=(max_value*e_current_2/abs(e_current_2));
    //      speed_x=(max_value*abs(e_current)/abs(e_current_2)*e_current/abs(e_current));
    //      speed_y=(max_value*abs(e_current_1)/abs(e_current_2)*e_current_1/abs(e_current_1));
    //     }           

    return {speed_x,speed_y};
}

double EulerDistance(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2){
	return double(sqrt(pow((p2.x - p1.x),2) + pow((p2.y - p1.y),2)));
}


int main(int argc, char **argv)
{   
    

    cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"left_controller_following");
    ros::NodeHandle n_("~");     /////~ for arguments
    // goal_pose_in_origin.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;

    odom_broadcaster=new tf2_ros::TransformBroadcaster();
    tf::TransformListener transformListener;


    goal_in_origin_2d_dash.theta+=180;

    // ros::Subscriber pose_subscriber_ = n_.subscribe("/goal_pose", 10000, GoalPoseCallback);
    ros::Publisher lc_pose2d_publisher = n_.advertise<geometry_msgs::Pose2D>("/planner_goal",10);
    ros::spinOnce();

    
    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z=0;


    geometry_msgs::Pose2D prev_lc_pose;

    

    while(n_.ok()){

        tf::StampedTransform camera_tf_transform_;

        transformListener.waitForTransform("/origin", "/left_controller", ros::Time(0),
                                                 ros::Duration(3));

        transformListener.lookupTransform("/origin", "/left_controller", ros::Time(0),
                                        camera_tf_transform_);


        transformListener.waitForTransform("/origin", "/robot_frame", ros::Time(0),
                                                 ros::Duration(3));

        transformListener.lookupTransform("/origin", "/robot_frame", ros::Time(0),
                                        camera_tf_transform_);




        // ROS_INFO_STREAM("Get 2d in robot");
        // auto goal2dinRobot=getPose2DInRobot(transformListener);         /// point in pose2d where we want to be angle in 0 360

        // ROS_INFO_STREAM("get in frame ");
        auto robotpose=getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
                                            "/robot_frame", "/origin");        
                                    //Pose of robot in origin




        auto left_controller_pose=getInFrame(transformListener,MakeGeometryMsgsPose(1,0,0, 0,0,0,1),
                                            "/left_controller", "/origin");


        auto lc_euler=QuaterniontoEuler(left_controller_pose.orientation);


        auto lcpose2d=MakeGeometryMsgsPose2D(left_controller_pose.position.x,left_controller_pose.position.y,lc_euler[2]);  

        lcpose2d.theta+=180;

        ROS_INFO_STREAM(lcpose2d);


        if(EulerDistance(lcpose2d,prev_lc_pose)>0.2){
            lc_pose2d_publisher.publish(lcpose2d);
            prev_lc_pose=lcpose2d;
        }

        
        ros::spinOnce();
        ros::Duration(1.0).sleep();


    }


}
