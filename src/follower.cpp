


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
#include "tf_tools.cpp"

tf2_ros::TransformBroadcaster *odom_broadcaster;
geometry_msgs::Pose2D goal_in_origin_2d;
geometry_msgs::Pose2D goal_in_origin_2d_dash;

float kp_x_=(0.6+0.2)*1000/5,ki_x_=0,kd_x_=450,prev_error_x_=0,total_error_x_=0;
float kp_theta_=0.025,ki_theta_=0.00000,kd_theta_=0.002,prev_error_theta_=0,total_error_theta_=0;  //kptheta=0.025
float kp_y_=(0.6+0.2)*1000/5,ki_y_=0,kd_y_=450,prev_error_y_=0,total_error_y_=0;

float speed_value=0.75;
TF *TF_;


using namespace std;

std::string frame_to_follow="/robot_frame_kf";




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




geometry_msgs::Pose2D getPose2DInRobot(tf::TransformListener &transformListener){
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, goal_in_origin_2d.theta );  // Create this quaternion from roll/pitch/yaw (in radians)
    myQuaternion.normalize();
    // ROS_INFO_STREAM("normalize");
    geometry_msgs::Quaternion geom_quat = tf2::toMsg(myQuaternion);

    // ROS_INFO_STREAM("tomsg");



    auto goal_pose_in_origin=MakeGeometryMsgsPose(goal_in_origin_2d.x,goal_in_origin_2d.y,0,geom_quat.x,geom_quat.y,geom_quat.z,geom_quat.w);

    auto goal_pose_in_robot=TF_->getInFrame(transformListener,goal_pose_in_origin,"/origin", frame_to_follow);
    // ROS_INFO_STREAM("getinfrmae");
    auto eul=TF_->QuaterniontoEuler(goal_pose_in_robot);
    // ROS_INFO_STREAM("QuaterniontoEuler");


    // ROS_INFO_STREAM(goal_pose_in_robot.position);
    // cout<<eul[0]<<" "<<eul[1]<<" "<<eul[2]<<endl;
    return MakeGeometryMsgsPose2D(goal_pose_in_robot.position.x,goal_pose_in_robot.position.y,180+(180/3.14159*eul[2]));
    
}






float PIDTheta(float goal_theta_,float euler_angle){
    float error_theta=goal_theta_-euler_angle;
    

    if (abs(error_theta)>300){///////////for avoiding reverse rotation when angle jumps from -180 to 180
        error_theta=-(error_theta/abs(error_theta))*(360-abs(error_theta));
    }

    //////////////////////////////////////////////////////////////////////////
	//float raw_pid_theta=(kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_))*3.14159/180
    /////////////////////////////////////////////////////////////

    float raw_pid_theta=(kp_theta_*error_theta+ki_theta_*error_theta+kd_theta_*(error_theta-prev_error_theta_));
    raw_pid_theta=-raw_pid_theta;
    if (raw_pid_theta>0){
        raw_pid_theta=min(float(speed_value*1.25),raw_pid_theta);

    }
    else{
        raw_pid_theta=max(float(-speed_value*1.25),raw_pid_theta);
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

             

    return {speed_x,speed_y};
}

double EulerDistance(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2){
	return double(sqrt(pow((p2.x - p1.x),2) + pow((p2.y - p1.y),2)));
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
    TF_=new TF();

    goal_in_origin_2d_dash.theta+=180;

    ros::Subscriber pose_subscriber_ = n_.subscribe("/goal_pose", 10000, GoalPoseCallback);
    ros::Publisher velocity_publisher=n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::spinOnce();

    
    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z=0;


    

    while(n_.ok()){

        tf::StampedTransform camera_tf_transform_;

        transformListener.waitForTransform("/origin", frame_to_follow, ros::Time(0),
                                                 ros::Duration(3));

        transformListener.lookupTransform("/origin", frame_to_follow, ros::Time(0),
                                        camera_tf_transform_);




        // ROS_INFO_STREAM("Get 2d in robot");
        auto goal2dinRobot=getPose2DInRobot(transformListener);         /// point in pose2d where we want to be angle in 0 360

        // ROS_INFO_STREAM("get in frame ");
        auto robotpose=TF_->getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
                                            frame_to_follow, "/origin");        
                                    //Pose of robot in origin





    
        


        // ROS_INFO_STREAM("EL   ");
        auto el=TF_->QuaterniontoEuler(robotpose);


        float raw_pid_theta;
        float raw_pid_x;
        float raw_pid_y;
        bool speed_loop=false;

        auto distance = EulerDistance(goal_in_origin_2d,MakeGeometryMsgsPose2D(robotpose.position.x,robotpose.position.y,0));

        if(distance >0.3){
        	speed_loop=true;
        	auto msg_val=Interpolator(goal2dinRobot.x,goal2dinRobot.y);
            raw_pid_theta=PIDTheta(goal_in_origin_2d_dash.theta,180+(180/3.14159*el[2])); 

            cmd_vel_msg.angular.z=-raw_pid_theta;

            // raw_pid_x=PIDX(goal2dinRobot.x);
            cmd_vel_msg.linear.x=msg_val[0]*1000.0;//raw_pid_x;

            // raw_pid_y=PIDY(goal2dinRobot.y);
            cmd_vel_msg.linear.y=msg_val[1]*1000.0;            //// because of mm input of cmd_vel
        }
        else{
            raw_pid_theta=PIDTheta(goal_in_origin_2d_dash.theta,180+(180/3.14159*el[2]));        
            cmd_vel_msg.angular.z=-raw_pid_theta;

            raw_pid_x=PIDX(goal2dinRobot.x);
            raw_pid_y=PIDY(goal2dinRobot.y);

            

            cmd_vel_msg.linear.x=raw_pid_x;
            cmd_vel_msg.linear.y=raw_pid_y;

        }

        
            // std::cout<<raw_pid_x<<" "<<raw_pid_y<<std::endl;
        

        // cout<<raw_pid_theta<<" "<<goal_in_origin_2d.theta<<" "<<el[2]<<endl; 
        // ROS_INFO_STREAM("error"<<goal_theta_-euler_angles[2],raw_pid_theta);//rpy
        // cout<<"error "<<goal_theta_-euler_angles[2]<<"raw_pid_theta "<<raw_pid_theta<<endl;
        cout<<"speed_loop:"<<distance<<" err x "<<goal_in_origin_2d.x-robotpose.position.x<<"  x  "<<raw_pid_x<<"  err y "<<goal_in_origin_2d.y-robotpose.position.y<<"  y  "<<raw_pid_y<<"  errtheta  "<<(goal_in_origin_2d_dash.theta-el[2])<<"  theta "<<raw_pid_theta<<endl;
        // cout<<position_wrt_robot[0]<<"       "<<position_wrt_robot[1]<<"   "<<goal_theta_-euler_angles[2]<<endl;

        velocity_publisher.publish(cmd_vel_msg);


        ros::spinOnce();

    }


}