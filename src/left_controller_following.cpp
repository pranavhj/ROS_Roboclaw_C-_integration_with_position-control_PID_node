


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
#include "geometry_msgs/PoseStamped.h"



#include "tf_tools.cpp"



tf2_ros::TransformBroadcaster *odom_broadcaster;
geometry_msgs::Pose2D goal_in_origin_2d;
geometry_msgs::Pose2D goal_in_origin_2d_dash;

float kp_x_=(0.6+0.2)*1000,ki_x_=0,kd_x_=150,prev_error_x_=0,total_error_x_=0;
float kp_theta_=0.025,ki_theta_=0.00000,kd_theta_=0.002,prev_error_theta_=0,total_error_theta_=0;  //kptheta=0.025
float kp_y_=(0.6+0.2)*1000,ki_y_=0,kd_y_=150,prev_error_y_=0,total_error_y_=0;

float speed_value=0.95;
std::string frame_to_follow="/robot_frame_kf";

geometry_msgs::PoseStamped obsPose;

TF *TF_;

geometry_msgs::Pose left_controller_pose;


using namespace std;

geometry_msgs::Pose getInFrame(tf::TransformListener &transformListener,geometry_msgs::Pose pose,std::string pose_frame_id, std::string op_frame_id){

    // tf::TransformListener transformListener1;
    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    StampedPose_in.header.frame_id = pose_frame_id;
    
    StampedPose_in.pose = pose;
    //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    transformListener.transformPose(op_frame_id,StampedPose_in,StampedPose_out);

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


void ObsPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
obsPose=*pose;



// obsPose.pose.orientation=left_controller_pose.orientation;

auto eul = TF_->QuaterniontoEuler(obsPose.pose);    //in rad

eul[2]=eul[2]+3.14159/2;

auto quat=TF_->EulerToQuaternion(eul[0],eul[1],eul[2]);

obsPose.pose.orientation=quat;




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

double EulerDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    return double(sqrt(pow((p2.position.x - p1.position.x),2) + pow((p2.position.y - p1.position.y),2) + pow((p2.position.z - p1.position.z),2)));
}

void lc_following(tf::TransformListener &transformListener,geometry_msgs::Pose2D &prev_lc_pose, ros::Publisher &lc_pose2d_publisher){
    auto robotpose=getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
                                            frame_to_follow, "/origin");        
                                    //Pose of robot in origin




    auto left_controller_pose1=getInFrame(transformListener,MakeGeometryMsgsPose(1.5,0,0, 0,0,0,1),
                                        "/left_controller", "/origin");    //get one meter ahead point for lc


    auto lc_euler=QuaterniontoEuler(left_controller_pose1.orientation);


    auto lcpose2d=MakeGeometryMsgsPose2D(left_controller_pose1.position.x,left_controller_pose1.position.y,lc_euler[2]);  

    lcpose2d.theta+=180;

    // ROS_INFO_STREAM(lcpose2d);


    if(EulerDistance(lcpose2d,prev_lc_pose)>0.2){
        lc_pose2d_publisher.publish(lcpose2d);
        prev_lc_pose=lcpose2d;
    }

    
    ros::spinOnce();
    ros::Duration(0.25).sleep();

}



geometry_msgs::Pose wall_following(tf::TransformListener &transformListener,geometry_msgs::Pose2D &prev_plate_pose, ros::Publisher &plate_pose2d_publisher){
    auto robotpose=getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
                                            frame_to_follow, "/origin");        
                                    //Pose of robot in origin



    left_controller_pose=getInFrame(transformListener,MakeGeometryMsgsPose(0,0,0, 0,0,0,1),
                                        "/left_controller", "/origin");    //get one meter ahead point for lc  
    ///////used in callback above



    for(int i=0;i<50;i++){
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    TF_->publishFrame( obsPose.pose,   "obstacle"  ,  "origin");
    ros::spinOnce();
    }



     transformListener.waitForTransform("/origin", "/obstacle", ros::Time(0),
                                                 ros::Duration(3));





    

    auto behind_obstacle=getInFrame(transformListener,MakeGeometryMsgsPose(0,-0.455*1.1+0.12,0,0,0,0,1),
                                        "/obstacle","/origin");


    
   
    TF_->publishFrame( behind_obstacle,   "toberobotframe"  ,  "origin");
    // //cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    

    auto plate_euler=QuaterniontoEuler(behind_obstacle.orientation);


    auto plate_pose2d=MakeGeometryMsgsPose2D(behind_obstacle.position.x,behind_obstacle.position.y,plate_euler[2]+(90));  

    //plate_pose2d.theta+=180;

    ROS_INFO_STREAM(EulerDistance(plate_pose2d,prev_plate_pose));


    if(EulerDistance(plate_pose2d,prev_plate_pose)>0.1){
        plate_pose2d_publisher.publish(plate_pose2d);
        prev_plate_pose=plate_pose2d;
    }

    
    ros::spinOnce();
    ros::Duration(0.25).sleep();

}


int main(int argc, char **argv)
{   
    

    cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"left_controller_following");
    ros::NodeHandle n_("~");     /////~ for arguments
    // goal_pose_in_origin.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;


    TF_=new TF();

    TF_->PublishStaticTransform("plate_frame", "robot_frame_kf",MakeGeometryMsgsPose(0,0.455,0,0,0,0,1));

    odom_broadcaster=new tf2_ros::TransformBroadcaster();
    tf::TransformListener transformListener;


    goal_in_origin_2d_dash.theta+=180;

    left_controller_pose.orientation.w=1;
    obsPose.pose.orientation.w=1;

    ros::Subscriber ObsPose_subscriber_ = n_.subscribe("/obsPose", 10000, ObsPoseCallback);
    ros::Publisher lc_pose2d_publisher = n_.advertise<geometry_msgs::Pose2D>("/planner_goal",10);
    ros::spinOnce();

    
    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z=0;


    geometry_msgs::Pose2D prev_lc_pose;
    geometry_msgs::Pose2D prev_plate_pose;

    

    while(n_.ok()){

        tf::StampedTransform camera_tf_transform_;

        transformListener.waitForTransform("/origin", "/left_controller", ros::Time(0),
                                                 ros::Duration(3));

        transformListener.lookupTransform("/origin", "/left_controller", ros::Time(0),
                                        camera_tf_transform_);


        transformListener.waitForTransform("/origin", frame_to_follow, ros::Time(0),
                                                 ros::Duration(3));

        transformListener.lookupTransform("/origin", frame_to_follow, ros::Time(0),
                                        camera_tf_transform_);


        //ROS_INFO_STREAM(obsPose);

       
        

        if(EulerDistance(obsPose.pose,MakeGeometryMsgsPose(-100,-100,-100,0,0,0,1))<1)
        {   //means no obstacle
                ROS_INFO_STREAM("lc_following"+ to_string(EulerDistance(obsPose.pose,MakeGeometryMsgsPose(-100,-100,-100,0,0,0,1))));
                lc_following(transformListener,prev_lc_pose,lc_pose2d_publisher);
            }

        else
            {
                ROS_INFO_STREAM("wall_following"+to_string(obsPose.pose.position.x)+" "+to_string(obsPose.pose.position.y)+" "+to_string(obsPose.pose.position.z)+" ");
                wall_following(transformListener,prev_plate_pose,lc_pose2d_publisher);

        }



        // ROS_INFO_STREAM("Get 2d in robot");
        // auto goal2dinRobot=getPose2DInRobot(transformListener);         /// point in pose2d where we want to be angle in 0 360

        // ROS_INFO_STREAM("get in frame ");
        


    }


}
