#pragma once

#include "ros/ros.h"
#include "roboclaw/RoboclawMotorVelocity.h"
#include "roboclaw/RoboclawEncoderSteps.h"
#include "rover/position.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Joy.h"
#include <sstream>
#include <iostream>

class Remote{


    private:
        geometry_msgs::Pose2D pose;


        ros::Publisher inv_kinematics_publisher;
        ros::Subscriber robot_odom_position_subscriber;
        ros::Subscriber remoteStateSubscriber;
        geometry_msgs::Pose2D robot_odom_position;
        sensor_msgs::Joy remoteState;
        sensor_msgs::Joy initialRemoteState;

        int noMovementValue=50;
        bool calibrationDone=false;






    public:
        


        Remote(ros::NodeHandle *n);
        void RobotOdomCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message);
        void RemoteStateCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void CalibrateRemote();
        void Execute();
};