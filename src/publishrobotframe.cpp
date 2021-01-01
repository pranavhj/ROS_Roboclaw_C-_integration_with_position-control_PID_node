#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "gazebo_msgs/ModelStates.h"
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
using namespace std;
geometry_msgs::PoseStamped trackerPoseStamped;//trackerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped leftControllerPoseStamped;//leftControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped rightControllerPoseStamped;//rightControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped headSetPoseStamped;//headSetPoseStamped.pose.orientation.w=1;
float CallbackCounter=0;
float tf_publisher_counter=0;
tf2_ros::TransformBroadcaster *odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;

// tf::TransformListener *transformListener;



vector<float> QuaterniontoEuler(geometry_msgs::Pose odom_){
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
    temp.push_back(180+(roll*180/3.14159));
    temp.push_back(180+(pitch*180/3.14159));
    temp.push_back(180+(yaw*180/3.14159)-19.5);
    return temp;
}


void publishFrame(geometry_msgs::PoseStamped poseStamped,string frame_id){

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "origin";
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


void PublishToTF(){
    //tf2_ros::TransformBroadcaster odom_broadcaster;
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = ros::Time::now();
    // odom_trans.header.frame_id = "origin";
    // odom_trans.child_frame_id = "tracker";

    // odom_trans.transform.translation.x = trackerPoseStamped.pose.position.x;
    // odom_trans.transform.translation.y = trackerPoseStamped.pose.position.y;
    // odom_trans.transform.translation.z = trackerPoseStamped.pose.position.z;
    // odom_trans.transform.rotation = trackerPoseStamped.pose.orientation;
   
    //    //send the transform
    // odom_broadcaster->sendTransform(odom_trans);

    // tf_publisher_counter++;
    // cout<<"tf  "<<tf_publisher_counter<<endl;

    publishFrame(trackerPoseStamped,"tracker");
    publishFrame(leftControllerPoseStamped,"left_controller");
    publishFrame(rightControllerPoseStamped,"right_controller");
    publishFrame(headSetPoseStamped,"headset");



    // Odom_.header.frame_id="odom";
    // Odom_.pose.pose=odom_;
    // Odom_.header.stamp=ros::Time::now();
    // Odom_.pose.pose=odom_;
    //Odom_.child_frame_id = "base_link";
}



geometry_msgs::Pose getInFrame(tf::TransformListener &transformListener,geometry_msgs::Pose pose,std::string pose_frame_id, std::string op_frame_id){

    // tf::TransformListener transformListener1;
    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    StampedPose_in.header.frame_id = pose_frame_id;
    pose.orientation.w=1;
    StampedPose_in.pose = pose;
    //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    transformListener.transformPose(op_frame_id,StampedPose_in,StampedPose_out);
    return StampedPose_out.pose;
    
}




void TrackerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 trackerPoseStamped=*pose;
 // CallbackCounter++;

}


void LeftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 leftControllerPoseStamped=*pose;
 // CallbackCounter++;

}


void RightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 rightControllerPoseStamped=*pose;
 // CallbackCounter++;

}


void HeadsetCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 headSetPoseStamped=*pose;
 // CallbackCounter++;

}

class TextData{
    private:


    std::string fileName="/home/pranav/catkin_ws/src/rover/src/calibration.txt";


    public:

        TextData(std::string str){
            fileName=str;

        }

        TextData(){

        }

        std::string read(){
            std::string data;
            std::ifstream myfileread (fileName);
            if (myfileread.is_open())
            {
                std::string line;
                while ( std::getline (myfileread,line) )
                {
                  //std::cout << line << '\n';
                  data=data+line+'\n';
                }

            }

            else {ROS_ERROR_STREAM("NOT OPENED");}

            myfileread.close();

            return data;


        }

        void write(std::string data){
            auto prev_data=read();

            std::ofstream myfile(fileName);

            if (myfile.is_open()) { /* ok, proceed with output */
                //ROS_ERROR_STREAM("OPENED");
                myfile<<prev_data;
                myfile<<data<<std::endl;


             }
            else{ ROS_ERROR_STREAM("NOT OPENED");}

            myfile.close();


        }


        void clear(){
            //auto prev_data=ReadDataFromFile();

            std::ofstream myfile("/home/pranav/ur5_ws/src/robot_control/example.md");

            if (myfile.is_open()) { 
                myfile<<"";
             }
            else{ ROS_ERROR_STREAM("NOT OPENED");}

            myfile.close();


        }
};



void StartTF(){
    
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "origin";
    odom_trans.child_frame_id = "base";

    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation.x=0;
    odom_trans.transform.rotation.y=0;
    odom_trans.transform.rotation.z=0;
    odom_trans.transform.rotation.w=1;
   
       //send the transform
    odom_broadcaster->sendTransform(odom_trans);





}
std::vector<double>  Parse(std::string s){
    std::vector<double> vect;
    int start=1;
    for(int i=0;i<s.size();i++){
        if(s[i]==',' ||   s[i]==')'){
            auto numstr=s.substr(start,i-start-1);
            start=i+1;
            //std::cout<<std::stod(numstr)<<std::endl;
            vect.push_back(std::stod(numstr));


        }

    }


    return vect;    

}



void PublishRobotFrame(tf::TransformListener &transformListener,vector<double> crvect){
    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");


    geometry_msgs::Pose robot_in_origin;
    robot_in_origin.position.x=crvect[0];
    robot_in_origin.position.y=crvect[1];
    robot_in_origin.position.z=crvect[2];
    robot_in_origin.orientation.w=1;



    cout<<"ROBOT IN ORIGIN "<<endl;
    ROS_INFO_STREAM(robot_in_origin);


    auto robot_in_tracker=getInFrame(transformListener,robot_in_origin,"/origin", "/tracker");

    cout<<"ROBOT IN TRACKER "<<endl;
    ROS_INFO_STREAM(robot_in_tracker);



    // geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    // StampedPose_in.header.frame_id = pose_frame_id;
    // pose.orientation.w=1;
    // StampedPose_in.pose = pose;
    // //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    // transformListener.transformPose(op_frame_id,StampedPose_in,StampedPose_out);
    // return StampedPose_out.pose;
    

    // static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // geometry_msgs::TransformStamped static_transformStamped;

    // static_transformStamped.header.stamp = ros::Time::now();
    // static_transformStamped.header.frame_id = "tracker";
    // static_transformStamped.child_frame_id = "robot_frame";
    // static_transformStamped.transform.translation.x = robot_in_tracker.position.x;
    // static_transformStamped.transform.translation.y = robot_in_tracker.position.y;
    // static_transformStamped.transform.translation.z = robot_in_tracker.position.z;
    // // tf2::Quaternion quat;
    // // quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    // static_transformStamped.transform.rotation.x = 0;
    // static_transformStamped.transform.rotation.y = 0;
    // static_transformStamped.transform.rotation.z = 0;
    // static_transformStamped.transform.rotation.w = 1;
    // static_broadcaster.sendTransform(static_transformStamped);


    







}

int main(int argc, char **argv)
{   
    trackerPoseStamped.pose.orientation.w=1;
    rightControllerPoseStamped.pose.orientation.w=1;
    leftControllerPoseStamped.pose.orientation.w=1;
    headSetPoseStamped.pose.orientation.w=1;

	cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"robot_frame_publisher");
    ros::NodeHandle n_("~");     /////~ for arguments
    trackerPoseStamped.pose.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;


    

    // ros::Subscriber tracker_subscriber_ = n_.subscribe("/trackerPose", 10000, TrackerCallback);
    // ros::Subscriber left_controller_subscriber_ = n_.subscribe("/leftControllerPose", 10000, LeftControllerCallback);
    // ros::Subscriber right_controller_subscriber_ = n_.subscribe("/rightControllerPose", 10000, RightControllerCallback);
    // ros::Subscriber headset_subscriber_ = n_.subscribe("/headPose", 10000, HeadsetCallback);

    



    odom_broadcaster=new tf2_ros::TransformBroadcaster();
    tf::TransformListener transformListener;
    // StartTF();
    
    tf::StampedTransform camera_tf_transform_;

    transformListener.waitForTransform("/origin", "/tracker", ros::Time(0),
                                             ros::Duration(3));

    transformListener.lookupTransform("/origin", "/tracker", ros::Time(0),
                                    camera_tf_transform_);



    // //if (robotFrame){
    auto calibInfoFile=TextData();
    std::string centerandradius=calibInfoFile.read();
    auto centerAndRadiusVect=Parse(centerandradius);

        

    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");


    geometry_msgs::Pose robot_in_origin;
    robot_in_origin.position.x=centerAndRadiusVect[0];
    robot_in_origin.position.y=centerAndRadiusVect[1];
    robot_in_origin.position.z=centerAndRadiusVect[2];
    robot_in_origin.orientation.w=1;



    cout<<"ROBOT IN ORIGIN "<<endl;
    ROS_INFO_STREAM(robot_in_origin);


    // auto robot_in_tracker=getInFrame(transformListener,robot_in_origin,"/origin", "/tracker");

    



    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
    StampedPose_in.header.frame_id = "/origin";
    robot_in_origin.orientation.w=1;
    StampedPose_in.pose = robot_in_origin;
    //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
    transformListener.transformPose("/tracker",StampedPose_in,StampedPose_out);
    

    cout<<"ROBOT IN TRACKER "<<endl;
    ROS_INFO_STREAM(StampedPose_out);


    auto robot_in_tracker=StampedPose_out.pose;


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "tracker";
    static_transformStamped.child_frame_id = "robot_frame";
    static_transformStamped.transform.translation.x = robot_in_tracker.position.x;
    static_transformStamped.transform.translation.y = robot_in_tracker.position.y;
    static_transformStamped.transform.translation.z = robot_in_tracker.position.z;
    // tf2::Quaternion quat;
    // quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(static_transformStamped);

   
    ROS_INFO_STREAM("FRAME PUBLISHED");
    ros::spinOnce();
    ros::Duration(3.0).sleep();
    
    

    
    
	
 //    while(n_.ok()){
 //        PublishToTF();
 //        //PublishRobotFrame(centerAndRadiusVect);
 //        ros::spinOnce();
	// }
    
}


