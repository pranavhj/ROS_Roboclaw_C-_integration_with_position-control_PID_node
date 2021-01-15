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


#include "tf_tools.cpp"

// #include "helper.h"



using namespace std;
geometry_msgs::PoseStamped trackerPoseStamped;//trackerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped leftControllerPoseStamped;//leftControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped rightControllerPoseStamped;//rightControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped headSetPoseStamped;//headSetPoseStamped.pose.orientation.w=1;
float CallbackCounter=0;
float tf_publisher_counter=0;

double last_recvd_tracker=0;
double last_recvd_headset=0;
double last_recvd_left_controller=0;
double last_recvd_right_controller=0;


TF *TF_;

// tf::TransformListener *transformListener;










void PublishToTF(){

    ros::spinOnce();
    auto timenow=ros::Time::now().toSec();
   
    if(timenow- last_recvd_tracker<0.1)
        TF_->publishFrame(trackerPoseStamped.pose,"tracker","origin");
    
    if(timenow- last_recvd_left_controller<0.1)
        TF_->publishFrame(leftControllerPoseStamped.pose,"left_controller","origin");
    
    if(timenow- last_recvd_right_controller<0.1) 
        TF_->publishFrame(rightControllerPoseStamped.pose,"right_controller","origin");
    
    if(timenow- last_recvd_headset<0.1)
        TF_->publishFrame(headSetPoseStamped.pose,"headset","origin");


}




void TrackerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 trackerPoseStamped=*pose;
 last_recvd_tracker=ros::Time::now().toSec();
 // CallbackCounter++;

}


void LeftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 leftControllerPoseStamped=*pose;
 last_recvd_left_controller=ros::Time::now().toSec();
 // CallbackCounter++;

}


void RightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 rightControllerPoseStamped=*pose;
 last_recvd_right_controller=ros::Time::now().toSec();
 // CallbackCounter++;

}


void HeadsetCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 headSetPoseStamped=*pose;
 last_recvd_headset=ros::Time::now().toSec();
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

            std::ofstream myfile(fileName);

            if (myfile.is_open()) { 
                myfile<<"";
             }
            else{ ROS_ERROR_STREAM("NOT OPENED");}

            myfile.close();


        }
};



void StartTF(){
    

    geometry_msgs::PoseStamped p;
    p.pose=TF::MakeGeometryMsgsPose(0,0,0,0,0,0,1);
    TF_->PublishStaticTransform("base","origin",p.pose);
 
}


std::vector<double>  Parse(std::string s){
    std::vector<double> vect;
    int start=1;
    for(int i=0;i<s.size();i++){
        if(s[i]==',' ||   s[i]==')'){
            auto numstr=s.substr(start,i-start-1);
            start=i+1;
            // std::cout<<std::stod(numstr)<<std::endl;
            vect.push_back(std::stod(numstr));


        }

    }


    return vect;    

}



void PublishRobotFrame(tf::TransformListener &transformListener,vector<double> crvect, bool calibration){
    
	std::string fileName="/home/pranav/catkin_ws/src/rover/src/calibration2.txt";
	TextData t(fileName);

	geometry_msgs::Pose robot_in_tracker;
	 if (calibration==true){
	    

	    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");


	    geometry_msgs::Pose robot_in_origin;
	    robot_in_origin.position.x=crvect[0];
	    robot_in_origin.position.y=crvect[1];
	    robot_in_origin.position.z=crvect[2];
	    robot_in_origin.orientation.w=1;



	    cout<<"ROBOT IN ORIGIN "<<endl;
	    ROS_INFO_STREAM(robot_in_origin);


	    


        PublishToTF();PublishToTF();ros::Duration(0.1).sleep();    ros::spinOnce();
        
                robot_in_tracker=TF_->getInFrame(transformListener,robot_in_origin,"/origin","/tracker");
        
                // robot_in_tracker=robot_in_origin;
        
                std::string transform_string="(";
                transform_string+=(std::to_string(robot_in_tracker.position.x)+", ");
                transform_string+=(std::to_string(robot_in_tracker.position.y)+", ");
                transform_string+=(std::to_string(robot_in_tracker.position.z)+", ");
        
                transform_string+=(std::to_string(robot_in_tracker.orientation.x)+", ");
                transform_string+=(std::to_string(robot_in_tracker.orientation.y)+", ");
                transform_string+=(std::to_string(robot_in_tracker.orientation.z)+", ");
                transform_string+=(std::to_string(robot_in_tracker.orientation.w)+")");
                
                cout<<"ROBOT IN tracker msg "<<endl;
                ROS_INFO_STREAM(robot_in_tracker);
        

	    // cout<<"ROBOT in Traacker string  "<<endl;
	    // ROS_INFO_STREAM(transform_string);
	    
	    t.clear();
	    t.write(transform_string);
	    ROS_INFO_STREAM("ROBOT IN TRACKER WRITTEN TO calibration2.txt");
	    return;

	}
	
	PublishToTF();PublishToTF();ros::spinOnce();
	//Read transform from text file to robot_in_tracker
	std::string robot_in_tracker_string=t.read();
	auto robot_in_tracker_vect=Parse(robot_in_tracker_string);

	robot_in_tracker.position.x=robot_in_tracker_vect[0];
	robot_in_tracker.position.y=robot_in_tracker_vect[1];
	robot_in_tracker.position.z=robot_in_tracker_vect[2];
	robot_in_tracker.orientation.x=robot_in_tracker_vect[3];
	robot_in_tracker.orientation.y=robot_in_tracker_vect[4];
	robot_in_tracker.orientation.z=robot_in_tracker_vect[5];
	robot_in_tracker.orientation.w=robot_in_tracker_vect[6];

	PublishToTF();PublishToTF();ros::spinOnce();

	ROS_INFO_STREAM("ROBOT IN TRACKER WITHOUT Calibration");
	ROS_INFO_STREAM(robot_in_tracker);
    
    auto ori=TF::EulerToQuaternion(3.14159/2,0,3.14159/2);
    robot_in_tracker.orientation=ori;
    

    TF_->PublishStaticTransform("robot_frame", "tracker", robot_in_tracker);
    
    







}





int main(int argc, char **argv)
{   
    trackerPoseStamped.pose.orientation.w=1;
    rightControllerPoseStamped.pose.orientation.w=1;
    leftControllerPoseStamped.pose.orientation.w=1;
    headSetPoseStamped.pose.orientation.w=1;

	cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"tf_publisher");
    ros::NodeHandle n_("~");     /////~ for arguments
    trackerPoseStamped.pose.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;


    TF_=new TF();


    

    ros::Subscriber tracker_subscriber_ = n_.subscribe("/trackerPose", 10000, TrackerCallback);
    ros::Subscriber left_controller_subscriber_ = n_.subscribe("/leftControllerPose", 10000, LeftControllerCallback);
    ros::Subscriber right_controller_subscriber_ = n_.subscribe("/rightControllerPose", 10000, RightControllerCallback);
    ros::Subscriber headset_subscriber_ = n_.subscribe("/headPose", 10000, HeadsetCallback);

    



    
    tf::TransformListener transformListener;
    StartTF();
    
    bool calibration;
    //auto args=n_.getParam("param", check);
    auto args=n_.getParam("calibration", calibration);


    cout << calibration << " "<<args<<endl;
    if(args==0){
    	ROS_INFO_STREAM("Wrong argument passed, please pass correct arg and run again");
    	ROS_INFO_STREAM("correct args are _calibration:=false _calibration:=true");
    	return 0;

    }
    // ROS_INFO("Got parameter : %s", check.c_str());



    vector<double> centerAndRadiusVect;
    if (calibration==1){
    	ROS_INFO_STREAM("Doing Calibration");
    	ROS_INFO_STREAM("Ensure that receiveUDP.py is running");
    	auto calibInfoFile=TextData();
        std::string centerandradius=calibInfoFile.read();
        centerAndRadiusVect=Parse(centerandradius);
        ROS_INFO_STREAM("centerandradius is ");
        std::cout<<centerAndRadiusVect[0]<<" "<<centerAndRadiusVect[1]<<" "<<centerAndRadiusVect[2]<<" "<<centerAndRadiusVect[3]<<endl;

        int temp_counter=0;
        while(temp_counter<8000){
            PublishToTF();PublishToTF();ros::spinOnce();
            temp_counter++;
        }


        PublishRobotFrame(transformListener,centerAndRadiusVect,calibration);

        
        ros::spinOnce();
        ROS_INFO_STREAM("ENDING PROGRAM, NOW RUN WITH calibration argument set to false");
        return 0;


        

    }

    else{
    	ROS_INFO_STREAM("Directly reading data");
    	auto calibInfoFile=TextData();
        std::string centerandradius=calibInfoFile.read();
        centerAndRadiusVect=Parse(centerandradius);
        ROS_INFO_STREAM("centerandradius is ");
        std::cout<<centerAndRadiusVect[0]<<" "<<centerAndRadiusVect[1]<<" "<<centerAndRadiusVect[2]<<" "<<centerAndRadiusVect[3]<<endl;
        

        int temp_counter=0;
        while(temp_counter<5000){
            PublishToTF();PublishToTF();ros::spinOnce();
            temp_counter++;
        }
        

        PublishRobotFrame(transformListener,centerAndRadiusVect,calibration);

        
        ros::spinOnce();

    }
        
   

    

    
    
	
    while(n_.ok()){
    	

        PublishToTF();
        
        ros::spinOnce();

	}
    
}


 // tf2_ros::TransformBroadcaster tfb;
    // geometry_msgs::TransformStamped transformStamped;
   
     
    // transformStamped.header.frame_id = "world11";
    // transformStamped.child_frame_id = "base_link";
    // transformStamped.transform.translation.x = 0.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = 0.0;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();
