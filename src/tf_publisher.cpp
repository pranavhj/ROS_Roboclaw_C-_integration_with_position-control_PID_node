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
geometry_msgs::PoseStamped trackerPoseStamped1;//trackerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped trackerPoseStamped2;//trackerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped leftControllerPoseStamped;//leftControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped rightControllerPoseStamped;//rightControllerPoseStamped.pose.orientation.w=1;
geometry_msgs::PoseStamped headSetPoseStamped;//headSetPoseStamped.pose.orientation.w=1;
float CallbackCounter=0;
float tf_publisher_counter=0;

double last_recvd_tracker1=0;
double last_recvd_tracker2=0;
double last_recvd_headset=0;
double last_recvd_left_controller=0;
double last_recvd_right_controller=0;
geometry_msgs::Pose robot_in_tracker1;
geometry_msgs::Pose robot_in_tracker2;

TF *TF_;

// tf::TransformListener *transformListener;










void PublishToTF(){

    ros::spinOnce();
    auto timenow=ros::Time::now().toSec();
   
    if(timenow- last_recvd_tracker1<0.5) {
        TF_->publishFrame(trackerPoseStamped1.pose, "tracker_1", "origin");
    }

    if(timenow- last_recvd_tracker2<0.5) {

        TF_->publishFrame(trackerPoseStamped2.pose, "tracker_2", "origin");
    }
    
    if(timenow- last_recvd_left_controller<0.5) {
        TF_->publishFrame(leftControllerPoseStamped.pose, "left_controller", "origin");
    }
    
    if(timenow- last_recvd_right_controller<0.5) {
        TF_->publishFrame(rightControllerPoseStamped.pose, "right_controller", "origin");
    }
    
    if(timenow- last_recvd_headset<0.5) {
        TF_->publishFrame(headSetPoseStamped.pose, "headset", "origin");
    }


}




void TrackerCallback1(const geometry_msgs::PoseStamped::ConstPtr& pose){
 
 // cout<<"Callback "<<CallbackCounter<<endl;
 trackerPoseStamped1=*pose;
 last_recvd_tracker1=ros::Time::now().toSec();
 // CallbackCounter++;

}

void TrackerCallback2(const geometry_msgs::PoseStamped::ConstPtr& pose){

    // cout<<"Callback "<<CallbackCounter<<endl;
    trackerPoseStamped2=*pose;
    last_recvd_tracker2=ros::Time::now().toSec();
    trackerPoseStamped2.pose.position.z=trackerPoseStamped1.pose.position.z;

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


    std::string fileName="/home/kartik/catkin_ws/src/rover/src/calibration.txt";


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



void PublishRobotFrame(tf::TransformListener &transformListener,vector<double> crvect1, vector<double> crvect2, bool calibration){
    
	std::string fileName1="/home/kartik/catkin_ws/src/rover/src/calibration_tf1.txt"; //old -> calibration2.txt
    std::string fileName2="/home/kartik/catkin_ws/src/rover/src/calibration_tf2.txt"; //old -> calibration2.txt
    TextData t1(fileName1);
    TextData t2(fileName2);

	
	 if (calibration==true){
	    

	    ROS_INFO_STREAM("PUBLISHING ROBOT FRAME..........");


	    geometry_msgs::Pose robot_in_origin1;
	    robot_in_origin1.position.x=crvect1[0];
	    robot_in_origin1.position.y=crvect1[1];
	    robot_in_origin1.position.z=crvect1[2];
	    robot_in_origin1.orientation.w=1;

	    geometry_msgs::Pose robot_in_origin2;
        robot_in_origin2.position.x=crvect2[0];
        robot_in_origin2.position.y=crvect2[1];
        robot_in_origin2.position.z=crvect2[2];
        robot_in_origin2.orientation.w=1;



	    cout<<"ROBOT IN ORIGIN1 "<<endl;
	    ROS_INFO_STREAM(robot_in_origin1);

        cout<<"ROBOT IN ORIGIN2 "<<endl;
        ROS_INFO_STREAM(robot_in_origin2);


	    

        double start =ros::Time::now().toSec();
        
        while(ros::Time::now().toSec()-start<1){
            PublishToTF();ros::spinOnce();
        }

        // PublishToTF();PublishToTF();ros::Duration(0.1).sleep();    ros::spinOnce();PublishToTF();PublishToTF();
        
                robot_in_tracker1=TF_->getInFrame(transformListener,robot_in_origin1,"/origin","/tracker_1");
                robot_in_tracker2=TF_->getInFrame(transformListener,robot_in_origin2,"/origin","/tracker_2");
                // robot_in_tracker=robot_in_origin;
        
                std::string transform_string1="(";
                transform_string1+=(std::to_string(robot_in_tracker1.position.x)+", ");
                transform_string1+=(std::to_string(robot_in_tracker1.position.y)+", ");
                transform_string1+=(std::to_string(robot_in_tracker1.position.z)+", ");
        
                transform_string1+=(std::to_string(robot_in_tracker1.orientation.x)+", ");
                transform_string1+=(std::to_string(robot_in_tracker1.orientation.y)+", ");
                transform_string1+=(std::to_string(robot_in_tracker1.orientation.z)+", ");
                transform_string1+=(std::to_string(robot_in_tracker1.orientation.w)+")");
                
                cout<<"ROBOT IN tracker1 msg "<<endl;
                ROS_INFO_STREAM(robot_in_tracker1);

                std::string transform_string2="(";
                transform_string2+=(std::to_string(robot_in_tracker2.position.x)+", ");
                transform_string2+=(std::to_string(robot_in_tracker2.position.y)+", ");
                transform_string2+=(std::to_string(robot_in_tracker2.position.z)+", ");

                transform_string2+=(std::to_string(robot_in_tracker2.orientation.x)+", ");
                transform_string2+=(std::to_string(robot_in_tracker2.orientation.y)+", ");
                transform_string2+=(std::to_string(robot_in_tracker2.orientation.z)+", ");
                transform_string2+=(std::to_string(robot_in_tracker2.orientation.w)+")");

                cout<<"ROBOT IN tracker2 msg "<<endl;
                ROS_INFO_STREAM(robot_in_tracker2);
        

	    // cout<<"ROBOT in Traacker string  "<<endl;
	    // ROS_INFO_STREAM(transform_string);
	    
	    t1.clear();
	    t1.write(transform_string1);
	    ROS_INFO_STREAM("ROBOT IN TRACKER1 WRITTEN TO calibration_tf1.txt");

        t2.clear();
        t2.write(transform_string2);
        ROS_INFO_STREAM("ROBOT IN TRACKER2 WRITTEN TO calibration_tf2.txt");
	    return;

	}
	
	PublishToTF();PublishToTF();ros::spinOnce();
	//Read transform from text file to robot_in_tracker
	std::string robot_in_tracker_string1=t1.read();
	auto robot_in_tracker_vect1=Parse(robot_in_tracker_string1);

    std::string robot_in_tracker_string2=t2.read();
    auto robot_in_tracker_vect2=Parse(robot_in_tracker_string2);

	robot_in_tracker1.position.x=robot_in_tracker_vect1[0];
	robot_in_tracker1.position.y=robot_in_tracker_vect1[1];
	robot_in_tracker1.position.z=robot_in_tracker_vect1[2];
	robot_in_tracker1.orientation.x=robot_in_tracker_vect1[3];
	robot_in_tracker1.orientation.y=robot_in_tracker_vect1[4];
	robot_in_tracker1.orientation.z=robot_in_tracker_vect1[5];
	robot_in_tracker1.orientation.w=robot_in_tracker_vect1[6];

    robot_in_tracker2.position.x=robot_in_tracker_vect2[0];
    robot_in_tracker2.position.y=robot_in_tracker_vect2[1];
    robot_in_tracker2.position.z=robot_in_tracker_vect2[2];
    robot_in_tracker2.orientation.x=robot_in_tracker_vect2[3];
    robot_in_tracker2.orientation.y=robot_in_tracker_vect2[4];
    robot_in_tracker2.orientation.z=robot_in_tracker_vect2[5];
    robot_in_tracker2.orientation.w=robot_in_tracker_vect2[6];

	PublishToTF();PublishToTF();ros::spinOnce();

	ROS_INFO_STREAM("ROBOT IN TRACKER1 WITHOUT Calibration");
	ROS_INFO_STREAM(robot_in_tracker1);

    ROS_INFO_STREAM("ROBOT IN TRACKER2 WITHOUT Calibration");
    ROS_INFO_STREAM(robot_in_tracker2);
    
    auto ori1=TF::EulerToQuaternion(3.14159/2,0,3.14159/2);
    robot_in_tracker1.orientation=ori1;

    auto ori2=TF::EulerToQuaternion(3.14159/2,0,3.14159/2);
    robot_in_tracker2.orientation=ori2;
    

    TF_->PublishStaticTransform("robot_frame_1", "tracker_1", robot_in_tracker1);
    TF_->PublishStaticTransform("robot_frame_2", "tracker_2", robot_in_tracker2);
    

}





int main(int argc, char **argv)
{   
    trackerPoseStamped1.pose.orientation.w=1;
    trackerPoseStamped2.pose.orientation.w=1;
    rightControllerPoseStamped.pose.orientation.w=1;
    leftControllerPoseStamped.pose.orientation.w=1;
    headSetPoseStamped.pose.orientation.w=1;

	cout<<"START MAIN "<<endl;
    ros::init(argc,argv,"tf_publisher");
    ros::NodeHandle n_("~");     /////~ for arguments
    trackerPoseStamped1.pose.orientation.w=1;
    trackerPoseStamped2.pose.orientation.w=1;
    cout<<"Node handle MAIN "<<endl;


    TF_=new TF();


    

    ros::Subscriber tracker_subscriber_1_ = n_.subscribe("/trackerPose1", 10000, TrackerCallback1);
    ros::Subscriber tracker_subscriber_2_ = n_.subscribe("/trackerPose2", 10000, TrackerCallback2);
    ros::Subscriber left_controller_subscriber_ = n_.subscribe("/leftControllerPose", 10000, LeftControllerCallback);
    ros::Subscriber right_controller_subscriber_ = n_.subscribe("/rightControllerPose", 10000, RightControllerCallback);
    ros::Subscriber headset_subscriber_ = n_.subscribe("/headPose", 10000, HeadsetCallback);


    ros::Publisher robot_pose_publisher1=n_.advertise<geometry_msgs::Pose>("/robot_pose1",10);
    ros::Publisher robot_pose_publisher2=n_.advertise<geometry_msgs::Pose>("/robot_pose2",10);
    geometry_msgs::Pose robot_pose_to_publish1;
    geometry_msgs::Pose robot_pose_to_publish2;



    
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



    vector<double> centerAndRadiusVect1, centerAndRadiusVect2;
    if (calibration==1){
    	ROS_INFO_STREAM("Doing Calibration");
    	ROS_INFO_STREAM("Ensure that receiveUDP.py is running");
    	auto calibInfoFile1=TextData("/home/kartik/catkin_ws/src/rover/src/calibration1.txt");
        auto calibInfoFile2=TextData("/home/kartik/catkin_ws/src/rover/src/calibration2.txt");
    	std::string centerandradius1 =calibInfoFile1.read();
        std::string centerandradius2 =calibInfoFile2.read();
        centerAndRadiusVect1=Parse(centerandradius1);
        centerAndRadiusVect2=Parse(centerandradius2);
        ROS_INFO_STREAM("centerandradius1 is ");
        std::cout<<centerAndRadiusVect1[0]<<" "<<centerAndRadiusVect1[1]<<" "<<centerAndRadiusVect1[2]<<" "<<centerAndRadiusVect1[3]<<endl;
        ROS_INFO_STREAM("centerandradius2 is ");
        std::cout<<centerAndRadiusVect2[0]<<" "<<centerAndRadiusVect2[1]<<" "<<centerAndRadiusVect2[2]<<" "<<centerAndRadiusVect2[3]<<endl;

        int temp_counter=0;
        while(temp_counter<8000){
            PublishToTF();PublishToTF();ros::spinOnce();
            temp_counter++;
        }


        PublishRobotFrame(transformListener,centerAndRadiusVect1, centerAndRadiusVect2, calibration);

        
        ros::spinOnce();
        ROS_INFO_STREAM("ENDING PROGRAM, NOW RUN WITH calibration argument set to false");
        return 0;


        

    }

    else{
    	ROS_INFO_STREAM("Directly reading data");
    	auto calibInfoFile1=TextData("/home/kartik/catkin_ws/src/rover/src/calibration_tf1.txt");
        std::string centerandradius1=calibInfoFile1.read();
        centerAndRadiusVect1=Parse(centerandradius1);
        ROS_INFO_STREAM("centerandradius1 is ");
        std::cout<<centerAndRadiusVect1[0]<<" "<<centerAndRadiusVect1[1]<<" "<<centerAndRadiusVect1[2]<<" "<<centerAndRadiusVect1[3]<<endl;


        ROS_INFO_STREAM("Directly reading data");
        auto calibInfoFile2=TextData("/home/kartik/catkin_ws/src/rover/src/calibration_tf2.txt");
        std::string centerandradius2=calibInfoFile2.read();
        centerAndRadiusVect2=Parse(centerandradius2);
        ROS_INFO_STREAM("centerandradius2 is ");
        std::cout<<centerAndRadiusVect2[0]<<" "<<centerAndRadiusVect2[1]<<" "<<centerAndRadiusVect2[2]<<" "<<centerAndRadiusVect2[3]<<endl;


        int temp_counter=0;
        while(temp_counter<5000){
            PublishToTF();PublishToTF();ros::spinOnce();
            temp_counter++;
        }
        

        PublishRobotFrame(transformListener,centerAndRadiusVect1, centerAndRadiusVect2, calibration);

        
        ros::spinOnce();

    }
        
   

    

    
    
	
    while(n_.ok()){
    	

        PublishToTF();
        try{
        	auto robot_in_origin1=TF_->getInFrame(transformListener,robot_in_tracker1,"/tracker_1","/origin");
        	robot_pose_publisher1.publish(robot_in_origin1);

            auto robot_in_origin2=TF_->getInFrame(transformListener,robot_in_tracker2,"/tracker_2","/origin");
            robot_pose_publisher2.publish(robot_in_origin2);
        }
        catch(...){

        }
        
        
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
