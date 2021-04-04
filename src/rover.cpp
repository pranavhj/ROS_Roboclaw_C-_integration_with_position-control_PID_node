/**
 * @copyright (c) 2020, Pranav Jain, Kartik Venkat
 *
 * @file rover.cpp
 *
 * @authors
 * Pranav Jain - pranavhj@umd.edu
 * Kartik Venkat - kartik97@terpmail.umd.edu
 *
 * @version 1.0
 *
 * @section LICENSE
 * MIT License
 *
 * @section DESCRIPTION:
 * This file contains the function definitions of the Rover class.
 *
 */


#include "rover.h"

#include "helper.h"


/**
   * @brief: Constructor for the Rover class
   * @param: pointer to the node handle
   * @return: None
   * */
Rover::Rover(ros::NodeHandle *n){
	std::cout<<"Before Ini"<<std::endl;
	last_FK_call=ros::Time::now().toSec();

	InitializeMatrices();

	velocity_publisher=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel",10);
	velocity_publisher_1=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel_1",10);
	ForwardKinematicsPositionPublisher=n->advertise<geometry_msgs::Pose2D>("/robot_odom_position",10);
	DonePublisher=n->advertise<std_msgs::String>("/DoneTopic",10);
	reset_enc_publisher=n->advertise<std_msgs::Bool>("/reset_enc",10);

	
	pose_subscriber=n->subscribe("/motor_enc",100,&Rover::poseCallback,this);
	// position_subscriber=n->subscribe("/position",100,&Rover::positionCallback,this);     //given by inverse kinema	
	pose_subscriber_1=n->subscribe("/motor_enc_1",10,&Rover::pose_1Callback,this);
	inv_kinematics_subscriber=n->subscribe("/inv_kinematics",10,&Rover::IKCallback,this);
	inv_kinematics_publisher=n->advertise<motor_controller::position>("/position",10);
	cmd_vel_subscriber=n->subscribe("/cmd_vel",1,&Rover::CMDVELCallback,this);


	TF_=new TF();
	ros::spinOnce();
	ros::Duration(0.5).sleep();
	

	ForwardKinematics();              //so that it does not move to zero from initial pose
	// IKpose=FKPose;
	// ROS_INFO_STREAM(IKpose);
	// ros::Duration(0.5).sleep();


	


	std_msgs::Bool d;
	d.data=true;
	reset_enc_publisher.publish(d);
	ros::Duration(3.0).sleep();
	//back:

	// try
	{	
		ros::spinOnce();
		tf::StampedTransform camera_tf_transform_;

	    transformListener.waitForTransform("origin", "robot_frame_1", ros::Time(0),
	                                             ros::Duration(3));

	    transformListener.lookupTransform("origin", "robot_frame_1", ros::Time(0),
	                                    camera_tf_transform_);



	    // ROS_INFO_STREAM("Transform b/w origin and robot_frame is ");

    	// std::cout<<camera_tf_transform_.getOrigin().x()<<camera_tf_transform_.getOrigin().y()<<camera_tf_transform_.getOrigin().z()<<std::endl;

	    //publish robot_ini state


		//do fk
		//get current pose
		//get val of ini pose from that

	    FK();                      //only uses encoders
	    FKVROriginFrame();          //only uses encs
	    ForwardKinematics();       //only uses encs



	    std::cout<<"Current pose is Xn_1 "<<Xn_1<<std::endl;
	    std::cout<<"FKPose is from ForwardKinematics ";
	    ROS_INFO_STREAM(FKPose);


	    auto quat=EulerToQuaternion(0,0,-Xn_1(2));
	    std::cout<<"Eul to quat done which is"<<std::endl;
	    ROS_INFO_STREAM(quat);



	    auto tht=float(Xn_1(2));
	    auto x=float(Xn_1(0)); auto y=float(Xn_1(1));
	    auto xo=-( (x*cos(tht)) + (y*sin(tht)) );
	    auto yo=-( -(x*sin(tht)) + (y*cos(tht)) );


	    auto  robot_pose_in_origin_ini =TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(xo,yo,0,   quat.x,quat.y,quat.z,quat.w), "robot_frame_1", "origin");
	    std::cout<<"Robot Frame1 Get in frame done"<<std::endl;

	    // auto robot_pose_in_origin_ini=getInFrame(MakeGeometryMsgsPose(0,0,0 ,0,0,0,1), "/robot_frame",  "/origin");
	    ROS_INFO_STREAM("Ini Robot1 pose in origin  is ");
	    ROS_INFO_STREAM(robot_pose_in_origin_ini);


	    TF_->PublishStaticTransform("/robot_initial_frame_1", "/origin",robot_pose_in_origin_ini);




        auto  robot_pose2_in_origin_ini =TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(xo,yo,0,   quat.x,quat.y,quat.z,quat.w), "robot_frame_2", "origin");
        std::cout<<"Robot Frame2 Get in frame done"<<std::endl;

        // auto robot_pose_in_origin_ini=getInFrame(MakeGeometryMsgsPose(0,0,0 ,0,0,0,1), "/robot_frame",  "/origin");
        ROS_INFO_STREAM("Ini Robot2 pose in origin  is ");
        ROS_INFO_STREAM(robot_pose2_in_origin_ini);


        TF_->PublishStaticTransform("/robot_initial_frame_2", "/origin",robot_pose2_in_origin_ini);
	

	}
		
}


/**
   * @brief:
   * @param:
   * @return: None
   * */
geometry_msgs::Quaternion Rover::EulerToQuaternion(double x,double y,double z){
	tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( x,y,z);  // Create this quaternion from roll/pitch/yaw (in radians)
    myQuaternion.normalize();
    ROS_INFO_STREAM(myQuaternion);
    geometry_msgs::Quaternion geom_quat = tf2::toMsg(myQuaternion);
    return geom_quat;
}



// geometry_msgs::Pose Rover::MakeGeometryMsgsPose(double px,double py,double pz,double rx,double ry,double rz,double rw){
// 	geometry_msgs::Pose p;
// 	p.position.x=px;
// 	p.position.y=py;
// 	p.position.z=pz;

// 	p.orientation.x=rx;
// 	p.orientation.y=ry;
// 	p.orientation.z=rz;
// 	p.orientation.w=rw;
// 	return p;
// }


// geometry_msgs::Pose2D Rover::MakeGeometryMsgsPose2D(double px,double py,double pt){
// 	geometry_msgs::Pose2D p;
// 	p.x=px;
// 	p.y=py;
// 	p.theta=pt;
// 	return p;
// }






/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::poseCallback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	//std::cout<<"poseCallback";
	//ROS_INFO_STREAM(*pose_message);
	pose.index=pose_message->index;
	pose.mot1_enc_steps=pose_message->mot1_enc_steps;
	pose.mot2_enc_steps=pose_message->mot2_enc_steps;
	//PIDcontroller(position.position_1,position.position_2);

	double t =ros::Time::now().toSec();
	if(t- last_FK_call>0.01){
		FKVROriginFrame();
		last_FK_call=t;
	}
	
}


/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::pose_1Callback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	//std::cout<<"pose_1Callback";
	//ROS_INFO_STREAM(*pose_message);
	pose_1.index=pose_message->index;
	pose_1.mot1_enc_steps=pose_message->mot1_enc_steps;
	pose_1.mot2_enc_steps=pose_message->mot2_enc_steps;
	double t =ros::Time::now().toSec();
	if(t- last_FK_call>0.01){
		FKVROriginFrame();
		last_FK_call=t;
	}

	//PIDcontroller(position.position_1,position.position_2);
	

}



/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::CMDVELCallback(const geometry_msgs::Twist::ConstPtr& pose_message){
	//std::cout<<"poseCallback";
	//ROS_INFO_STREAM(*pose_message);
	cmd_vel=*pose_message;
	//ROS_INFO_STREAM(cmd_vel);	
		
	//PIDcontroller(position.position_1,position.position_2);
	
}



/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::IKCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message){   
	//IKpose=*(pose_message);
	IKpose.x=(pose_message->x);
	IKpose.y=(pose_message->y);
	IKpose.theta=(pose_message->theta);         /////////////Is in deg
	
	std::cout<<"IKCallback"<<std::endl;
	ROS_INFO_STREAM(IKpose);
}

/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::IK(){              //takes IK pose and converts to position values wrt robot_initial
	
	ros::spinOnce();
	float x=IKpose.x;
    float y=IKpose.y;
    float w=IKpose.theta;
    // std::cout<<"xyw "<<x<<" "<<y<<" "<<w<<std::endl;
    
	// w=w*3.14159/180;
    auto p1=matrixCalculation(x,y,w);
    // std::cout<<"Targets:"<<p1.position.x<<" "<<p1.position.y<<" "<<p1.position.z<<std::endl;
    position.position_1=-int(p1.position.x);
    position.position_2=int(p1.position.y);
    position.position_3=int(p1.position.z);//angle in deg
    // inv_kinematics_publisher.publish(position);
    //cout<<"Published"<<endl;
    // ROS_INFO_STREAM("IN IK");
    // ROS_INFO_STREAM(position);
    
}

/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::ExecuteIK(){
	//std::cout<<position.position_1<<"    "<<position.position_2<<"  "<<position.position_3<<std::endl;

	//cout<<"Start"<<endl;
	//ROS_INFO_STREAM(position);

	// std::cout<<"Posns:"<<position.position_1<<position.position_2<<position.position_3<<std::endl;
	// ROS_INFO_STREAM(IKpose);
	IK();

	PIDcontroller(position.position_1,position.position_2,position.position_3);
	//cout<<"End"<<endl;

	ForwardKinematics();





	roboclaw::RoboclawMotorVelocity vel_msg;
	vel_msg.index=0;
	vel_msg.mot1_vel_sps=0;
	vel_msg.mot2_vel_sps=0;                                               //To stop the motor when pid successful
	velocity_publisher.publish(vel_msg);


	roboclaw::RoboclawMotorVelocity vel_msg_1;
	vel_msg_1.index=0;							//To stop the motor when pid successful
	vel_msg_1.mot2_vel_sps=0;
	velocity_publisher_1.publish(vel_msg_1);
	//std::cout<<"zeros"<<std::endl;


	ros::spinOnce();

//cout<<pose.mot1_enc_steps<<endl;
}

/**
   * @brief:
   * @param:
   * @return: float
   * */
float Euler(geometry_msgs::Pose2D finalState,geometry_msgs::Pose2D prevState){
	return float(sqrt((finalState.x*finalState.x)+(finalState.y*finalState.y))-sqrt((prevState.x*prevState.x)+(prevState.y*prevState.y)));
	}



/**
   * @brief:
   * @param:
   * @return: None
   * */
void Rover::ExecuteCMDVEL(){
	ros::spinOnce();
	
	
	
	geometry_msgs::Pose2D finalState;
	finalState.x=cmd_vel.linear.x;
	finalState.y=cmd_vel.linear.y;
	finalState.theta=cmd_vel.angular.z;
	//ROS_INFO_STREAM(finalState);
	
	if(abs(Euler(finalState,prevState))>interpolation_speed){
		//divide speeds
		auto temp=finalState;
		float tempxval=interpolation_speed;
		if(abs(finalState.x-prevState.x)>abs(finalState.y-prevState.y)){
				
				if(abs(finalState.x-prevState.x)<tempxval){
					tempxval=finalState.x-prevState.x;
					}
				else{
					tempxval=(finalState.x-prevState.x)/abs(finalState.x-prevState.x)*tempxval;
					}
				finalState.x=prevState.x+tempxval;
				finalState.y=prevState.y+((temp.y-prevState.y)/abs(temp.x-prevState.x)*abs(tempxval));
			
				ros::Duration(0.01).sleep();
				
			}
		else{
				if(abs(finalState.y-prevState.y)<tempxval){
					tempxval=finalState.y-prevState.y;
					}
				else{
					tempxval=(finalState.y-prevState.y)/abs(finalState.y-prevState.y)*tempxval;
					}
				finalState.y=prevState.y+tempxval;
				//if(finalState.x==0)
				finalState.x=prevState.x+((temp.x-prevState.x)/abs(temp.y-prevState.y)*abs(tempxval));
			
				ros::Duration(0.01).sleep();
				}
		
		}
		
		
		
	prevState=finalState;
	auto p1=matrixCalculation(finalState.x,finalState.y,finalState.theta);
	
	//ROS_INFO_STREAM(finalState);
	//std::cout<<*targets<<" "<<*(targets+1)<<" "<<*(targets+2)<<std::endl;
	auto speed=p1.position.x;//pos1/=10000;
	auto speed1=p1.position.y;//pos2/=10000;
	auto speed2=p1.position.z;//pos3/=10000;
	
	ros::spinOnce();
	ForwardKinematics();
	
	{	/*
		if(abs(speed-prev_speed)>interpolation_speed){
			speed=prev_speed+((speed-prev_speed)/abs(speed-prev_speed)*interpolation_speed);
			ros::Duration(0.001).sleep();
		}
		if(abs(speed1-prev_speed_1)>interpolation_speed){
			speed1=prev_speed_1+((speed1-prev_speed_1)/abs(speed1-prev_speed_1)*interpolation_speed);	
			ros::Duration(0.001).sleep();
			
		}
		if(abs(speed2-prev_speed_2)>interpolation_speed){
			speed2=prev_speed_2+((speed2-prev_speed_2)/abs(speed2-prev_speed_2)*interpolation_speed);
			ros::Duration(0.001).sleep();
		}
		*/
		//std::cout<<speed<<" "<<speed1<<" "<<speed2<<std::endl;
	}


	roboclaw::RoboclawMotorVelocity vel_msg;
	vel_msg.index=0;
	vel_msg.mot1_vel_sps=speed;
	vel_msg.mot2_vel_sps=-speed1;                                               //To stop the motor when pid successful
	velocity_publisher.publish(vel_msg);


	roboclaw::RoboclawMotorVelocity vel_msg_1;
	vel_msg_1.index=0;							//To stop the motor when pid successful
	vel_msg_1.mot2_vel_sps=-speed2;
	velocity_publisher_1.publish(vel_msg_1);
	
	
	prev_speed=speed;
	prev_speed_1=speed1;
	prev_speed_2=speed2;
	
	
	ros::spinOnce();
	ForwardKinematics();
	KalmanFilter();
	
}






void Rover::ExecuteCMDVELNoInterpolation(){
	ros::spinOnce();
	
	geometry_msgs::Pose2D finalState;
	finalState.x=cmd_vel.linear.x;
	finalState.y=cmd_vel.linear.y;
	finalState.theta=cmd_vel.angular.z;
	//ROS_INFO_STREAM(finalState);
	

	auto p1=matrixCalculation(finalState.x,finalState.y,finalState.theta);
	
	//ROS_INFO_STREAM(finalState);
	//std::cout<<*targets<<" "<<*(targets+1)<<" "<<*(targets+2)<<std::endl;
	auto speed=p1.position.x;//pos1/=10000;
	auto speed1=p1.position.y;//pos2/=10000;
	auto speed2=p1.position.z;//pos3/=10000;
	
	ros::spinOnce();
	ForwardKinematics();
	


	roboclaw::RoboclawMotorVelocity vel_msg;
	vel_msg.index=0;
	vel_msg.mot1_vel_sps=speed;
	vel_msg.mot2_vel_sps=-speed1;                                               //To stop the motor when pid successful
	velocity_publisher.publish(vel_msg);


	roboclaw::RoboclawMotorVelocity vel_msg_1;
	vel_msg_1.index=0;							//To stop the motor when pid successful
	vel_msg_1.mot2_vel_sps=-speed2;
	velocity_publisher_1.publish(vel_msg_1);
	
	
	prev_speed=speed;
	prev_speed_1=speed1;
	prev_speed_2=speed2;
	
	
	ros::spinOnce();
	ForwardKinematics();
	KalmanFilter();

}











//Pose to pose wrt current robot pose
geometry_msgs::Pose2D Rover::ConvertPosition(geometry_msgs::Pose2D remoteState){
    geometry_msgs::Pose2D temp;
    float theta=FKPose.theta*3.142/180;
    float err_x=remoteState.x;
    float err_y=remoteState.y;
    temp.x=((cos(theta)*err_x)+(sin(theta)*err_y));
    temp.y=((-sin(theta)*err_x)+(cos(theta)*err_y));
    temp.theta=remoteState.theta;
    return temp;


}




void Rover::ExecuteIKOnlySpeed(){

	ros::spinOnce();
	float x=IKpose.x;
    float y=IKpose.y;
    float w=IKpose.theta;                    //In origin
    // std::cout<<"xyw "<<x<<" "<<y<<" "<<w<<std::endl;

    // ROS_INFO_STREAM("IKPOSE");
    // ROS_INFO_STREAM(IKpose);

    auto quat=TF::EulerToQuaternion(0,0,w);
    auto goal_in_robot_intial=TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(x,y,0,quat.x,quat.y,quat.z,quat.w), "origin", "robot_initial_frame_1");
    // ROS_INFO_STREAM("goal_in_robot_intial");
    // ROS_INFO_STREAM(goal_in_robot_intial);
    
	// w=w*3.14159/180;
	auto eul=TF_->QuaterniontoEuler(goal_in_robot_intial);
    auto p1=matrixCalculation(-goal_in_robot_intial.position.x*1000,
    						  goal_in_robot_intial.position.y*1000,
    						  -eul[2]*1/angle_correction_factor);
    // std::cout<<"Targets:"<<p1.position.x<<" "<<p1.position.y<<" "<<p1.position.z<<std::endl;
    position.position_1=-int(p1.position.x);
    position.position_2=int(p1.position.y);
    position.position_3=int(p1.position.z);
	PIDControllerOnlySpeed(position.position_1,position.position_2,position.position_3);
}




geometry_msgs::Pose Rover::GetEncoderPosnsFromTF(){
	auto robot_in_robot_intial=TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(0,0,0,0,0,0,1), "robot_frame_1", "robot_initial_frame_1");

	auto eul=TF_->QuaterniontoEuler(robot_in_robot_intial);

	auto wheelPosns=matrixCalculation(-robot_in_robot_intial.position.x*1000,     //converts into mm
									  robot_in_robot_intial.position.y*1000,     //converts into mm
									  -eul[2]*1/angle_correction_factor);                                  //in rad

	return wheelPosns;
}

void Rover::PIDControllerOnlySpeed(float goal,float goal1, float goal2){
	
	roboclaw::RoboclawMotorVelocity vel_msg_; 
	roboclaw::RoboclawMotorVelocity vel_msg_1;
	vel_msg_.index=0;
	vel_msg_1.index=0;

	std::cout<<"GOAL"<<std::endl;
	std::cout<<goal<<" "<<goal1<<" "<<goal2<<std::endl;
	auto wheelPosns_wrt_VR=GetEncoderPosnsFromTF();
	

	ROS_INFO_STREAM("wheelPosns_wrt_VR");
	ROS_INFO_STREAM(wheelPosns_wrt_VR);




	float e_current=goal-float(wheelPosns_wrt_VR.position.x);           ///  Check which posn corresponds to which
	float e_current_1=goal1-float(wheelPosns_wrt_VR.position.y);
	float e_current_2=goal2-float(wheelPosns_wrt_VR.position.z);

	
	

	if((abs((goal)-(float(-wheelPosns_wrt_VR.position.x)))>150.0        ||    abs((goal1)-(float(wheelPosns_wrt_VR.position.y)))>150.0    ||   abs((goal2)-(float(wheelPosns_wrt_VR.position.z)))>150.0)  &&   ros::ok() )
		{

		ForwardKinematics();                    //Publishes FK Pose to ros
			{	ros::spinOnce();
				
				e_current=goal-float(-wheelPosns_wrt_VR.position.x);
				e_current_1=goal1-float(wheelPosns_wrt_VR.position.y);
				e_current_2=goal2-float(wheelPosns_wrt_VR.position.z);
				//std::cout<<"errors "<<e_current<<" "<<e_current_1<<" "<<e_current_2<<std::endl;
				float max_speed_=abs(e_current_1)*3.5;
				if (max_speed_>full_speed)
					max_speed_=full_speed;
				float speed=0;
				float speed1=0;
				float speed2=0;
				if (abs(e_current) >= abs(e_current_1)   and  abs(e_current) >= abs(e_current_2)  )
					{
					speed=(max_speed_*e_current/abs(e_current));
					speed1=(max_speed_*abs(e_current_1)/abs(e_current)*e_current_1/abs(e_current_1));
					speed2=(max_speed_*abs(e_current_2)/abs(e_current)*e_current_2/abs(e_current_2));

					}

				else if (abs(e_current_1) >= abs(e_current_2)   and  abs(e_current_1) >= abs(e_current))
					{//ROS_INFO_STREAM("22222222");
					 
					 speed1=(max_speed_*e_current_1/abs(e_current_1));
					 speed=(max_speed_*abs(e_current)/abs(e_current_1)*e_current/abs(e_current));
					 speed2=(max_speed_*abs(e_current_2)/abs(e_current_1)*e_current_2/abs(e_current_2));
					}

				else if (abs(e_current_2) >= abs(e_current)   and  abs(e_current_2) >= abs(e_current_1))
					{
					 //ROS_INFO_STREAM("3333333");
					 //std::cout<<"errors "<<e_current<<" "<<e_current_1<<" "<<e_current_2<<" "<<max_speed<<std::endl;
				
					 speed2=(max_speed_*e_current_2/abs(e_current_2));
					 speed=(max_speed_*abs(e_current)/abs(e_current_2)*e_current/abs(e_current));
					 speed1=(max_speed_*abs(e_current_1)/abs(e_current_2)*e_current_1/abs(e_current_1));
					}			

				//std::cout<<"speeds "<<speed<<" "<<speed1<<" "<<speed2<<std::endl;
					
				if(abs(speed-prev_speed)>interpolation_speed){
					speed=prev_speed+((speed-prev_speed)/abs(speed-prev_speed)*interpolation_speed);
					ros::Duration(0.01).sleep();
				}
				if(abs(speed1-prev_speed_1)>interpolation_speed){
					speed1=prev_speed_1+((speed1-prev_speed_1)/abs(speed1-prev_speed_1)*interpolation_speed);	
					ros::Duration(0.01).sleep();
					
				}
				if(abs(speed2-prev_speed_2)>interpolation_speed){
					speed2=prev_speed_2+((speed2-prev_speed_2)/abs(speed2-prev_speed_2)*interpolation_speed);
					ros::Duration(0.01).sleep();
				}



				std::cout<<"SpeedLoop"<<"   "<<speed<<"  "<<speed1<<"  "<<speed2<<std::endl;
				DoneString.data="SpeedLoop";
				DonePublisher.publish(DoneString);

				prev_speed=speed;
				prev_speed_1=speed1;
				prev_speed_2=speed2;
				vel_msg_.mot1_vel_sps=int(1*speed);
				vel_msg_1.mot2_vel_sps=int(1*speed1);
				vel_msg_.mot2_vel_sps=int(1*speed2);
			
				velocity_publisher.publish(vel_msg_);
				velocity_publisher_1.publish(vel_msg_1);
				//ROS_INFO_STREAM(vel_msg_);ROS_INFO_STREAM(vel_msg_1);
			}

		}
		else{
			roboclaw::RoboclawMotorVelocity vel_msg;
			vel_msg.index=0;
			vel_msg.mot1_vel_sps=0;
			vel_msg.mot2_vel_sps=0;                                               //To stop the motor when pid successful
			

			roboclaw::RoboclawMotorVelocity vel_msg_1;
			vel_msg_1.index=0;							//To stop the motor when pid successful
			vel_msg_1.mot2_vel_sps=0;


			velocity_publisher.publish(vel_msg);
			velocity_publisher_1.publish(vel_msg_1);
			ROS_INFO_STREAM("#########STOP");
		}

	}


void Rover::PIDcontroller(float goal, float goal1,float goal2){
	IK();
	roboclaw::RoboclawMotorVelocity vel_msg_; 
	roboclaw::RoboclawMotorVelocity vel_msg_1;
	vel_msg_.index=0;
	vel_msg_1.index=0;
	// ROS_INFO_STREAM("IN PID");
	// ROS_INFO_STREAM(position);

	goal=position.position_1;
	goal1=position.position_2;
	goal2=position.position_3;
	float e_current=goal-float(pose.mot1_enc_steps);
	float e_current_1=goal1-float(pose_1.mot2_enc_steps);
	float e_current_2=goal2-float(pose.mot2_enc_steps);
	// std::cout<<"IN PID "<<goal<<" "<<goal1<<" "<<goal2<<endl;

	while((abs((goal)-(float(pose.mot1_enc_steps)))>10.0        ||    abs((goal1)-(float(pose_1.mot2_enc_steps)))>10.0    ||   abs((goal2)-(float(pose.mot2_enc_steps)))>10.0)  &&   ros::ok() )
		{
		remote->Execute();
		
		IK();
		//std::cout<<"PID"<<std::endl;
		ForwardKinematics();
		if(abs((goal)-(float(pose.mot1_enc_steps)))>50     &&   abs((goal1)-(float(pose_1.mot2_enc_steps)))>50    &&   abs((goal2)-(float(pose.mot2_enc_steps)))>50)
			{	ros::spinOnce();
				goal=position.position_1;
				goal1=position.position_2;
				goal2=position.position_3;
				e_current=goal-float(pose.mot1_enc_steps);
				e_current_1=goal1-float(pose_1.mot2_enc_steps);
				e_current_2=goal2-float(pose.mot2_enc_steps);
				//std::cout<<"errors "<<e_current<<" "<<e_current_1<<" "<<e_current_2<<std::endl;
				float max_speed_=abs(e_current_1)*3.5;
				if (max_speed_>full_speed)
					max_speed_=full_speed;
				float speed=0;
				float speed1=0;
				float speed2=0;
				if (abs(e_current) >= abs(e_current_1)   and  abs(e_current) >= abs(e_current_2)  )
					{
					speed=(max_speed_*e_current/abs(e_current));
					speed1=(max_speed_*abs(e_current_1)/abs(e_current)*e_current_1/abs(e_current_1));
					speed2=(max_speed_*abs(e_current_2)/abs(e_current)*e_current_2/abs(e_current_2));

					}

				else if (abs(e_current_1) >= abs(e_current_2)   and  abs(e_current_1) >= abs(e_current))
					{//ROS_INFO_STREAM("22222222");
					 
					 speed1=(max_speed_*e_current_1/abs(e_current_1));
					 speed=(max_speed_*abs(e_current)/abs(e_current_1)*e_current/abs(e_current));
					 speed2=(max_speed_*abs(e_current_2)/abs(e_current_1)*e_current_2/abs(e_current_2));
					}

				else if (abs(e_current_2) >= abs(e_current)   and  abs(e_current_2) >= abs(e_current_1))
					{
					 //ROS_INFO_STREAM("3333333");
					 //std::cout<<"errors "<<e_current<<" "<<e_current_1<<" "<<e_current_2<<" "<<max_speed<<std::endl;
				
					 speed2=(max_speed_*e_current_2/abs(e_current_2));
					 speed=(max_speed_*abs(e_current)/abs(e_current_2)*e_current/abs(e_current));
					 speed1=(max_speed_*abs(e_current_1)/abs(e_current_2)*e_current_1/abs(e_current_1));
					}			

				//std::cout<<"speeds "<<speed<<" "<<speed1<<" "<<speed2<<std::endl;
					
				if(abs(speed-prev_speed)>interpolation_speed){
					speed=prev_speed+((speed-prev_speed)/abs(speed-prev_speed)*interpolation_speed);
					ros::Duration(0.01).sleep();
				}
				if(abs(speed1-prev_speed_1)>interpolation_speed){
					speed1=prev_speed_1+((speed1-prev_speed_1)/abs(speed1-prev_speed_1)*interpolation_speed);	
					ros::Duration(0.01).sleep();
					
				}
				if(abs(speed2-prev_speed_2)>interpolation_speed){
					speed2=prev_speed_2+((speed2-prev_speed_2)/abs(speed2-prev_speed_2)*interpolation_speed);
					ros::Duration(0.01).sleep();
				}



				std::cout<<"SpeedLoop"<<"   "<<speed<<"  "<<speed1<<"  "<<speed2<<std::endl;
				DoneString.data="SpeedLoop";
				DonePublisher.publish(DoneString);

				prev_speed=speed;
				prev_speed_1=speed1;
				prev_speed_2=speed2;
				vel_msg_.mot1_vel_sps=int(1*speed);
				vel_msg_1.mot2_vel_sps=int(1*speed1);
				vel_msg_.mot2_vel_sps=int(1*speed2);
			
				velocity_publisher.publish(vel_msg_);
				velocity_publisher_1.publish(vel_msg_1);
				//ROS_INFO_STREAM(vel_msg_);ROS_INFO_STREAM(vel_msg_1);
					}
		
			else
		{
		
			goal=position.position_1;
			goal1=position.position_2;
			goal2=position.position_3;	
			float e=goal-float(pose.mot1_enc_steps);
			float e1=goal1-float(pose_1.mot2_enc_steps);
			float e2=goal2-float(pose.mot2_enc_steps);
		    float speed=Kp1*e+ki1*(total_error_1)+kd1*(e-prev_e1);
			float speed1=Kp2*e1+ki2*(total_error_2)+kd2*(e1-prev_e2);
			float speed2=Kp3*e2+ki3*(total_error_3)+kd3*(e2-prev_e3);

			if (speed>full_speed)
				speed=full_speed;
			else if(speed<-max_speed)
				speed=-full_speed;

			if (speed1>full_speed)
				speed1=full_speed;
			else if(speed1<-max_speed)
				speed1=-full_speed;

			if (speed2>full_speed)
				speed2=full_speed;
			else if(speed2<-max_speed)
				speed2=-full_speed;

			int temp1=00,temp2=00;
			// if (e<0)
			// 	temp1=-temp1;
			// if (e2<0)
			// 	temp2=-temp2;
			
		 //   if (speed>3000    &&   speed1>3000      &&    speed2>3000)
		 //   		{speed=speed*(abs(e_current)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));
		 //   		speed1=speed1*(abs(e_current_1)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));
		 //   		speed2=speed2*(abs(e_current_2)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));}



			prev_speed=speed;
			prev_speed_1=speed1;
			prev_speed_2=speed2;
			vel_msg_.mot1_vel_sps=int(speed);
			vel_msg_1.mot2_vel_sps=int(speed1);
			vel_msg_.mot2_vel_sps=int(speed2);
			//vel_msg_1.mot1_vel_sps=1*speed;                    ///Just for checking


			
			//std::cout<<speed<<"   "<<speed1<<"   "<<speed2<<std::endl;
			std::cout<<abs(goal)-abs(float(pose.mot1_enc_steps))<<"    "<<abs(goal1)-abs(float(pose_1.mot2_enc_steps))<<"    "<<abs(goal2)-abs(float(pose.mot2_enc_steps))<<" speeds    "<<temp1+speed<<"   "<<speed1<<"   "<<speed2+temp2<<std::endl;
			
			velocity_publisher.publish(vel_msg_);
			velocity_publisher_1.publish(vel_msg_1);	


			ros::spinOnce();
			//loop_rate.sleep();
			prev_e1=e;
			total_error_1=total_error_1+e;

			prev_e2=e1;
			total_error_2=total_error_2+e1;

			prev_e3=e2;
			total_error_3=total_error_3+e2;
		}
	//ros::Duration(0.1).sleep();
	}

/*vel_msg_.mot1_vel_sps=0;
vel_msg_.mot2_vel_sps=0;
velocity_publisher.publish(vel_msg_);

vel_msg_1.mot2_vel_sps=0;
velocity_publisher_1.publish(vel_msg_1);*/



//std::cout<<"DONE"<<std::endl;
DoneString.data="Done";
DonePublisher.publish(DoneString);

//ros::spinOnce();
//std::cout<<"loop ended"<<std::endl;
//std::cout<<"hi"<<std::endl;

}



void Rover::ForwardKinematics(){
	ros::spinOnce();
	auto w1=float(pose.mot1_enc_steps);
	auto w2=-float(pose_1.mot2_enc_steps);
	auto w3=-float(pose.mot2_enc_steps);

	


	
	
	geometry_msgs::Pose2D temp;
	temp.x=((4503599627370496.0*w1)/25397388702750567.0) - ((1501205072273259.0*w2)/16931592468500378.0) - ((6004778717228287.0*w3)/67726369874001512.0);
    temp.y=-((w1/67726369874001512.0) - ((5196483093625199.0*w2)/33863184937000756.0) + ((1299120773406300.0*w3)/8465796234250189.0));
 	temp.theta=(- (562949953421312.0*w1)/1904804152706292525.0) - ((5124113313359391.0*w2)/17337950687744387072.0) - ((160127432459421.0*w3)/541810958992012096.0);
 	

 	temp.theta=temp.theta*180.0/3.14159 *angle_correction_factor  ;
 	FKPose=temp;
	//std::cout<<temp.x<<"  "<<temp.y<<"  "<<temp.theta<<std::endl;
	ForwardKinematicsPositionPublisher.publish(temp);

}



void Rover::ForwardKinematics(float w1,float w2,float w3){
	

	


	
	
	geometry_msgs::Pose2D temp;
	temp.x=((4503599627370496.0*w1)/25397388702750567.0) - ((1501205072273259.0*w2)/16931592468500378.0) - ((6004778717228287.0*w3)/67726369874001512.0);
    temp.y=-((w1/67726369874001512.0) - ((5196483093625199.0*w2)/33863184937000756.0) + ((1299120773406300.0*w3)/8465796234250189.0));
 	temp.theta=(- (562949953421312.0*w1)/1904804152706292525.0) - ((5124113313359391.0*w2)/17337950687744387072.0) - ((160127432459421.0*w3)/541810958992012096.0);
 	

 	temp.theta=temp.theta*180.0/3.14159 *angle_correction_factor  ;

 	ROS_INFO_STREAM(temp);
 	// FKPose=temp;
	//std::cout<<temp.x<<"  "<<temp.y<<"  "<<temp.theta<<std::endl;
	// ForwardKinematicsPositionPublisher.publish(temp);

}




void Rover::InitializeMatrices(){

	ROS_INFO_STREAM("Start");
	B.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	A.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	Xn_1.resize(3);
	ROS_INFO_STREAM("Bdone");
	W_prev.resize(3);
	ROS_INFO_STREAM("Bdone");
	Pk_1.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	H.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	M.resize(1,1);
	ROS_INFO_STREAM("Bdone");
	Q.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	R.resize(3,3);
	ROS_INFO_STREAM("Bdone");
	I3x3.resize(3,3);
	ROS_INFO_STREAM("Bdone");


	B(0,0)=(4503599627370496.0)/25397388702750567.0*1.0/1000.0;
	B(0,1)= -(1501205072273259.0)/16931592468500378.0*1.0/1000.0;
	B(0,2)= -((6004778717228287.0)/67726369874001512.0)*1.0/1000.0;

	B(1,0)=(1/67726369874001512.0)*-1.0*1.0/1000.0;
	B(1,1)= -((5196483093625199.0)/33863184937000756.0)*-1.0*1.0/1000.0;
	B(1,2)= ((1299120773406300.0)/8465796234250189.0)*-1.0*1.0/1000.0;


	B(2,0)= -(562949953421312.0)/1904804152706292525.0*angle_correction_factor;
	B(2,1)= - ((5124113313359391.0)/17337950687744387072.0)*angle_correction_factor;
	B(2,2)= -(160127432459421.0)/541810958992012096.0*angle_correction_factor;


	std::cout<<"############## B #############"<<B<<std::endl;

	

	
	A(0,0)=1;A(0,1)=0;A(0,2)=0;
	A(1,0)=0;A(1,1)=1;A(1,2)=0;
	A(2,0)=0;A(2,1)=0;A(2,2)=1;


	std::cout<<"############### A #############"<<A<<std::endl;


	Pk_1(0,0)=0.000001;Pk_1(0,1)=0;Pk_1(0,2)=0;
	Pk_1(1,0)=0;Pk_1(1,1)=0.000001;Pk_1(1,2)=0;
	Pk_1(2,0)=0;Pk_1(2,1)=0;Pk_1(2,2)=0.000001;



	std::cout<<"############ P initial #############"<<Pk_1<<std::endl;


	H(0,0)=1;H(0,1)=0;H(0,2)=0;
	H(1,0)=0;H(1,1)=1;H(1,2)=0;    //////////////Get values of transformation
	H(2,0)=0;H(2,1)=0;H(2,2)=1;

	std::cout<<"############ H initial #############"<<H<<std::endl;

	
	M(0,0)=1;


	std::cout<<"############ M initial #############"<<M<<std::endl;


	R<< 0.00001, 0, 0,
		0, 0.00001, 0,     /////////////for obs   //being called only 
		0, 0, 0.00001;

	std::cout<<"############ R initial #############"<<R<<std::endl;


	Q<< 0.00002, 0, 0,
		0, 0.00002, 0,     					// for pred                     
		0, 0, 0.00002;


	std::cout<<"############ Q initial #############"<<Q<<std::endl;

	I3x3=A;

	std::cout<<"############ I3x3 initial #############"<<I3x3<<std::endl;

	Y.resize(3);
	Y<<0,0,0;
	D.resize(3);
	D<<0,0,0;    //////////////////Update from calibration
	W_prev<<0,0,0;


}





//
// Gives pose of robot wrt robot_start 
//
void Rover::FK(){

	ros::spinOnce();
	auto w1_current=float(pose.mot1_enc_steps);
	auto w2_current=-float(pose_1.mot2_enc_steps);
	auto w3_current=-float(pose.mot2_enc_steps);

	Eigen::Vector3d W;
	W.resize(3);


	
	W<<w1_current-W_prev(0),
	w2_current-W_prev(1),
	w3_current-W_prev(2);

	
	auto X= A*Xn_1 + 
			B*W;
	

	Xn_1=X;



	W_prev<<w1_current,w2_current,w3_current;

	
	//Update Xn-1 from KF step and update W_prev to current rotations when KF calc
}



void Rover::FKVROriginFrame(){



	// ros::spinOnce();
	auto w1_current=float(pose.mot1_enc_steps);
	auto w2_current=-float(pose_1.mot2_enc_steps);
	auto w3_current=-float(pose.mot2_enc_steps);
	Eigen::Vector3d W;
	W.resize(3);


	
	W<<w1_current-W_prev(0),
	w2_current-W_prev(1),
	w3_current-W_prev(2);


	auto X= A*Xn_1 + 
			B*W;

	



	Xn_1=X;
	W_prev<<w1_current,w2_current,w3_current;

	Pk_1 = A*Pk_1*A.transpose() + I3x3*Q*I3x3.transpose();

	// std::cout<<X<<std::endl<<std::endl;
	// ROS_ERROR_STREAM("PREDICTION...");
	// ROS_ERROR_STREAM(Pk_1);


	try{
		auto kf_pose_robot_frame_in_robot_initial_frame=TF_->ConvertVectorToPose({X(0),X(1),X(2)});
		kf_pose_robot_frame_in_robot_initial_frame.position.z=lastObsPose.position.z;
		TF_->publishFrame(kf_pose_robot_frame_in_robot_initial_frame,"robot_frame_kf","robot_initial_frame_1");
        TF_->publishFrame(kf_pose_robot_frame_in_robot_initial_frame,"robot_frame_kf","robot_initial_frame_2");
	}

	catch(...){
		ROS_ERROR_STREAM("cannot push robot_frame_kf to tf2");
	}



	
	//Update Xn-1 from KF step and update W_prev to current rotations when KF calc
}


//void Rover::CalculateError(){
//    //
//}
//Publishes frame robot_frame_KF
void Rover::KalmanFilter(){
	//ROS_INFO_STREAM("KalmanFilter");
	
	// FK();
	// FKVROriginFrame();

	

	//Xn-1 is in robot_initial_frame

	//convert this to origin

//	CalculateError();

	auto pose_in_robot_initial=TF_->ConvertVectorToPose( {double(Xn_1(0)),double(Xn_1(1)),double(Xn_1(2))} );   //in m rad from prediction

	lastObsPose=pose_in_robot_initial;

	// std::cout<<" Prediction pose  "<<Xn_1<<std::endl<<std::endl;

	// auto Pkest=A*Pk_1*A.transpose() + I3x3*Q*I3x3.transpose();             //include transformation in A that happens by tf
	// ROS_INFO_STREAM("prediction1 done");

	// std::cout<<Pkest<<std::endl<<std::endl;

	auto K= Pk_1* H.transpose()*((H*Pk_1*H.transpose() + R ).inverse());   //M is 1    include transformation in H that happens by tf
	// ROS_INFO_STREAM("Kdone");
	// std::cout<<K<<std::endl<<std::endl;


	{
		//ROS_INFO_STREAM("Kdone");
		//ROS_INFO_STREAM("prediction done");
		//////////////////
		////////////////
		// Get VR transform values
		//Y is updated in subsriber
		//////////////////////////
		////////////////////////
		

		
		// float error1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5));
		// float error2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5));
		// float error3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.01));
		// Eigen::Vector3d Er;
		// Er.resize(3);
		// Er<<error1,error2,error3;
	}


	//converts prediction into observation
	auto Yest=H*(Xn_1)+D;                      //D has to have value of ini angle but not in full tf case
	// std::cout<<"Yest "<<Yest<<std::endl<<std::endl;


	// auto Y=H*(Xn_1)+D;                            //will be from vr controller get robot frame from tf

    for(int i=0;i<2;i++) {
        //actual obs
        geometry_msgs::Pose robot_observed_pose;

        //do this better
        robot_observed_pose = TF_->getInFrame(transformListener,
                                              TF::MakeGeometryMsgsPose(0, 0, 0, 0, 0, 0, 1),
                                              "/robot_frame_" + std::to_string(i+1),
                                              "/robot_initial_frame_" + std::to_string(i+1));


        auto Y = TF_->PosetoEigenVector3d(robot_observed_pose);
        prev_obs_pose = robot_observed_pose;

        // std::cout<<"Y "<<Y<<std::endl<<std::endl;
        FKVROriginFrame();
        auto error_estimates=CalculateError();
        //ROS_INFO_STREAM(std::to_string(error_estimates[0]) + " " + std::to_string(error_estimates[1]) + " " + std::to_string(error_estimates[2]) );
        if(error_estimates[1]>0.1) {

            if(i==0    && error_estimates[0]>error_estimates[2]) {
                ROS_INFO_STREAM(std::to_string(error_estimates[0]) + " " + std::to_string(error_estimates[1]) + " "
                                    + std::to_string(error_estimates[2]) + "---FAIL");
                ROS_ERROR_STREAM("Tracker 1 is unavailable  !!!");
                continue;
            }
            if(i==1    && error_estimates[0] < error_estimates[2]) {
                ROS_INFO_STREAM(std::to_string(error_estimates[0]) + " " + std::to_string(error_estimates[1]) + " "
                                    + std::to_string(error_estimates[2]) + "---FAIL");
                ROS_ERROR_STREAM("Tracker 2 is unavailable  !!!");
                continue;
            }
        }

        ROS_INFO_STREAM(std::to_string(error_estimates[0]) + " " + std::to_string(error_estimates[1]) + " "
                            + std::to_string(error_estimates[2]));


        // std::cout<<"Y-Yest "<<Y-Yest<<std::endl<<std::endl<<std::endl;

        ROS_INFO_STREAM("@@@@@@@@@@@@@@@@@@@@@@@@@@@@"+std::to_string(i));
        auto X = Xn_1 + K * (Y - Yest);                                        //is in robot_initial




        // std::cout<<"X "<<X<<std::endl<<std::endl<<std::endl;
        Xn_1 = X;

        try {
//            ROS_INFO_STREAM("------------Publishing without error:"+ std::to_string(i));
            auto kf_pose_robot_frame_in_robot_initial_frame = TF_->ConvertVectorToPose({X(0), X(1), X(2)});
            kf_pose_robot_frame_in_robot_initial_frame.position.z = lastObsPose.position.z;
            TF_->publishFrame(kf_pose_robot_frame_in_robot_initial_frame, "robot_frame_kf", "robot_initial_frame_"+std::to_string(i+1));
        }

        catch (...) {
            ROS_ERROR_STREAM("cannot push robot_frame_kf to tf2");
        }
    }







	auto Pmeas=(I3x3-K*H)*Pk_1;
	// std::cout<<"Pmeas "<<Pmeas<<std::endl<<std::endl;

	Pk_1=Pmeas;

	// std::cout<<X<<std::endl<<std::endl;
	// ROS_ERROR_STREAM("observation...");
	// ROS_ERROR_STREAM(Pk_1);

	//Here we have X i.e. robot_frame in origin in m rad



	/////////////////
	//Publish X
	///////////////////

	//convert X back to robot initial frame and submit back a vector 3d
	
	




	kf_counter++;
	

	// ROS_INFO_STREAM("####################################");



}







std::vector<double> Rover::CalculateError(){

    //robot_frame_1
    //robot_frame_2
    //robot_enc
    FKVROriginFrame();

    auto rb_eig_enc = Xn_1;
    //Xn_1=//from enc


    auto rbf1 = TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(0,0,0, 0,0,0,1),  "/robot_frame_1" , "/robot_initial_frame_1");

    auto rbf2 = TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(0,0,0, 0,0,0,1),  "/robot_frame_2" , "/robot_initial_frame_2");

    auto rb_enc = TF_->ConvertVectorToPose({rb_eig_enc(0),rb_eig_enc(1),rb_eig_enc(2)});


//    ROS_INFO_STREAM("RBF1-init1");ROS_INFO_STREAM(rbf1);ROS_INFO_STREAM("");ROS_INFO_STREAM("");
//    ROS_INFO_STREAM("RBF2-init2");ROS_INFO_STREAM(rbf2);ROS_INFO_STREAM("");ROS_INFO_STREAM("");
    ROS_INFO_STREAM("RBFec-init");ROS_INFO_STREAM(rb_enc);


    auto rbf1_e_rbf2 = EulerDistance(rbf1,rbf2); //+ sqrt(pow((rbf1.orientation.x-rbf1.orientation.x),2);
    auto rbf1_e_rb_enc = EulerDistance(rbf1,rb_enc);
    auto rbf2_e_rb_enc = EulerDistance(rbf2,rb_enc);

    return {rbf1_e_rb_enc, rbf1_e_rbf2, rbf2_e_rb_enc};









}



// IK, give x,y,t wrt robot initial outputs wheel posns
//w in rad
geometry_msgs::Pose Rover::matrixCalculation(float x, float y, float w)           //////Inv Kinematics calculation function for rover
{    float body_twist;
    float body_velocity_x;
    float body_velocity_y;
    float wheel_radius;
    float wheel_distance; //chassis radius
    float motor_angVel_1; //angular velocity
    float motor_angVel_2;
    float motor_angVel_3;
    body_twist = w;
    body_velocity_x = x;
    body_velocity_y = y;
    wheel_radius = 101.6; /*in mms*/
    wheel_distance = 300; /*in mms*/
    motor_angVel_1 = ((-1*wheel_distance*body_twist) + body_velocity_x)/wheel_radius;
    motor_angVel_2 = ((-1*wheel_distance*body_twist) - (0.5*body_velocity_x) - (0.866*body_velocity_y))/wheel_radius;
    motor_angVel_3 = ((-1*wheel_distance*body_twist) - (0.5*body_velocity_x) + (0.866*body_velocity_y))/wheel_radius;
    	
	geometry_msgs::Pose p;


    p.position.x=(motor_angVel_1)*180/3.14159*2400/360;
    p.position.y=(motor_angVel_2)*180/3.14159*2400/360;
    p.position.z=(motor_angVel_3)*180/3.14159*2400/360;


    return p;
}

