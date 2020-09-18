#include "rover.h"



Rover::Rover(ros::NodeHandle *n){
	velocity_publisher=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel",10);
	velocity_publisher_1=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel_1",10);
	ForwardKinematicsPositionPublisher=n->advertise<geometry_msgs::Pose2D>("/robot_odom_position",10);
	DonePublisher=n->advertise<std_msgs::String>("/DoneTopic",10);

	
	pose_subscriber=n->subscribe("/motor_enc",100,&Rover::poseCallback,this);
	position_subscriber=n->subscribe("/position",100,&Rover::positionCallback,this);     //given by inverse kinema	
	pose_subscriber_1=n->subscribe("/motor_enc_1",10,&Rover::pose_1Callback,this);
	inv_kinematics_subscriber=n->subscribe("/inv_kinematics",10,&Rover::IKCallback,this);




	ros::spinOnce();
	ForwardKinematics();              //so that it does not move to zero from initial pose
	IKpose=FKPose;



	
}

void Rover::IK(){              //takes IK pose and converts to position values
	float x=IKpose.x;
    float y=IKpose.y;
    float w=IKpose.theta;
    
	w=w*3.14159/180;
    float *targets=matrixCalculation(x,y,w);

    motor_controller::position position;
    position.position_1=int(*(targets));
    position.position_2=int(*(targets+1));
    position.position_3=int(*(targets+2));//angle in deg
    //inv_kinematics_publisher.publish(position);
    //cout<<"Published"<<endl;
    ros::spinOnce();
}


void Rover::ExecuteIK(){
	//std::cout<<position.position_1<<"    "<<position.position_2<<"  "<<position.position_3<<std::endl;
	IK();
	//cout<<"Start"<<endl;
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


	ros::spinOnce();

//cout<<pose.mot1_enc_steps<<endl;
}

void Rover::poseCallback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	pose.index=pose_message->index;
	pose.mot1_enc_steps=pose_message->mot1_enc_steps;
	pose.mot2_enc_steps=pose_message->mot2_enc_steps;
	//PIDcontroller(position.position_1,position.position_2);
	

}


void Rover::pose_1Callback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	pose_1.index=pose_message->index;
	pose_1.mot1_enc_steps=pose_message->mot1_enc_steps;
	pose_1.mot2_enc_steps=pose_message->mot2_enc_steps;
	//PIDcontroller(position.position_1,position.position_2);
	

}

void Rover::positionCallback(const motor_controller::position::ConstPtr& position_message){
	position.position_1=position_message->position_1;
	position.position_2=-position_message->position_2;
	position.position_3=-position_message->position_3;
	//std::cout<<position.position_1<<"  "<<position.position_2<<"  "<<position.position_3<<std::endl;
	total_error_1=0;
	total_error_2=0;
	total_error_3=0;	
	//PIDcontroller(position.position_1,position.position_2);
	
	

}


void Rover::IKCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message)
{   IKpose=*(pose_message);
}

void Rover::PIDcontroller(float goal, float goal1,float goal2)
{


roboclaw::RoboclawMotorVelocity vel_msg_; 
roboclaw::RoboclawMotorVelocity vel_msg_1;
vel_msg_.index=0;
vel_msg_1.index=0;
goal=position.position_1;
goal1=position.position_2;
goal2=position.position_3;
float e_current=goal-float(pose.mot1_enc_steps);
float e_current_1=goal1-float(pose_1.mot2_enc_steps);
float e_current_2=goal2-float(pose.mot2_enc_steps);


while((abs((goal)-(float(pose.mot1_enc_steps)))>10.0        ||    abs((goal1)-(float(pose_1.mot2_enc_steps)))>10.0    ||   abs((goal2)-(float(pose.mot2_enc_steps)))>10.0)  &&   ros::ok() )
	{
	ForwardKinematics();
	if(abs((goal)-(float(pose.mot1_enc_steps)))>500     &&   abs((goal1)-(float(pose_1.mot2_enc_steps)))>500    &&   abs((goal2)-(float(pose.mot2_enc_steps)))>500)
		{	ros::spinOnce();
			goal=position.position_1;
			goal1=position.position_2;
			goal2=position.position_3;
			e_current=goal-float(pose.mot1_enc_steps);
			e_current_1=goal1-float(pose_1.mot2_enc_steps);
			e_current_2=goal2-float(pose.mot2_enc_steps);
			int max_speed_=abs(e_current_1)*3.5;
			if (max_speed_>full_speed)
				max_speed_=full_speed;
			int speed=0;
			int speed1=0;
			int speed2=0;
			if (abs(e_current) >= abs(e_current_1)   and  abs(e_current) >= abs(e_current_2)  )
				{speed=max_speed_*e_current/abs(e_current);
				speed1=max_speed_*abs(e_current_1/e_current)*e_current_1/abs(e_current_1);
				speed2=max_speed_*abs(e_current_2/e_current)*e_current_2/abs(e_current_2);

				}

			else if (abs(e_current_1) >= abs(e_current_2)   and  abs(e_current_1) >= abs(e_current))
				{speed1=max_speed_*e_current_1/abs(e_current_1);
				 speed=max_speed_*abs(e_current/e_current_1)*e_current/abs(e_current);
				 speed2=max_speed_*abs(e_current_2/e_current_1)*e_current_2/abs(e_current_2);
				}

			else if (abs(e_current_2) >= abs(e_current)   and  abs(e_current_2) >= abs(e_current_1))
				{speed2=max_speed_*e_current_2/abs(e_current_2);
				 speed=max_speed_*abs(e_current/e_current_2)*e_current/abs(e_current);
				 speed1=max_speed_*abs(e_current_1/e_current_2)*e_current_1/abs(e_current_1);
				}			

			//std::cout<<"SpeedLoop"<<"   "<<speed<<"  "<<speed1<<"  "<<speed2<<std::endl;
			DoneString.data="SpeedLoop";
			DonePublisher.publish(DoneString);
			vel_msg_.mot1_vel_sps=int(1*speed);
			vel_msg_1.mot2_vel_sps=int(1*speed1);
			vel_msg_.mot2_vel_sps=int(1*speed2);
		
			velocity_publisher.publish(vel_msg_);
			velocity_publisher_1.publish(vel_msg_1);

			
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

		if (speed>max_speed)
			speed=max_speed;
		else if(speed<-max_speed)
			speed=-max_speed;

		if (speed1>max_speed)
			speed1=max_speed;
		else if(speed1<-max_speed)
			speed1=-max_speed;

		if (speed2>max_speed)
			speed2=max_speed;
		else if(speed2<-max_speed)
			speed2=-max_speed;

		int temp1=00,temp2=00;
		// if (e<0)
		// 	temp1=-temp1;
		// if (e2<0)
		// 	temp2=-temp2;
		
	 //   if (speed>3000    &&   speed1>3000      &&    speed2>3000)
	 //   		{speed=speed*(abs(e_current)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));
	 //   		speed1=speed1*(abs(e_current_1)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));
	 //   		speed2=speed2*(abs(e_current_2)/(abs(e_current)+abs(e_current_1)+abs(e_current_2)));}

		vel_msg_.mot1_vel_sps=int(speed);
		vel_msg_1.mot2_vel_sps=int(speed1);
		vel_msg_.mot2_vel_sps=int(speed2);
		//vel_msg_1.mot1_vel_sps=1*speed;                    ///Just for checking


		
		//std::cout<<speed<<"   "<<speed1<<"   "<<speed2<<std::endl;
		//std::cout<<abs(goal)-abs(float(pose.mot1_enc_steps))<<"    "<<abs(goal1)-abs(float(pose_1.mot2_enc_steps))<<"    "<<abs(goal2)-abs(float(pose.mot2_enc_steps))<<" speeds    "<<temp1+speed<<"   "<<speed1<<"   "<<speed2+temp2<<std::endl;
		
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

vel_msg_.mot1_vel_sps=0;
vel_msg_.mot2_vel_sps=0;
velocity_publisher.publish(vel_msg_);

vel_msg_1.mot2_vel_sps=0;
velocity_publisher_1.publish(vel_msg_1);



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

	/*auto r=101.6;auto R=300;
	auto m=180.0*2400.0*r/360.0/3.14159;m=1/m;
	float cos30=sqrt(3)/2;

	geometry_msgs::Pose2D temp;
	temp.x=((-w1/2.0)+(-w2/2.0)+(w3))//m*((2.00/3.0*w1)-(1.0/3.0*w2)-(1.0/3.0*w3));
	temp.y=m*((0*w1)-(250.0/433.0*w2)+(250.0/433.0*w3));
	temp.theta=m/R/3*(-w1-w2-w3);*/



	geometry_msgs::Pose2D temp;
	temp.x=((4503599627370496.0*w1)/25397388702750567.0) - ((1501205072273259.0*w2)/16931592468500378.0) - ((6004778717228287.0*w3)/67726369874001512.0);
    temp.y=(w1/67726369874001512.0) - ((5196483093625199.0*w2)/33863184937000756.0) + ((1299120773406300.0*w3)/8465796234250189.0);
 	temp.theta=(- (562949953421312.0*w1)/1904804152706292525.0) - ((5124113313359391.0*w2)/17337950687744387072.0) - ((160127432459421.0*w3)/541810958992012096.0);
 	temp.theta=temp.theta*180.0/3.14159;
 	FKPose=temp;
	//std::cout<<temp.x<<"  "<<temp.y<<"  "<<temp.theta<<std::endl;
	ForwardKinematicsPositionPublisher.publish(temp);

}

float* Rover::matrixCalculation(float x, float y, float w)
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
    targets[0]=(motor_angVel_1)*180/3.14159*2400/360;
    targets[1]=(motor_angVel_2)*180/3.14159*2400/360;
    targets[2]=(motor_angVel_3)*180/3.14159*2400/360;
    return &targets[0];    
}




