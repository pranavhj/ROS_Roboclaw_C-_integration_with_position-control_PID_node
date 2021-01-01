#include "rover.h"



Rover::Rover(ros::NodeHandle *n){
	InitializeMatrices();
	velocity_publisher=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel",10);
	velocity_publisher_1=n->advertise<roboclaw::RoboclawMotorVelocity>("/motor_cmd_vel_1",10);
	ForwardKinematicsPositionPublisher=n->advertise<geometry_msgs::Pose2D>("/robot_odom_position",10);
	DonePublisher=n->advertise<std_msgs::String>("/DoneTopic",10);

	
	pose_subscriber=n->subscribe("/motor_enc",100,&Rover::poseCallback,this);
	position_subscriber=n->subscribe("/position",100,&Rover::positionCallback,this);     //given by inverse kinema	
	pose_subscriber_1=n->subscribe("/motor_enc_1",10,&Rover::pose_1Callback,this);
	inv_kinematics_subscriber=n->subscribe("/inv_kinematics",10,&Rover::IKCallback,this);
	inv_kinematics_publisher=n->advertise<motor_controller::position>("/position",10);
	cmd_vel_subscriber=n->subscribe("/cmd_vel",1,&Rover::CMDVELCallback,this);


	ros::spinOnce();
	ros::Duration(0.5).sleep();
	

	ForwardKinematics();              //so that it does not move to zero from initial pose
	IKpose=FKPose;
	ROS_INFO_STREAM(IKpose);
	ros::Duration(0.5).sleep();
	
}



void Rover::poseCallback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	//std::cout<<"poseCallback";
	//ROS_INFO_STREAM(*pose_message);
	pose.index=pose_message->index;
	pose.mot1_enc_steps=pose_message->mot1_enc_steps;
	pose.mot2_enc_steps=pose_message->mot2_enc_steps;
	//PIDcontroller(position.position_1,position.position_2);
	
}


void Rover::CMDVELCallback(const geometry_msgs::Twist::ConstPtr& pose_message){
	//std::cout<<"poseCallback";
	//ROS_INFO_STREAM(*pose_message);
	cmd_vel=*pose_message;
	//ROS_INFO_STREAM(cmd_vel);	
		
	//PIDcontroller(position.position_1,position.position_2);
	
}

void Rover::pose_1Callback(const roboclaw::RoboclawEncoderSteps::ConstPtr& pose_message){
	//std::cout<<"pose_1Callback";
	//ROS_INFO_STREAM(*pose_message);
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


void Rover::IKCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message){   
	//IKpose=*(pose_message);
	IKpose.x+=(pose_message->x/5000);
	IKpose.y+=(pose_message->y/5000);
	IKpose.theta+=(pose_message->theta/300);
	
	//std::cout<<"IKCallback"<<std::endl;
}


void Rover::IK(){              //takes IK pose and converts to position values
	
	ros::spinOnce();
	float x=IKpose.x;
    float y=IKpose.y;
    float w=IKpose.theta;
    
	w=w*3.14159/180;
    float *targets=matrixCalculation(x,y,w);

    motor_controller::position position;
    position.position_1=int(*(targets));
    position.position_2=int(*(targets+1));
    position.position_3=int(*(targets+2));//angle in deg
    inv_kinematics_publisher.publish(position);
    //cout<<"Published"<<endl;
    //ROS_INFO_STREAM(position);
    
}


void Rover::ExecuteIK(){
	//std::cout<<position.position_1<<"    "<<position.position_2<<"  "<<position.position_3<<std::endl;
	IK();
	//cout<<"Start"<<endl;
	//ROS_INFO_STREAM(position);
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

float Euler(geometry_msgs::Pose2D finalState,geometry_msgs::Pose2D prevState){
	return float(sqrt((finalState.x*finalState.x)+(finalState.y*finalState.y))-sqrt((prevState.x*prevState.x)+(prevState.y*prevState.y)));
	}




void Rover::ExecuteCMDVEL(){
	ros::spinOnce();
	
	
	
	geometry_msgs::Pose2D finalState;
	finalState.x=cmd_vel.linear.x;
	finalState.y=cmd_vel.linear.y;
	finalState.theta=cmd_vel.angular.z;
	//ROS_INFO_STREAM(finalState);
	
	if(abs(Euler(finalState,prevState))>100){
		//divide speeds
		auto temp=finalState;
		float tempxval=10;
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
	float *targets=matrixCalculation(finalState.x,finalState.y,finalState.theta);
	
	//ROS_INFO_STREAM(finalState);
	//std::cout<<*targets<<" "<<*(targets+1)<<" "<<*(targets+2)<<std::endl;
	auto speed=*targets;//pos1/=10000;
	auto speed1=*(targets+1);//pos2/=10000;
	auto speed2=*(targets+2);//pos3/=10000;
	
	ros::spinOnce();
	ForwardKinematics();
	
	/*
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




void Rover::GotoPosition(geometry_msgs::Pose2D remoteState){/////////////remote state is in world frame


	
	auto finalState=ConvertPosition(remoteState);
	if(abs(Euler(finalState,prevState))>100){
		//divide speeds
		auto temp=finalState;
		float tempxval=100;
		if(abs(finalState.x-prevState.x)>abs(finalState.y-prevState.y)){
				
				if(abs(finalState.x-prevState.x)<100){
					tempxval=finalState.x-prevState.x;
					}
				else{
					tempxval=(finalState.x-prevState.x)/abs(finalState.x-prevState.x)*tempxval;
					}
				finalState.x=prevState.x+tempxval;
				finalState.y=prevState.y+((temp.y-prevState.y)/abs(temp.x-prevState.x)*abs(tempxval));
			
				ros::Duration(0.05).sleep();
				
			}
		else{
				if(abs(finalState.y-prevState.y)<100){
					tempxval=finalState.y-prevState.y;
					}
				else{
					tempxval=(finalState.y-prevState.y)/abs(finalState.y-prevState.y)*tempxval;
					}
				finalState.y=prevState.y+tempxval;
				//if(finalState.x==0)
				finalState.x=prevState.x+((temp.x-prevState.x)/abs(temp.y-prevState.y)*abs(tempxval));
			
				ros::Duration(0.05).sleep();
				}


		
		}






	prevState=finalState;
	float *targets=matrixCalculation(finalState.x,finalState.y,finalState.theta);
	
	ROS_INFO_STREAM(finalState);
	//std::cout<<*targets<<" "<<*(targets+1)<<" "<<*(targets+2)<<std::endl;
	auto speed=*targets;//pos1/=10000;
	auto speed1=*(targets+1);//pos2/=10000;
	auto speed2=*(targets+2);//pos3/=10000;
	
	ros::spinOnce();
	ForwardKinematics();
	
	/*
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


}





void Rover::PIDcontroller(float goal, float goal1,float goal2){
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

	/*auto r=101.6;auto R=300;
	auto m=180.0*2400.0*r/360.0/3.14159;m=1/m;
	float cos30=sqrt(3)/2;

	geometry_msgs::Pose2D temp;
	temp.x=((-w1/2.0)+(-w2/2.0)+(w3))//m*((2.00/3.0*w1)-(1.0/3.0*w2)-(1.0/3.0*w3));
	temp.y=m*((0*w1)-(250.0/433.0*w2)+(250.0/433.0*w3));
	temp.theta=m/R/3*(-w1-w2-w3);*/

	//std::cout<<w1<<w2<<w3<<std::endl;

	geometry_msgs::Pose2D temp;
	temp.x=((4503599627370496.0*w1)/25397388702750567.0) - ((1501205072273259.0*w2)/16931592468500378.0) - ((6004778717228287.0*w3)/67726369874001512.0);
    temp.y=(w1/67726369874001512.0) - ((5196483093625199.0*w2)/33863184937000756.0) + ((1299120773406300.0*w3)/8465796234250189.0);
 	temp.theta=(- (562949953421312.0*w1)/1904804152706292525.0) - ((5124113313359391.0*w2)/17337950687744387072.0) - ((160127432459421.0*w3)/541810958992012096.0);
 	temp.theta=temp.theta*180.0/3.14159;
 	FKPose=temp;
	//std::cout<<temp.x<<"  "<<temp.y<<"  "<<temp.theta<<std::endl;
	ForwardKinematicsPositionPublisher.publish(temp);

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


	B(0,0)=(4503599627370496.0)/25397388702750567.0;
	B(0,1)= -(1501205072273259.0)/16931592468500378.0;
	B(0,2)= -((6004778717228287.0)/67726369874001512.0);

	B(1,0)=(1/67726369874001512.0);
	B(1,1)= -((5196483093625199.0)/33863184937000756.0);
	B(1,2)= ((1299120773406300.0)/8465796234250189.0);


	B(2,0)= -(562949953421312.0)/1904804152706292525.0;
	B(2,1)= - ((5124113313359391.0)/17337950687744387072.0);
	B(2,2)= -(160127432459421.0)/541810958992012096.0;


	std::cout<<"############## B #############"<<B<<std::endl;

	

	
	A(0,0)=1;A(0,1)=0;A(0,2)=0;
	A(1,0)=0;A(1,1)=1;A(1,2)=0;
	A(2,0)=0;A(2,1)=0;A(2,2)=1;


	std::cout<<"############### A #############"<<A<<std::endl;


	Pk_1(0,0)=1;Pk_1(0,1)=0;Pk_1(0,2)=0;
	Pk_1(1,0)=0;Pk_1(1,1)=1;Pk_1(1,2)=0;
	Pk_1(2,0)=0;Pk_1(2,1)=0;Pk_1(2,2)=1;



	std::cout<<"############ P initial #############"<<Pk_1<<std::endl;


	H(0,0)=1;H(0,1)=0;H(0,2)=0;
	H(1,0)=0;H(1,1)=1;H(1,2)=0;    //////////////Get values of transformation
	H(2,0)=0;H(2,1)=0;H(2,2)=1;

	std::cout<<"############ H initial #############"<<H<<std::endl;

	
	M(0,0)=1;


	std::cout<<"############ M initial #############"<<M<<std::endl;


	R<< 25, 0, 0,
		0, 25, 0,     /////////////Increase Q if correction not received for long
		0, 0, 0.001;

	std::cout<<"############ R initial #############"<<R<<std::endl;


	Q<< 0.05, 0, 0,
		0, 0.05, 0,
		0, 0, 0.001;


	std::cout<<"############ Q initial #############"<<Q<<std::endl;

	I3x3=A;

	std::cout<<"############ I3x3 initial #############"<<I3x3<<std::endl;

	Y.resize(3);
	Y<<0,0,0;
	D.resize(3);
	D<<0,0,0;    //////////////////Update from calibration
	W_prev<<0,0,0;







}

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
	

	//std::cout<<X<<std::endl;
	//std::cout<<std::endl<<std::endl;



	Xn_1=X;
	W_prev<<w1_current,w2_current,w3_current;

	
	//Update Xn-1 from KF step and update W_prev to current rotations when KF calc
}

void Rover::KalmanFilter(){
	//ROS_INFO_STREAM("KalmanFilter");
	
	FK();


	
	

	auto Pkest=A*Pk_1*A.transpose() + I3x3*Q*I3x3.transpose();
	//ROS_INFO_STREAM("prediction1 done");

	auto K= Pkest* H.transpose()*((H*Pk_1*H.transpose() + R ).inverse());   //M is 1
	//ROS_INFO_STREAM("Kdone");


	




	//ROS_INFO_STREAM("Kdone");
	//ROS_INFO_STREAM("prediction done");
	//////////////////
	////////////////
	// Get VR transform values
	//Y is updated in subsriber
	//////////////////////////
	////////////////////////
	

	
	float error1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5));
	float error2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/5));
	float error3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.01));
	Eigen::Vector3d Er;
	Er.resize(3);
	Er<<error1,error2,error3;


	auto Yest=H*(Xn_1+Er)+D;                      //D has to have value of ini angle

	auto Y=H*(Xn_1)+D;                            //will be from vr controller



	std::cout<<K<<std::endl<<std::endl<<std::endl;


	auto X=Xn_1+ K*(Y-Yest);
	std::cout<<X<<std::endl<<std::endl<<std::endl;

	auto Pmeas=(I3x3-K*H)*Pkest;

	Pk_1=Pmeas;



	/////////////////
	//Publish X
	///////////////////








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



