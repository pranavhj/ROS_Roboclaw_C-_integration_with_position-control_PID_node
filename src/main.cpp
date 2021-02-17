
#include "ros/ros.h"
#include "rover.h"
#include "remote.h"

using namespace std;









int main(int argc, char **argv)
{
ros::init(argc,argv,"main_node");
ros::NodeHandle n("~");

bool free;
    
auto args=n.getParam("free", free);

cout << free << " "<<args<<endl;
if(args==0){
	ROS_INFO_STREAM("Wrong argument passed, please pass correct arg and run again");
	ROS_INFO_STREAM("correct args are _free:=false _free:=true");
	return 0;

}



bool cmd_vel;
    
auto args1=n.getParam("cmd_vel", cmd_vel);

cout << cmd_vel << " "<<args1<<endl;
if(args1==0){
	ROS_INFO_STREAM("Wrong argument passed, please pass correct arg and run again");
	ROS_INFO_STREAM("correct args are _cmd_vel:=false _cmd_vel:=true");
	return 0;

}



ros::Rate rate(10);
std::cout<<"Started MAIN"<<std::endl;
Rover *rover=new Rover(&n);
Remote *remote=new Remote(&n);
rover->SetRemote(remote);
while(ros::ok()){
	//remote->Execute();
	//rover->ExecuteIK();
	if(!free){
		//rover->ExecuteCMDVELNoInterpolation();

		if (!cmd_vel){
			rover->KalmanFilter();
			rover->ExecuteIKOnlySpeed();
		}
		else{
			rover->KalmanFilter();
			// rover->ExecuteCMDVELNoInterpolation();
			rover->ExecuteCMDVEL();
		}

		// auto w=rover->GetEncoderPosnsFromTF();
		// ROS_INFO_STREAM(w);
	}
	else{

	 	double start =ros::Time::now().toSec();
		// rover->ForwardKinematics();
		// rover->FKVROriginFrame();
		/*while(ros::Time::now().toSec()-start<0.09){
			ros::spinOnce();

		}*/

		rover->KalmanFilter();
		
		ros::spinOnce();
	}
	//rover->GotoPosition(remote->GetRemoteState());
	//ROS_INFO_STREAM(remote->GetRemoteState());
	ros::spinOnce();
	}
}
/*


//cout<<position.position_1<<"    "<<position.position_2<<"  "<<position.position_3<<endl;
while(ros::ok())
{//cout<<"Start"<<endl;
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


ros::spinOnce();}*/
//cout<<pose.mot1_enc_steps<<endl;





