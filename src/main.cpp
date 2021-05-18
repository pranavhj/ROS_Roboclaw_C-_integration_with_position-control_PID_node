
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
			rover->KalmanFilter();          /// This function combines readings from robot odometry and both trackers to get optimal pose of robot


			//rover->ExecuteIKOnlySpeed();               /// This is a function that i tried that tries to remove the imperfect initial motion towards goal which did not give good results
		}
		else{
			rover->KalmanFilter();


			// rover->ExecuteCMDVELNoInterpolation(); //  This is a function to make robot move without interpolating the speeds to final speeds for each wheel
			// rover->ExecuteCMDVELNoInterpolation();  /// this function puts large jerk on motors and wheels due to high changes in accelaration
			

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






