
#include "ros/ros.h"
#include "rover.h"
#include "remote.h"

using namespace std;









int main(int argc, char **argv)
{
ros::init(argc,argv,"main_node");
ros::NodeHandle n;
ros::Rate rate(10);

Rover *rover=new Rover(&n);
Remote *remote=new Remote(&n);
while(ros::ok()){
	remote->Execute();
	rover->ExecuteIK();
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





