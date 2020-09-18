#include "remote.h"


Remote::Remote(ros::NodeHandle *n){
    remoteState.header.seq=0;
    
   
    
    ros::Rate rate(10);
    inv_kinematics_publisher=n->advertise<geometry_msgs::Pose2D>("/inv_kinematics",10);
    robot_odom_position_subscriber=n->subscribe("/robot_odom_position",10,&Remote::RobotOdomCallback,this);
    remoteStateSubscriber=n->subscribe("/remote_state",10,&Remote::RemoteStateCallback,this);
    
    ros::spinOnce();

}


void Remote::Execute(){
    if(calibrationDone==false){
            CalibrateRemote();
            return;
        }
        auto leftJoyStickX=remoteState.axes[0]-initialRemoteState.axes[0];
        auto leftJoyStickY=remoteState.axes[1]-initialRemoteState.axes[1];
        auto rightJoystickX=remoteState.axes[2]-initialRemoteState.axes[2];
        auto rightJoystickY=remoteState.axes[3]-initialRemoteState.axes[3];

        geometry_msgs::Pose2D diff;

        if(abs(leftJoyStickX)>noMovementValue){
            diff.theta=leftJoyStickX*0.01;
        }

        if(abs(rightJoystickX)>noMovementValue){
            diff.x=rightJoystickX*1;
        }

        if(abs(rightJoystickY)>noMovementValue){
            diff.y=rightJoystickY*1;
        }

        geometry_msgs::Pose2D finalState;
        finalState.x=robot_odom_position.x+diff.x;
        finalState.y=robot_odom_position.y+diff.y;
        finalState.theta=robot_odom_position.theta+diff.theta;

        //ros::Duration(0.1).sleep();
        ros::spinOnce();
}


/*int main(int argc, char **argv)
{
    
    

    while(ros::ok())
    {   float x=pose.x;
        float y=pose.y;
        float w=pose.theta;
        while(1){
	        if(w>=360){
	        	w=w-360;
	        }
	        if(w<0){
	        	w=w+360;
	        }
	        if(w<=360    &&  w>=0 ){
	        	break;
	        }
    	}
    	w=w*3.14159/180;
        

    }


}*/



void Remote::CalibrateRemote(){
    
    if(remoteState.header.seq==0){
        return;
    }
    sensor_msgs::Joy init;
    for(int i=0;i<1000;i++){
    ros::spinOnce();
    init.axes[0]+=remoteState.axes[0];
    init.axes[1]+=remoteState.axes[1];
    init.axes[2]+=remoteState.axes[2];
    init.axes[3]+=remoteState.axes[3];
    std::cout<<"Calibrating Remote"<<std::endl;

    }
    init.axes[0]=init.axes[0]/1000;
    init.axes[1]=init.axes[1]/1000;
    init.axes[2]=init.axes[2]/1000;
    init.axes[3]=init.axes[3]/1000;
    initialRemoteState=init;
    std::cout<<"calibrationDone "<<initialRemoteState<<std::endl;
    calibrationDone=true;
}


void Remote::RobotOdomCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message)
{   robot_odom_position=*(pose_message);
}


void Remote::RemoteStateCallback(const sensor_msgs::Joy::ConstPtr& msg){
    remoteState=*(msg);
}