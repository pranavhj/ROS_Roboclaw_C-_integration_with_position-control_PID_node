#include "remote.h"


Remote::Remote(ros::NodeHandle *n){
    remoteState.header.seq=0;
    ROS_INFO_STREAM("Remote initialized");
   
    
    ros::Rate rate(10);
    inv_kinematics_publisher=n->advertise<geometry_msgs::Pose2D>("/inv_kinematics",10);
    cmd_vel_publisher=n->advertise<geometry_msgs::Twist>("/cmd_vel",100);
    
    robot_odom_position_subscriber=n->subscribe("/robot_odom_position",10,&Remote::RobotOdomCallback,this);
    remoteStateSubscriber=n->subscribe("/remote_state",10,&Remote::RemoteStateCallback,this);
    
    ros::spinOnce();

}


void Remote::Execute(){
    if(calibrationDone==false){
            //std::cout<<"Calibration not complete"<<std::endl;
            CalibrateRemote();
            ros::spinOnce();
            return;
        }
        if(remoteState.header.seq==0){
            std::cout<<"seq is still zero"<<std::endl;
            return;
        }
        //std::cout<<"69"<<std::endl;
        
        auto leftJoyStickX=remoteState.axes[0]-initialRemoteState.axes[0];
        auto leftJoyStickY=remoteState.axes[1]-initialRemoteState.axes[1];
        auto rightJoystickX=remoteState.axes[2]-initialRemoteState.axes[2];
        auto rightJoystickY=remoteState.axes[3]-initialRemoteState.axes[3];
        //std::cout<<"74"<<std::endl;
        geometry_msgs::Pose2D diff;
        
        if(remoteState.buttons[0]==1){
            up_down_speed_factor+=1;
            if(up_down_speed_factor>20)
                up_down_speed_factor=20;
                
            std::cout<<"updownfactor "<<up_down_speed_factor<<std::endl;
            ros::Duration(0.3).sleep();   
        }
        
        if(remoteState.buttons[1]==1){
            up_down_speed_factor-=1;
            if(up_down_speed_factor<0)
                up_down_speed_factor=2;
            std::cout<<"updownfactor "<<up_down_speed_factor<<std::endl;
            ros::Duration(0.3).sleep();   
        }
        
        if(remoteState.buttons[2]==1){
            rotation_speed_factor+=0.005;
            if(rotation_speed_factor>0.035)
                rotation_speed_factor=0.035;
            std::cout<<"rotation_speed_factor "<<rotation_speed_factor<<std::endl;
            ros::Duration(0.3).sleep();   
        }
        
        if(remoteState.buttons[3]==1){
            
            rotation_speed_factor-=0.005;
            if(up_down_speed_factor<0.005)
                up_down_speed_factor=0.005;
            std::cout<<"rotation_speed_factor "<<rotation_speed_factor<<std::endl;
            ros::Duration(0.3).sleep();   
        }
        
        //std::cout<<up_down_speed_factor<<"  "<<rotation_speed_factor<<std::endl;
        if(abs(leftJoyStickX)>noMovementValue){
            diff.theta=leftJoyStickX*rotation_speed_factor;
        }

        if(abs(rightJoystickX)>noMovementValue){
            diff.x=rightJoystickX*up_down_speed_factor;
        }

        if(abs(rightJoystickY)>noMovementValue){
            diff.y=rightJoystickY*up_down_speed_factor;
        }
        geometry_msgs::Pose2D finalState;
        finalState.x=diff.x;
        finalState.y=diff.y;
        finalState.theta=diff.theta;
        {
            inv_kinematics_publisher.publish(finalState);
            //ros::Duration(0.1).sleep();
        }   
        //ROS_INFO_STREAM(finalState);
        //ROS_INFO_STREAM(remoteState);
        
        remoteVals=finalState;
        
        auto cmd_vel=GetRemoteState();
        cmd_vel_publisher.publish(cmd_vel);
        ros::spinOnce();

}



geometry_msgs::Twist Remote::GetRemoteState(){
    remoteStateCMDVEL.linear.x=remoteVals.x;
    remoteStateCMDVEL.linear.y=remoteVals.y;
    remoteStateCMDVEL.linear.z=0;
    remoteStateCMDVEL.angular.x=0;
    remoteStateCMDVEL.angular.y=0;
    remoteStateCMDVEL.angular.z=remoteVals.theta;
    
    
    return remoteStateCMDVEL;
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
    initialRemoteState.axes={0,0,0,0};
    std::cout<<"calibrationDone "<<remoteState;
    calibrationDone=true;
}


void Remote::RobotOdomCallback(const geometry_msgs::Pose2D::ConstPtr& pose_message)
{   robot_odom_position=*(pose_message);
}


void Remote::RemoteStateCallback(const sensor_msgs::Joy::ConstPtr& msg){
    remoteState=*(msg);
}