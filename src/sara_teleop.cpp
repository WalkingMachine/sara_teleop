//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager/controller_manager.h>

bool TeleopOn = false;
int TeleopMode = 2;
bool ArmTrajMode = false;
bool OldArmTrajMode = true;
sensor_msgs::JointState CurState;
ros::Publisher ArmVelCtrlPub;
ros::Publisher ArmTrjCtrlPub;
ros::Publisher BaseVelCtrlPub;
ros::Publisher HeadVelCtrlPub;
ros::ServiceClient Switch;
std::vector<std::string> TrajRessources;
trajectory_msgs::JointTrajectory Animation;
bool RB6 = false;
bool RB7 = false;
bool RB1 = false;
bool RB2 = false;
bool RB3 = false;




void ModePlus(){
    TeleopMode ++;
    if ( TeleopMode > 2 ) TeleopMode = 1;
}
void ModeMoin(){
    TeleopMode --;
    if ( TeleopMode < 1 ) TeleopMode = 2;
}
void AddPoint(){
    trajectory_msgs::JointTrajectoryPoint Point;
    int Length1 = CurState.name.size();
    int Length2 = Animation.joint_names.size();
    for ( int i = 0; i < Length1; i++ ) {
        //for ( int j=0; j < Length2; j++ ){
            //if ( CurState.name[i] == Animation.joint_names[j]  ){
                Point.positions.push_back( CurState.position[i] );
            //}
        //}
    }
    ros::Duration T;
    T.fromSec( Animation.points.size() );
    Point.time_from_start = T;
    Animation.points.push_back( Point );
}
void ClearPoints(){
    Animation.points.clear();
}
void LaunchAnimation(){
    ArmTrajMode = true;
}
void ArmMode( sensor_msgs::JoyPtr joy ){

    std_msgs::Float64MultiArray VelMsg;
    VelMsg.data.push_back((joy->axes[0] * -15) );
    VelMsg.data.push_back((joy->axes[1] * -15) );
    VelMsg.data.push_back((joy->axes[3] * -15) );
    VelMsg.data.push_back((joy->axes[4] * -15) );
    VelMsg.data.push_back(((joy->axes[2]) - (joy->axes[5])) * -30);
    VelMsg.data.push_back(0);
    VelMsg.data.push_back(0);
    ArmVelCtrlPub.publish( VelMsg );

    if ( joy->buttons[0] ){ if ( !RB1 ){
            AddPoint();
            RB1 = true;
        } } else { RB1 = false; }
    if ( joy->buttons[1] ){ if ( !RB2 ){
            ClearPoints();
            RB2 = true;
        } } else { RB2 = false; }
    if ( joy->buttons[2] ){ if ( !RB3 ){
            LaunchAnimation();
            RB3 = true;
        } } else { RB3 = false; }
    if ( ArmTrajMode ) {
        int sum = 0;
        for (int i = 0; i < 7; i++) {
            sum += abs(VelMsg.data[i] * 1000.0);
        }
        if (sum != 0 || joy->buttons[1]) {
            ArmTrajMode = false;
            ROS_INFO("Animation Interrupted");
        } else {
            ROS_INFO("Running Animation");
        }
    }
}


void BaseVelMode( sensor_msgs::JoyPtr joy ){
    //ArmVelCtrlPub.publish( VelMsg );  // TODO
}
void HeadVelMode( sensor_msgs::JoyPtr joy ){
    //ArmVelCtrlPub.publish( VelMsg );  // TODO
}
void JoyCB( sensor_msgs::JoyPtr joy ){
    if (TeleopOn ) {
        switch ( TeleopMode ){
            case 1:
                ROS_INFO("Arm and head Control Mode");
                ROS_INFO("%d Points saved", Animation.points.size());
                ArmMode( joy );
                HeadVelMode( joy );
                break;
            case 2:
                ROS_INFO("Base and head Control Mode");
                BaseVelMode( joy );
                HeadVelMode( joy );
                break;
            default:
                ROS_INFO("invalid mode");
                TeleopOn = false;
                break;
        }
    } else {
        ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
        if ( joy->axes[2]  <= -0.999 && joy->axes[5] <= -0.999 ) {
            TeleopOn = true;
        }
    }
    if ( joy->buttons[6] == 1 ) {
        if (!RB6) { ModeMoin(); RB6 = true; }
    }else{ RB6 = false; }
    if ( joy->buttons[7] == 1 ) {
        if (!RB7) { ModePlus(); RB7 = true; }
    }else{ RB7 = false; }

}

void StateCB( sensor_msgs::JointState State ){
    CurState = State;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);
    ros::Subscriber StateSub = nh.subscribe("joint_states", 1, &StateCB);

    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>( "sara_arm_velocity_controller/command", 1 );
    ArmTrjCtrlPub = nh.advertise<trajectory_msgs::JointTrajectory>( "sara_arm_trajectory_controller/command", 1 );
    //BaseVelCtrlPub = nh.advertise<trajectory_msgs::JointTrajectory>( "", 1 );  // TODO
    //HeadVelCtrlPub = nh.advertise<trajectory_msgs::JointTrajectory>( "", 1 );  // TODO
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
    ros::ServiceClient Switch = nh.serviceClient<controller_manager_msgs::SwitchController>( "controller_manager/switch_controller");

    ros::ServiceClient List = nh.serviceClient<controller_manager_msgs::ListControllers>( "controller_manager/list_controllers");
    List.waitForExistence();
    controller_manager_msgs::ListControllers listmsg;
    List.call(listmsg);
    int Length1 = listmsg.response.controller.size();
    for ( int i=0; i<Length1; i++ ){
        if( listmsg.response.controller[i].name == "sara_arm_trajectory_controller" ){
            Animation.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
        }
    }
    if ( Animation.joint_names.size() == 0 ) exit(1);

    // Load controllers
    controller_manager_msgs::LoadController msg;
    Load.waitForExistence( );
    msg.request.name = "sara_arm_velocity_controller";
    Load.call( msg );
    msg.request.name = "sara_arm_trajectory_controller";
    Load.call( msg );
    msg.request.name = "sara_base_mecanum_controller";
    Load.call( msg );
    msg.request.name = "sara_head_pitch_controller";
    Load.call( msg );
    msg.request.name = "sara_head_yaw_controller";
    Load.call( msg );

    ROS_INFO("start teleop");
    while ( ros::ok() ){
        if ( OldArmTrajMode != ArmTrajMode ) {
            controller_manager_msgs::SwitchController msg;
            msg.request.strictness = 50;
            Switch.waitForExistence();
            ROS_INFO("Calling switch");
            if (!ArmTrajMode) { // switch from traj to vel
                msg.request.start_controllers.push_back("sara_arm_velocity_controller");
                msg.request.stop_controllers.push_back("sara_arm_trajectory_controller");
            } else { // switch from vel to traj
                msg.request.start_controllers.push_back("sara_arm_trajectory_controller");
                msg.request.stop_controllers.push_back("sara_arm_velocity_controller");
            }
            OldArmTrajMode = ArmTrajMode;
            Switch.call(msg);
            if ( ArmTrajMode ){
                Switch.waitForExistence();
                ArmTrjCtrlPub.publish( Animation );
            }
        }
        ros::spinOnce();
    }
    return 0;
}