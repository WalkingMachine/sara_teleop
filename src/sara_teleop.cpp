//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <std_msgs/builtin_uint8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>
#include <dynamixel_msgs/JointState.h>
#include <robotiq_c_model_control/c_model_ethercat_client.h>


bool TeleopOn = false;
int TeleopMode = 2;
bool ArmTrajMode = false;
bool OldArmTrajMode = true;
sensor_msgs::JointState CurArmState;
robotiq_c_model_control::CModel_robot_output hand_cmd;
bool pinceState = true;
ros::Publisher ArmVelCtrlPub;
ros::Publisher ArmTrjCtrlPub;
ros::Publisher BaseVelCtrlPub;
ros::Publisher HeadPosCtrlPub;
ros::Publisher GripperCtrlPub;
trajectory_msgs::JointTrajectory Animation;
trajectory_msgs::JointTrajectory EmptyAnimation;
ros::Duration Animation_duration;
ros::Time Animation_time;
dynamixel_msgs::JointState CurHeadState;
bool RB6 = false;
bool RB7 = false;
bool RB1 = false;
bool RB2 = false;
bool RB3 = false;
bool RB4 = false;
float OldHeadCmd = 0;


void ModePlus(){ TeleopMode ++; if ( TeleopMode > 3 ) TeleopMode = 1; }
void ModeMoin(){ TeleopMode --; if ( TeleopMode < 1 ) TeleopMode = 3; }
void AddPoint(){
    trajectory_msgs::JointTrajectoryPoint Point;
    int Length1 = CurArmState.name.size();
    int Length2 = Animation.joint_names.size();
    for ( int i = 0; i < Length1; i++ ) {
        Point.positions.push_back( CurArmState.position[i] );
    }
    ros::Duration T;
    T.fromNSec( (Animation.points.size()+1)*1000000000 );
    Point.time_from_start = T;
    Animation.points.push_back( Point );
}
void ClearPoints(){ Animation.points.clear(); }
void LaunchAnimation(){
    ArmTrajMode = true;
    ros::Duration T;
    T.fromNSec( 5000000000 );
    Animation_duration = Animation.points[Animation.points.size()-1].time_from_start+T;
    Animation_time = ros::Time::now();
}
void ToggleGripper(){
    if ( pinceState ) {
        hand_cmd.rPR = 0;
        pinceState = false;
    } else {
        hand_cmd.rPR = 250;
        pinceState = true;
    };
    GripperCtrlPub.publish(hand_cmd);
}
void ArmCtrl(sensor_msgs::JoyPtr joy){

    std_msgs::Float64MultiArray VelMsg;
    VelMsg.data.push_back((joy->axes[0] * -15) );
    VelMsg.data.push_back((joy->axes[1] * -15) );
    VelMsg.data.push_back((joy->axes[3] * 15) );
    VelMsg.data.push_back((joy->axes[4] * -15) );
    VelMsg.data.push_back(((joy->axes[2]) - (joy->axes[5])) * -30);
    VelMsg.data.push_back(0);
    VelMsg.data.push_back(0);
    ArmVelCtrlPub.publish( VelMsg );

    if ( joy->buttons[0] == 1 ){ if ( !RB1 ){
            AddPoint();
            RB1 = true;
        } } else { RB1 = false; }
    if ( joy->buttons[1] == 1 ){ if ( !RB2 ){
            ClearPoints();
            RB2 = true;
        } } else { RB2 = false; }
    if ( joy->buttons[2] == 1 ){ if ( !RB3 ){
            LaunchAnimation();
            RB3 = true;
        } } else { RB3 = false; }
    if ( joy->buttons[3] == 1 ){ if ( !RB4 ){
            ToggleGripper();
            RB4 = true;
        } } else { RB4 = false; }
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


void BaseVelCtrl(sensor_msgs::JoyPtr joy){
    geometry_msgs::Twist twister;
    //ArmVelCtrlPub.publish( VelMsg );  // TODO
    float safety = joy->axes[2] * joy->axes[5];
    // linear velocity
    twister.linear.x = joy->axes[1]*2*safety;
    twister.linear.y = joy->axes[0]*2*safety;
    // angular velocity
    twister.angular.z = safety * (joy->axes[3]);

    BaseVelCtrlPub.publish( twister );
}
void HeaPoseCtrl(sensor_msgs::JoyPtr joy){
    if ( joy->axes[6] != OldHeadCmd ){
        CurHeadState.goal_pos += joy->axes[6]*0.5;
        OldHeadCmd = joy->axes[6];
    }
    HeadPosCtrlPub.publish( CurHeadState );
}
void JoyCB( sensor_msgs::JoyPtr joy ){
    if (TeleopOn ) {
        switch ( TeleopMode ){
            case 1:
                ROS_INFO("Arm and head Control Mode");
                ROS_INFO("%d Points saved", (int)Animation.points.size());
                ArmCtrl(joy);
                HeaPoseCtrl(joy);
                break;
            case 2:
                ROS_INFO("Base and head Control Mode");
                BaseVelCtrl(joy);
                HeaPoseCtrl(joy);
                break;
            case 3:
                ROS_INFO("Emotion Mode");
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

void ArmStateCB(sensor_msgs::JointState State){
    CurArmState = State;
}
void HeadStateCB(dynamixel_msgs::JointState State){
    CurHeadState = State;
}
void EmotionCB( std_msgs::UInt8 msg ){

    if ( TeleopMode == 3 ){

        ClearPoints();
        ArmTrajMode = false;
        trajectory_msgs::JointTrajectoryPoint Point;
        ros::Duration T;
        T.fromNSec( 1000000000 );
        Point.time_from_start = T;

        ClearPoints();
        switch ( msg.data ){
            case 1:
                Point.positions.push_back(-1.271065294203758);
                Point.positions.push_back(-0.0735725925779338);
                Point.positions.push_back(-0.15369062621414642);
                Point.positions.push_back(0.09231440647125311);
                Point.positions.push_back(0.24865288876563296);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 2:
                Point.positions.push_back(-0.26353138504505136);
                Point.positions.push_back(-0.0735725925779338);
                Point.positions.push_back(-0.024048386748432904);
                Point.positions.push_back(-0.21522563680291107);
                Point.positions.push_back(-0.3009808390942216);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 3:
                Point.positions.push_back(-0.6734447105669972);
                Point.positions.push_back(-0.0735725925779338);
                Point.positions.push_back(-0.1412601866841312);
                Point.positions.push_back(0.33580406482577363);
                Point.positions.push_back(1.1159385283759236);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 4:
                Point.positions.push_back(-1.1911567970728871);
                Point.positions.push_back(-0.7961188781499859);
                Point.positions.push_back(0.002787640710473216);
                Point.positions.push_back(0.5152137522304061);
                Point.positions.push_back(1.2064008119031786);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 5:
                Point.positions.push_back(-2.0002837913560865);
                Point.positions.push_back(-1.0735725925779338);
                Point.positions.push_back(-0.22417147863149633);
                Point.positions.push_back(0.37477972735285814);
                Point.positions.push_back(-0.7965404790809751);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 6:
                Point.positions.push_back(-1.5349146354472634);
                Point.positions.push_back(-1.0735725925779338);
                Point.positions.push_back(-0.576672176196575);
                Point.positions.push_back(-0.2563539639103407);
                Point.positions.push_back(1.6242166035541892);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            case 7:
                Point.positions.push_back(-1.2568786054480072);
                Point.positions.push_back(-4.796351678194171);
                Point.positions.push_back(-2.4977602503213285);
                Point.positions.push_back(-0.5666054357790946);
                Point.positions.push_back(0.9409450241187216);
                Point.positions.push_back(0.0);
                Point.positions.push_back(0.0);
                break;
            default:
                break;
        }
        Animation.points.push_back( Point );
        LaunchAnimation();
    }
}
int main(int argc, char **argv) {
    sleep(5);

    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;


    // Subscribers
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);
    ros::Subscriber ArmStateSub = nh.subscribe("joint_states", 1, &ArmStateCB);
    ros::Subscriber ArmTrajSub = nh.subscribe("sara_arm_trajectory_controller/state", 1, &ArmStateCB);
    ros::Subscriber HeadStateSub = nh.subscribe("neckHead_controller/state", 1, &HeadStateCB);
    ros::Subscriber EmotionSub = nh.subscribe("sara_face/Emotion", 1, &EmotionCB);

    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>( "sara_arm_velocity_controller/command", 1 );
    ArmTrjCtrlPub = nh.advertise<trajectory_msgs::JointTrajectory>( "sara_arm_trajectory_controller/command", 1 );
    GripperCtrlPub = nh.advertise<robotiq_c_model_control::CModel_robot_output>( "CModelRobotOutput", 1 );
    // New base control publisher
    //BaseVelCtrlPub = nh.advertise<trajectory_msgs::JointTrajectory>( "", 1 );  // TODO
    // Old base control publisher
    BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>( "safe_cmd_vel", 1 );
    HeadPosCtrlPub = nh.advertise<dynamixel_msgs::JointState>( "/neckHead/state", 1 );  // TODO renew to roscontrol
    // Old Head control publisher
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
    ros::ServiceClient Switch = nh.serviceClient<controller_manager_msgs::SwitchController>( "controller_manager/switch_controller");


    hand_cmd.rACT = 1;
    hand_cmd.rGTO = 1;
    hand_cmd.rSP = 200;
    hand_cmd.rFR = 0;
    hand_cmd.rPR = 0;
    pinceState = 1;


    ros::ServiceClient List = nh.serviceClient<controller_manager_msgs::ListControllers>( "controller_manager/list_controllers");
    List.waitForExistence();
    controller_manager_msgs::ListControllers listmsg;
    List.call(listmsg);
    int Length1 = listmsg.response.controller.size();
    for ( int i=0; i<Length1; i++ ){
        if( listmsg.response.controller[i].name == "sara_arm_trajectory_controller" ){
            Animation.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
            EmptyAnimation.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
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
        if ( ArmTrajMode && ros::Time::now() > Animation_time+Animation_duration ) {
            ArmTrajMode = false;
        }
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
                ArmTrjCtrlPub.publish( EmptyAnimation );
                ArmTrjCtrlPub.publish( Animation );
            }
        }
        ros::spinOnce();
    }
    return 0;
}