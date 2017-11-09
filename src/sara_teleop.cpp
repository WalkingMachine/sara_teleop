//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <std_msgs/builtin_uint8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <wm_tts/say.h>
#include <std_msgs/Float64.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <wm_trajectory_manager/save_trajectory.h>

#define NBJOINTS 7
#define MAXHEADANGLE 0.8
#define MINHEADANGLE -0.8

sensor_msgs::JointState CurArmState;
ros::Publisher SayPub;
ros::Publisher ArmVelCtrlPub;
ros::Publisher BaseVelCtrlPub;
ros::Publisher HeadCtrlPub;
ros::Publisher HandCtrlPub;
bool TeleopOn = false;
int JointIndex = 0;
bool Buttons[30] = {false};
double HeadAngle = 0;
double HandState = 0.1;
bool ArmMode = false;
ros::ServiceClient Switch;
trajectory_msgs::JointTrajectory MyTrajectory;
ros::ServiceClient save;
std::string JointNames[NBJOINTS] = {
        "right shoulder roll joint"
        , "right shoulder pitch joint"
        , "right shoulder yaw joint"
        , "right elbow pitch joint"
        , "right elbow yaw joint"
        , "right wrist pitch joint"
        , "right wrist roll joint"};

void Say( std::string sentence ){
    wm_tts::say msg;
    msg.sentence = sentence;
    SayPub.publish( msg );
}
void SaveTrajectory(){
    wm_trajectory_manager::save_trajectory srv;
    srv.request.trajectory = MyTrajectory;
    srv.request.file = "new_trajectory";
    save.call(srv);
    Say("saving_trajectory");
}
void ResetTrajectory(){
    MyTrajectory.points.clear();
}
void AddPointToTrajectory(){
    trajectory_msgs::JointTrajectoryPoint Point;
    unsigned long Length1 = CurArmState.name.size();
    unsigned long Length2 = MyTrajectory.joint_names.size();
    for ( int i = 0; i < Length1; i++ ) {
        for ( int j=0; j<Length2; j++){
            if ( CurArmState.name[j] == MyTrajectory.joint_names[i] ){
                Point.positions.push_back( CurArmState.position[i] );
            }
        }
    }
    ros::Duration T;
    T.fromNSec( (MyTrajectory.points.size()+1)*100000000 );
    Point.time_from_start = T;
    MyTrajectory.points.push_back(Point);
    Say("adding point");
}
void ArmStateCB(sensor_msgs::JointState State){
    CurArmState = State;
}

void ToggleArmMode(){
    // switch arm to trajectory mode mode
    controller_manager_msgs::SwitchController msg2;
    if (ArmMode){
        msg2.request.start_controllers.push_back("sara_arm_trajectory_controller");
        msg2.request.stop_controllers.push_back("sara_arm_velocity_controller");
        Say("Arm control off");
    } else {
        msg2.request.start_controllers.push_back("sara_arm_velocity_controller");
        msg2.request.stop_controllers.push_back("sara_arm_trajectory_controller");
        Say("Arm control on");
    }
    msg2.request.strictness = 50;
    Switch.waitForExistence();
    Switch.call(msg2);
    ArmMode = !ArmMode;
}
void HandCtrl(sensor_msgs::JoyPtr joy){
    if ( joy->buttons[0] && !Buttons[0] ){
        if ( HandState == 0 )
            HandState = 0.1;
        else
            HandState = 0;
        control_msgs::GripperCommandActionGoal msg;
        msg.goal.command.position = HandState;
        HandCtrlPub.publish( msg );
    }
}
void HeadCtrl(sensor_msgs::JoyPtr joy){

    std_msgs::Float64 msg;
    HeadAngle += joy->axes[4]*-0.01;
    HeadAngle = HeadAngle < MAXHEADANGLE ? HeadAngle : MAXHEADANGLE;
    HeadAngle = HeadAngle > MINHEADANGLE ? HeadAngle : MINHEADANGLE;
    msg.data = HeadAngle;

    HeadCtrlPub.publish( msg );

}
void ArmCtrl(sensor_msgs::JoyPtr joy){
    if (ArmMode) {
        std_msgs::Float64MultiArray VelMsg;
        for (int i = 0; i < NBJOINTS; i++) {
            double vel = 0;
            if (i == JointIndex)
                vel = ((joy->axes[2]) - (joy->axes[5])) * -0.15;
            VelMsg.data.push_back(vel);
        }
        ArmVelCtrlPub.publish(VelMsg);
        if (joy->buttons[7] && !Buttons[7]) {
            JointIndex++;
            if (JointIndex >= NBJOINTS) JointIndex = 0;
            Say(JointNames[JointIndex]);
        }
        if (joy->buttons[6] && !Buttons[6]) {
            JointIndex--;
            if (JointIndex < 0) JointIndex = NBJOINTS - 1;
            Say(JointNames[JointIndex]);
        }
        if (joy->buttons[2] && !Buttons[2]) {
            AddPointToTrajectory();
        }
        if (joy->buttons[3] && !Buttons[3]) {
            SaveTrajectory();
        }
    }
    if ( joy->buttons[1] && !Buttons[1] ){
        ToggleArmMode();
        ResetTrajectory();
    }

}

void BaseVelCtrl(sensor_msgs::JoyPtr joy){
    geometry_msgs::Twist twister;

    float safety = joy->buttons[4] * joy->buttons[5];
    // linear velocity
    twister.linear.x = joy->axes[1]*2*safety;
    twister.linear.y = joy->axes[0]*2*safety;
    // angular velocity
    twister.angular.z = safety * (joy->axes[3]);

    BaseVelCtrlPub.publish( twister );
}
void JoyCB( sensor_msgs::JoyPtr joy )
{
    if (TeleopOn)
    {
        geometry_msgs::Twist twister;
        BaseVelCtrl(joy);
        ArmCtrl( joy);
        HeadCtrl( joy );
        HandCtrl( joy );
        for ( int i=0; i<joy->buttons.size(); i++ ){
            Buttons[i] = (bool)joy->buttons[i];
        }
    } else
    {
        if (joy->axes[2] > 0.9 && joy->axes[5] > 0.9)
            //if (joy->buttons[4] && joy->buttons[5])
        {
            ROS_INFO("Teleop is now on. Please be gentle with me.");
            Say( "Good! I'm now ready to move. Press B to take control of my arm or A to open or close my gripper." );
            TeleopOn = true;
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber ArmStateSub = nh.subscribe("joint_states", 1, &ArmStateCB);
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);

    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>( "sara_arm_velocity_controller/command", 1 );
    BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
    HeadCtrlPub = nh.advertise<std_msgs::Float64>( "/sara_head_pitch_controller/command", 1 );
    SayPub = nh.advertise<wm_tts::say>( "say", 1 );
    HandCtrlPub = nh.advertise<control_msgs::GripperCommandActionGoal>( "/sara_gripper_action_controller/gripper_cmd/goal", 1 );

    // controller services
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
    Switch = nh.serviceClient<controller_manager_msgs::SwitchController>( "controller_manager/switch_controller");
    ROS_INFO("Waiting for controller manager");
    save = nh.serviceClient<wm_trajectory_manager::save_trajectory>("save_trajectory");


    ros::ServiceClient List = nh.serviceClient<controller_manager_msgs::ListControllers>( "controller_manager/list_controllers");
    List.waitForExistence();
    controller_manager_msgs::ListControllers listmsg;
    List.call(listmsg);
    unsigned long Length1 = listmsg.response.controller.size();
    for ( int i=0; i<Length1; i++ ){
        if( listmsg.response.controller[i].name == "sara_arm_trajectory_controller" ){
            MyTrajectory.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
        }
    }
    if ( MyTrajectory.joint_names.size() == 0 ) exit(1);


    // Load controllers
    controller_manager_msgs::LoadController msg;
    Load.waitForExistence( );
    msg.request.name = "sara_arm_velocity_controller";
    Load.call( msg );
    msg.request.name = "sara_base_mecanum_controller";
    Load.call( msg );
    msg.request.name = "sara_head_pitch_controller";
    Load.call( msg );


    // start the loop
    ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
    Say( "I'm now in teleop mode, press both triggers to turn me on." );
    ros::spin();


    return 0;
}
