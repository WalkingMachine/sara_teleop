//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <std_msgs/builtin_uint8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <wm_tts/say.h>
#include <std_msgs/Float64.h>
#include <control_msgs/GripperCommand.h>

#define NBJOINTS 7
#define MAXHEADANGLE 0.8
#define MINHEADANGLE -0.8

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
void HandCtrl(sensor_msgs::JoyPtr joy){
    if ( joy->buttons[6] && !Buttons[6] ){
        if ( HandState == 0 )
            HandState = 0.1;
        else
            HandState = 0;
        control_msgs::GripperCommand msg;
        msg.position = HandState;
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

    std_msgs::Float64MultiArray VelMsg;
    for ( int i=0; i<NBJOINTS; i++ ){
        double vel = 0;
        if ( i == JointIndex )
            vel = ((joy->axes[2]) - (joy->axes[5])) * -0.15;
        VelMsg.data.push_back( vel );
    }
    ArmVelCtrlPub.publish( VelMsg );
    if ( joy->buttons[7] && !Buttons[7] ){
        JointIndex++;
        if ( JointIndex >= NBJOINTS ) JointIndex = 0;
        Say( JointNames[JointIndex] );
    }
    if ( joy->buttons[6] && !Buttons[6] ){
        JointIndex--;
        if ( JointIndex < 0 ) JointIndex = NBJOINTS;
        Say( JointNames[JointIndex] );
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
            Say( "Good! I'm now ready to move." );
            TeleopOn = true;
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);

    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>( "sara_arm_velocity_controller/command", 1 );
    BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
    HeadCtrlPub = nh.advertise<std_msgs::Float64>( "/sara_head_pitch_controller/command", 1 );
    SayPub = nh.advertise<wm_tts::say>( "say", 1 );
    HandCtrlPub = nh.advertise<control_msgs::GripperCommand>( "/sara_gripper_action_controller/goal", 1 );

    // controller services
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
    ros::ServiceClient Switch = nh.serviceClient<controller_manager_msgs::SwitchController>( "controller_manager/switch_controller");
    ROS_INFO("Waiting for controller manager");

    // Load controllers
    controller_manager_msgs::LoadController msg;
    Load.waitForExistence( );
    msg.request.name = "sara_arm_velocity_controller";
    Load.call( msg );
    msg.request.name = "sara_base_mecanum_controller";
    Load.call( msg );
    msg.request.name = "sara_head_pitch_controller";
    Load.call( msg );

    // switch arm to velocity mode
    controller_manager_msgs::SwitchController msg2;
    msg2.request.strictness = 50;
    msg2.request.start_controllers.push_back("sara_arm_velocity_controller");
    msg2.request.stop_controllers.push_back("sara_arm_trajectory_controller");
    Switch.waitForExistence();
    Switch.call(msg2);

    // start the loop
    ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
    Say( "I'm now in teleop mode, press both triggers to turn me on." );
    ros::spin();

    // switch arm to trajectory mode mode
    msg2.request.strictness = 50;
    msg2.request.start_controllers.push_back("sara_arm_trajectory_controller");
    msg2.request.stop_controllers.push_back("sara_arm_velocity_controller");
    Switch.waitForExistence();
    Switch.call(msg2);

    return 0;
}
