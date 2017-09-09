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

#define NBJOINTS 7

ros::Publisher SayPub;
ros::Publisher ArmVelCtrlPub;
bool TeleopOn = false;
ros::Publisher BaseVelCtrlPub;
int JointIndex = 0;
bool Buttons[30] = {false};
std::string JointNames[NBJOINTS] = {
        "sara_right_shoulder_roll_joint"
        , "sara_right_shoulder_pitch_joint"
        , "sara_right_shoulder_yaw_joint"
        , "sara_right_elbow_pitch_joint"
        , "sara_right_elbow_yaw_joint"
        , "sara_right_wrist_pitch_joint"
        , "sara_right_wrist_roll_joint"};


void ArmCtrl(sensor_msgs::JoyPtr joy){

    std_msgs::Float64MultiArray VelMsg;
    for ( int i=0; i<NBJOINTS; i++ ){
        double vel = 0;
        if ( i == JointIndex )
            vel = ((joy->axes[2]) - (joy->axes[5])) * -15;
        VelMsg.data.push_back( vel );
    }
    ArmVelCtrlPub.publish( VelMsg );
    if ( joy->buttons[7] && !Buttons[7] ){
        JointIndex++;
        if ( JointIndex >= NBJOINTS ) JointIndex = 0;
        wm_tts::say msg;
        msg.sentence = JointNames[JointIndex];
        SayPub.publish( msg );
    }
    if ( joy->buttons[6] && !Buttons[6] ){
        JointIndex--;
        if ( JointIndex < 0 ) JointIndex = NBJOINTS;
        wm_tts::say msg;
        msg.sentence = JointNames[JointIndex];
        SayPub.publish( msg );
    }
    for ( int i=0; i<joy->buttons.size(); i++ ){
        Buttons[i] = (bool)joy->buttons[i];
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
        ROS_INFO("Base Control Mode");
        BaseVelCtrl(joy);
        ArmCtrl( joy);
    } else
    {
        ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
        if (joy->buttons[4] && joy->buttons[5])
        {
            TeleopOn = true;
        }
    }
}

int main(int argc, char **argv) {
    sleep(5);

    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);

    // Old base control publisher
    BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );

    // Load Sara arm controller
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>( "sara_arm_velocity_controller/command", 1 );
    SayPub = nh.advertise<wm_tts::say>( "say", 1 );
    controller_manager_msgs::SwitchController msg;
    msg.request.strictness = 50;
    msg.request.start_controllers.push_back("sara_arm_velocity_controller");
    msg.request.stop_controllers.push_back("sara_arm_trajectory_controller");

    ros::spin();
    return 0;
}
