//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <std_msgs/builtin_uint8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


bool TeleopOn = false;
ros::Publisher BaseVelCtrlPub;

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
void JoyCB( sensor_msgs::JoyPtr joy )
{
    if (TeleopOn)
    {
        geometry_msgs::Twist twister;
        ROS_INFO("Base Control Mode");
        BaseVelCtrl(joy);
    } else
    {
        ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
        if (joy->axes[2] <= -0.999 && joy->axes[5] <= -0.999)
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

    ros::spin();
    return 0;
}
