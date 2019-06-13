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
#include <std_msgs/String.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <wm_trajectory_manager/save_trajectory.h>
#include <stdio.h>

#define NBJOINTS 8
#define MAXHEADANGLE 0.8
#define MINHEADANGLE -0.8

sensor_msgs::JointState CurArmState;
ros::Publisher SayPub;
ros::Publisher ArmVelCtrlPub;
ros::Publisher BaseVelCtrlPub;
ros::Publisher HeadCtrlPub;
ros::Publisher HeadCtrlPubYaw;
ros::Publisher HandCtrlPub;
bool TeleopOn = false;
int JointIndex = 0;
bool buttonsAlreadyPressed[30] = {false};
double HeadAngle = 0;
double HandState = 0.1;
bool ArmMode = false;
double rotvel = 0;
double xvel = 0;
double yvel = 0;
bool toogleOnOffMem{true};

std::string Sentence = "";

ros::ServiceClient Switch;
trajectory_msgs::JointTrajectory MyTrajectory;
ros::ServiceClient save;
std::string JointNames[NBJOINTS] = {
        "right shoulder roll joint", "right shoulder pitch joint", "right shoulder yaw joint",
        "right elbow pitch joint", "right elbow yaw joint", "right wrist pitch joint", "right wrist roll joint", "Base actuator joint"};


void Say(std::string sentence) {
    wm_tts::say msg;
    msg.sentence = sentence;
    SayPub.publish(msg);
}

void SaveTrajectory() {
    wm_trajectory_manager::save_trajectory srv;
    srv.request.trajectory = MyTrajectory;
    srv.request.file = "new_trajectory";
    save.waitForExistence();

    save.call(srv);
    Say("saving trajectory");
}

void ResetTrajectory() {
    MyTrajectory.points.clear();
}

void AddPointToTrajectory() {
    trajectory_msgs::JointTrajectoryPoint Point;
    unsigned long Length1 = CurArmState.name.size();
    unsigned long Length2 = MyTrajectory.joint_names.size();
    for (int i = 0; i < Length1; i++) {
        for (int j = 0; j < Length2; j++) {
            if (CurArmState.name[i] == MyTrajectory.joint_names[j]) {
                Point.positions.push_back(CurArmState.position[i]);
            }
        }
    }
    ros::Duration T;
    T.fromSec((MyTrajectory.points.size() + 0.5));
    Point.time_from_start = T;
    MyTrajectory.points.push_back(Point);
    Say("adding point");
}

void ArmStateCB(sensor_msgs::JointState State) {
    CurArmState = State;
}

void ToggleArmMode() {
    // switch arm to trajectory mode mode
    controller_manager_msgs::SwitchController msg2;
    if (ArmMode) {
        msg2.request.start_controllers.push_back("sara_arm_trajectory_controller");
        msg2.request.stop_controllers.push_back("sara_arm_velocity_controller");
        //        Say("Position mode");
    } else {
        msg2.request.start_controllers.push_back("sara_arm_velocity_controller");
        msg2.request.stop_controllers.push_back("sara_arm_trajectory_controller");
        //        Say("Velocity mode");
    }
    msg2.request.strictness = 50;
    Switch.waitForExistence();
    Switch.call(msg2);
    ArmMode = !ArmMode;
}

void HandCtrl(sensor_msgs::JoyPtr joy) {
    if (joy->buttons[0] && !buttonsAlreadyPressed[0]) {
        if (HandState == 0)
            HandState = 0.1;
        else
            HandState = 0;
        control_msgs::GripperCommandActionGoal msg;
        msg.goal.command.position = HandState;
        HandCtrlPub.publish(msg);
    }
}

void HeadCtrl(sensor_msgs::JoyPtr joy) {

    // Pitch control
    std_msgs::Float64 msg;
    HeadAngle += joy->axes[4] * -0.01;
    HeadAngle = HeadAngle < MAXHEADANGLE ? HeadAngle : MAXHEADANGLE;
    HeadAngle = HeadAngle > MINHEADANGLE ? HeadAngle : MINHEADANGLE;
    msg.data = HeadAngle;
    HeadCtrlPub.publish(msg);

    // Yaw control
    msg.data = joy->axes[3] + joy->axes[0];
    HeadCtrlPubYaw.publish(msg);

}

void ArmCtrl(sensor_msgs::JoyPtr joy) {
    if (ArmMode) {
        std_msgs::Float64MultiArray VelMsg;
        for (int i = 0; i < NBJOINTS; i++) {
            double vel = 0;
            if (i == JointIndex)
                vel = ((joy->axes[2]) - (joy->axes[5])) * -0.5;
            VelMsg.data.push_back(vel);
        }
        ArmVelCtrlPub.publish(VelMsg);
        if (joy->buttons[7] && !buttonsAlreadyPressed[7]) {
            JointIndex++;
            if (JointIndex >= NBJOINTS) JointIndex = 0;
            //Say(JointNames[JointIndex]);
        }
        if (joy->buttons[6] && !buttonsAlreadyPressed[6]) {
            JointIndex--;
            if (JointIndex < 0) JointIndex = NBJOINTS - 1;
            //Say(JointNames[JointIndex]);
        }
        if (joy->buttons[2] && !buttonsAlreadyPressed[2]) {
            AddPointToTrajectory();
        }
        if (joy->buttons[3] && !buttonsAlreadyPressed[3]) {
            SaveTrajectory();
        }
    }
    if (joy->buttons[1] && !buttonsAlreadyPressed[1]) {
        ToggleArmMode();
        ResetTrajectory();
    }

}

void BaseVelCtrl(sensor_msgs::JoyPtr joy) {
    geometry_msgs::Twist twister;

    float safety = joy->buttons[4] * joy->buttons[5];
    // linear velocity
    xvel += (joy->axes[1] * 2 - xvel) * 0.2;
    yvel += (joy->axes[0] * 2 - yvel) * 0.2;

    twister.linear.x = xvel * safety;
    twister.linear.y = yvel * safety;
    // angular velocity
    rotvel += (joy->axes[3] - rotvel) * 0.15;
    twister.angular.z = safety * rotvel;

    BaseVelCtrlPub.publish(twister);
}

void JoyCB(sensor_msgs::JoyPtr joy) {
    if (TeleopOn) {
        geometry_msgs::Twist twister;
        BaseVelCtrl(joy);
        ArmCtrl(joy);
        HeadCtrl(joy);
        HandCtrl(joy);
        for (int i = 0; i < joy->buttons.size(); i++) {
            buttonsAlreadyPressed[i] = (bool) joy->buttons[i];
        }
        if (joy->axes[7] > 0.0) {
            Say(Sentence);
            sleep(2);
        } else if (joy->axes[7] < 0.0) {
            Say("Bonjours, mon nom est Sara.");
            sleep(2);
        }
        if (joy->axes[6] > 0.0) {
            Say("Je suis heureuse de vous rencontrer.");
            sleep(2);
        } else if (joy->axes[6] < 0.0) {
            Say("Je suis un robot d'assistance personnelle construit et programmé par le club Oualking Machine.");
            sleep(2);
        }
    }
    if (joy->axes[2] < -0.9 && joy->axes[5] < -0.9) {
        if (toogleOnOffMem) {
            ROS_INFO("TOOGLE!");
            if (TeleopOn) {
                ROS_INFO("Teleop is now off. I should be able to take care of myself for the time being");
                TeleopOn = false;
            } else {
                ROS_INFO("Teleop is now on. Please be gentle with me.");
                TeleopOn = true;
            }
            toogleOnOffMem = false;
        }
    } else {
        toogleOnOffMem = true;
    }
}

void SetSayCB(std_msgs::String sentence) {
    Sentence = sentence.data;
}

int main(int argc, char **argv) {

    TeleopOn = false;

    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Waiting for services
    ros::service::waitForService("controller_manager/load_controller");
    ros::service::waitForService("controller_manager/list_controllers");
    ros::service::waitForService("controller_manager/switch_controller");
    ros::service::waitForService("save_trajectory");


    ROS_INFO("starting publishers");
    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>("sara_arm_velocity_controller/command", 1);
    BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    HeadCtrlPub = nh.advertise<std_msgs::Float64>("/sara_head_pitch_controller/command", 1);
    HeadCtrlPubYaw = nh.advertise<std_msgs::Float64>("/sara_head_yaw_controller/command", 1);
    SayPub = nh.advertise<wm_tts::say>("say", 1);
    HandCtrlPub = nh.advertise<control_msgs::GripperCommandActionGoal>(
            "/sara_gripper_action_controller/gripper_cmd/goal", 1);


    ROS_INFO("starting subscribers");
    // Subscribers
    ros::Subscriber ArmStateSub = nh.subscribe("joint_states", 1, &ArmStateCB);
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);
    ros::Subscriber Sentence = nh.subscribe("sentenceToSay", 1, &SetSayCB);

    ROS_INFO("starting service clients");
    // controller services
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>(
            "controller_manager/load_controller");
    Switch = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    save = nh.serviceClient<wm_trajectory_manager::save_trajectory>("save_trajectory");
    ros::ServiceClient List = nh.serviceClient<controller_manager_msgs::ListControllers>(
            "controller_manager/list_controllers");


    ROS_INFO("getting controller");
    List.waitForExistence();
    controller_manager_msgs::ListControllers listmsg;


    do {
        List.call(listmsg);
        unsigned long Length1 = listmsg.response.controller.size();
        for (int i = 0; i < Length1; i++) {
            if (listmsg.response.controller[i].name == "sara_arm_trajectory_controller") {
                MyTrajectory.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
            }
        }
        if (MyTrajectory.joint_names.size() == 0) {
            sleep(1);
            continue;
        } else break;

    } while (true);

    ROS_INFO("loading controllers");
    // Load controllers
    controller_manager_msgs::LoadController msg;
    Load.waitForExistence();
    msg.request.name = "sara_arm_velocity_controller";
    Load.call(msg);
    //    msg.request.name = "sara_base_mecanum_controller";
    //    Load.call( msg );
    //    msg.request.name = "sara_head_pitch_controller";
    //    Load.call( msg );


    // start the loop
    ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
    ros::spin();


    return 0;
}
