#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

#include <kdl/frames.hpp>

long double duration;
long double x;
long double y;

int i;

geometry_msgs::Pose2D pose;

geometry_msgs::Pose poseCheat;

geometry_msgs::Pose2D poseMobile;

geometry_msgs::Pose2D poseWorld;

ros::Publisher publisherMobile;

ros::Publisher publisherWorld;

long double r;

double rotX, rotY, rotZ;

KDL::Rotation rotation;

ros::Time timeCurrent;
ros::Time timePrevious;

long double theta_left_current;
long double theta_back_current;
long double theta_right_current;
long double theta_left_previous;
long double theta_back_previous;
long double theta_right_previous;

ros::Time timeCurrent;
ros::Time timePrevious;

long double v_left;
long double v_back;
long double v_right;

void onJointStateMessage(const sensor_msgs::JointState::ConstPtr& input){
    timeCurrent = ros::Time::now();
    for (i = 0; i < input->name.size(); i++) {
        if (input->name[i].c_str()[0] == 'l') {
            theta_left_current = input->position[i];
        } else if (input->name[i].c_str()[0] == 'b') {
            theta_back_current = input->position[i];
        } else if (input->name[i].c_str()[0] == 'r') {
            theta_right_current = input->position[i];
        }
    }
    duration = (timeCurrent - timePrevious).toSec();
    v_left  = (theta_left_current  - theta_left_previous ) / duration;
    v_back  = (theta_back_current  - theta_back_previous ) / duration;
    v_right = (theta_right_current - theta_right_previous) / duration;
    // message.v_left  = v_left ;
    // message.v_back  = v_back ;
    // message.v_right = v_right;
    // publisher.publish(message);
    timePrevious = timeCurrent;
    theta_left_previous  = theta_left_current ;
    theta_back_previous  = theta_back_current ;
    theta_right_previous = theta_right_current;
    onEncoderMessage();

}

void onEncoderMessage(){
    v_left0  = v_left  * r;
    v_back0  = v_back  * r;
    v_right0 = v_right * r;
    timeCurrent = ros::Time::now();
    duration = (timeCurrent - timePrevious).toSec();
    timePrevious = timeCurrent;
    x     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    y     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    theta = (v_left0 + v_back0 + v_right0) / 3;
    poseWorld.x     = (x     * duration) + poseWorld.x    ;
    poseWorld.y     = (y     * duration) + poseWorld.y    ;
    poseWorld.theta = (theta * duration) + poseWorld.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherWorld.publish(poseWorld);
        poseWorld.x     = poseWorld.x    ;
        poseWorld.y     = poseWorld.y    ;
        poseWorld.theta = poseWorld.theta;
    }
    poseWorld.x     = (x     * duration) + poseMobile.x    ;
    poseWorld.y     = (y     * duration) + poseMobile.y    ;
    poseWorld.theta = (theta * duration) + poseMobile.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherMobile.publish(poseWorld);
        poseMobile.x     = poseWorld.x    ;
        poseMobile.y     = poseWorld.y    ;
        poseMobile.theta = poseWorld.theta;
    }
}

void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
    for (i = 0; i < input->name.size(); i++) {
        if (((input->name[i]).find(openBaseString) == std::string::npos) || ((input->name[i]).find(originString) == std::string::npos)) {
            continue;
        }
        poseCheat = input->pose[i];
        rotation = KDL::Rotation::Quaternion(poseCheat.orientation.x, poseCheat.orientation.y, poseCheat.orientation.z, poseCheat.orientation.w);
        rotation.GetRPY(rotX, rotY, rotZ);
        poseWorld.x     = poseCheat.position.x;
        poseWorld.y     = poseCheat.position.y;
        poseWorld.theta = rotZ;
        publisherWorld.publish(poseWorld);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("/joint_states", 1, updateJoint);
    ros::Subscriber subscriber1;
    
    publisherWorld  = node.advertise<geometry_msgs::Pose2D>("pose/world" , 1);
    publisherMobile = node.advertise<geometry_msgs::Pose2D>("pose/mobile", 1);
    ros::spin();
    return 0;
}
