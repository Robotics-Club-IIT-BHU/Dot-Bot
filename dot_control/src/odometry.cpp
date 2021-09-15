#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>

#include <kdl/frames.hpp>


long double duration;
long double x;
long double y;
long double theta;

int i;

geometry_msgs::Pose pose;

geometry_msgs::Pose poseCheat;

geometry_msgs::PoseStamped poseMobile;

geometry_msgs::PoseStamped poseWorld;
geometry_msgs::PoseStamped poseWorld1;

double poseWorld_theta = 0;
double poseMobile_theta = 0;
ros::Publisher publisherMobile;

ros::Publisher publisherWorld;

long double r=0.019;

double rotX, rotY, rotZ;


ros::Time timeCurrent;
ros::Time timePrevious;

long double theta_left_current;
long double theta_back_current;
long double theta_right_current;
long double theta_left_previous;
long double theta_back_previous;
long double theta_right_previous;

long double v_left;
long double v_back;
long double v_right;
long double v_left0;
long double v_back0;
long double v_right0;

void onEncoderMessage(){
    v_left0  = v_left  * r;
    v_back0  = v_back  * r;
    v_right0 = v_right * r;
    //timeCurrent = ros::Time::now();
    //duration = (timeCurrent - timePrevious).toSec();
    //timePrevious = timeCurrent;
    x     = ((2.0 * v_back0) - v_left0 - v_right0) / 3.0;
    y     = ((1.73 * v_right0) - (1.73 * v_left0)) / 3.0;
    theta = (v_left0 + v_back0 + v_right0) / (3*0.04);
    // poseWorld.pose.position.x     = (x     * duration) + poseWorld.pose.position.x    ;
    // poseWorld.pose.position.y     = (y     * duration) + poseWorld.pose.position.y    ;
    // poseWorld_theta = (theta * duration) + poseWorld_theta;
    // if ((!std::isnan(poseWorld.pose.position.x)) && (!std::isnan(poseWorld.pose.position.y)) && (!std::isnan(poseWorld_theta))) {
    //     poseWorld.header.stamp = ros::Time::now();
    //     publisherMobile.publish(poseWorld);
    //     // poseWorld.x     = poseWorld.x    ; 
    //     // poseWorld.y     = poseWorld.y    ;
    //     // poseWorld.theta = poseWorld.theta;
    //     tf::Quaternion q;
    //     q.setRPY(0,0,poseWorld_theta);

    //     poseWorld.pose.orientation.x = q.x();
    //     poseWorld.pose.orientation.y = q.y();
    //     poseWorld.pose.orientation.z = q.z();
    //     poseWorld.pose.orientation.w = q.w();
        
    // }
    poseWorld.pose.position.x     = (x     * duration) + poseMobile.pose.position.x    ;
    poseWorld.pose.position.y     = (y     * duration) + poseMobile.pose.position.y    ;
    poseWorld_theta = (theta * duration) + poseMobile_theta;
    if ((!std::isnan(poseWorld.pose.position.x)) && (!std::isnan(poseWorld.pose.position.y)) && (!std::isnan(poseWorld_theta))) {
        poseWorld.header.stamp = ros::Time::now();
        publisherMobile.publish(poseWorld);
        poseMobile.pose.position.x     = poseWorld.pose.position.x    ;
        poseMobile.pose.position.y     = poseWorld.pose.position.y    ;
        poseMobile_theta = poseWorld_theta;
        
        tf::Quaternion q;
        q.setRPY(0,0,poseMobile_theta);

        poseMobile.pose.orientation.x = q.x();
        poseMobile.pose.orientation.y = q.y();
        poseMobile.pose.orientation.z = q.z();
        poseMobile.pose.orientation.w = q.w();
        poseWorld.pose.orientation=poseMobile.pose.orientation;
        //poseMobile.orientation.setRPY(0,0,poseMobile_theta);
    }
    //std::cout<< poseWorld.x << " " << poseWorld.y << " " << poseWorld.theta << "\n";
}

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
    timePrevious = timeCurrent;
    theta_left_previous  = theta_left_current ;
    theta_back_previous  = theta_back_current ;
    theta_right_previous = theta_right_current;
    onEncoderMessage();
    std::cout<< duration<< "\n";

}



void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
   for (i = 0; i < input->name.size(); i++) {
       if (((input->name[i]).find("dot") == std::string::npos) || ((input->name[i]).find("dot") == std::string::npos)) {
           continue;
       }
       poseCheat = input->pose[i];
    //    tf::Quaternion q(
    //    poseCheat.orientation.x,
    //    poseCheat.orientation.y,
    //    poseCheat.orientation.z,
    //    poseCheat.orientation.w);
    //    tf::Matrix3x3 m(q);
      // m.getRPY(rotX, rotY, rotZ);
       poseWorld1.pose.position.x     = poseCheat.position.x;
       poseWorld1.pose.position.y     = poseCheat.position.y;
       //poseWorld1.pose.theta = rotZ;
       poseWorld1.pose.orientation = poseCheat.orientation;
       poseWorld1.header.stamp = ros::Time::now();
       publisherWorld.publish(poseWorld1);
   }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("/dot/joint_states", 1, onJointStateMessage);
    ros::Subscriber gaz_sub = node.subscribe("/gazebo/model_states", 1, onGazeboMessage);
    ros::Subscriber subscriber1;
    poseWorld.pose.position.z = 0.1;
    poseWorld1.pose.position.z = 0.2;
    poseWorld.header.frame_id = "origin_link";
    poseWorld1.header.frame_id = "origin_link";
   
    publisherWorld  = node.advertise<geometry_msgs::PoseStamped>("pose/world" , 1);
    publisherMobile = node.advertise<geometry_msgs::PoseStamped>("pose/mobile", 1);
    ros::spin();
    return 0;
}
