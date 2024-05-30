#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Publisher pub;

// Forces callback function
void forcesCb(ConstWrenchStampedPtr &_msg){
    geometry_msgs::WrenchStamped wrench_msg;
    // std::cout << "Force: " << _msg->wrench().force().x() << " " << _msg->wrench().force().y() << " " << _msg->wrench().force().z();

    wrench_msg.wrench.force.x = _msg->wrench().force().x();
    wrench_msg.wrench.force.y = _msg->wrench().force().y();
    wrench_msg.wrench.force.z = _msg->wrench().force().z();
    wrench_msg.wrench.torque.x = _msg->wrench().torque().x();
    wrench_msg.wrench.torque.y = _msg->wrench().torque().y();
    wrench_msg.wrench.torque.z = _msg->wrench().torque().z();

    wrench_msg.header.stamp = ros::Time::now();
    pub.publish(wrench_msg);
}


int main(int _argc, char **_argv){

     // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "force_measure");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::WrenchStamped>("forces/gazebo", 1000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/aerotrain/joint_revo/force_torque/wrench", forcesCb);

    // Busy wait loop...replace with your own code as needed.
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


    // Mayke sure to shut everything down.

    }
    gazebo::client::shutdown();
}

