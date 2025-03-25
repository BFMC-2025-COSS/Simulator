#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Float64.h>

// const double WHEEL_BASE = 0.26;  

double x_ = 11.77, y_ = 2.05, yaw_ = 0.0; 
double linear_velocity_ = 0.0;  
double angular_velocity_ = 0.0;  

ros::Publisher odom_pub_;
ros::Time current_time_, last_time_;


double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;  
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    yaw_ = getYawFromQuaternion(msg->orientation);
    angular_velocity_ = msg->angular_velocity.z; 
    // ROS_INFO("IMU Yaw: %f rad, Angular Velocity: %f rad/s", yaw_, angular_velocity_);
}


void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
    linear_velocity_ = msg->data ;  
}

void updateOdometry() {
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();
    last_time_ = current_time_;


    double dx = linear_velocity_ * cos(yaw_) * dt;
    double dy = linear_velocity_ * sin(yaw_) * dt;

    x_ += dx;
    y_ += dy;
    ROS_INFO("Odometry Update: dt=%.3f, linear_velocity=%.3f, yaw=%.3f", dt, linear_velocity_, yaw_);
    ROS_INFO("Position: x=%.3f, y=%.3f", x_, y_);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

    odom.twist.twist.linear.x = linear_velocity_;
    odom.twist.twist.angular.z = angular_velocity_;

    odom_pub_.publish(odom);


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_time_, "odom", "base_link"));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_odom_node");
    ros::NodeHandle nh;

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    ros::Subscriber imu_sub = nh.subscribe("/automobile/IMU", 10, imuCallback);
    ros::Subscriber speed_sub = nh.subscribe<std_msgs::Float64>("/pp/speed", 10, speedCallback);


    current_time_ = ros::Time::now();
    last_time_ = current_time_;

    ros::Rate rate(50);  
    while (ros::ok()) {
        ros::spinOnce();
        updateOdometry();
        rate.sleep();
    }
    return 0;
}
