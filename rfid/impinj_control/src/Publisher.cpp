
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv,"velocity_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.linear.y = 0.2;
        vel_msg.linear.z = 0.0;

        pub.publish(vel_msg);
        ROS_INFO("Publish robot velocity.");
        loop_rate.sleep();
    }
}
