#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <vector>
#include <iostream>
#include "tf/transform_datatypes.h"
#include <fstream>

class OdomPublish{
    private:
        ros::NodeHandle nh;
        ros::Subscriber laser_odom_sub;
        ros::Subscriber signal_save;
        std::vector<nav_msgs::Odometry> odomVec;
        std::vector<double> time;
        std::vector<double> posX;
        std::vector<double> posY;
        std::vector<double> posZ;
        std::vector<double> roll;
        std::vector<double> pitch;
        std::vector<double> yaw;

    public:
        OdomPublish():nh("~"){
            laser_odom_sub = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &OdomPublish::subCallback, this);
            signal_save = nh.subscribe<std_msgs::String>("/save", 5, &OdomPublish::subSaveCallback, this);

        }

        void subCallback(const nav_msgs::Odometry::ConstPtr& odom){
            time.push_back(odom->header.stamp.toSec());
            posX.push_back(odom->pose.pose.position.x);
            posY.push_back(odom->pose.pose.position.y);
            // posZ.push_back(odom->pose.pose.position.z);
            // tf::Quaternion quat;
            // tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
            // double rollTemp = 0, pitchTemp = 0, yawTemp = 0;
            // tf::Matrix3x3(quat).getRPY(rollTemp, pitchTemp, yawTemp);
            // roll.push_back(rollTemp);
            // pitch.push_back(pitchTemp);
            // yaw.push_back(yawTemp);
        }

        void subSaveCallback(const std_msgs::String::ConstPtr& string){
            ros::shutdown();
        }

        ~OdomPublish(){
            if(time.size() != 0){
                std::cout << "Saving the file..." << std::endl;
                char fileName[200] = {0};
                unsigned int current_Time = ros::Time::now().toSec();
                std::ofstream outFile;
                sprintf(fileName, "//home//jetson//data//test//odom//laser%d.txt", current_Time);
                outFile.open(fileName, std::ios::out);
                for (unsigned long i = 0; i < time.size(); ++i){
                    outFile << std::fixed << posX[i] << "\t" << std::fixed << posY[i] << "\t" << std::fixed << time[i] << std::endl;
                }
                outFile.close();
                std::cout << "Finished!" << std::endl;
            }
            else{
                std::cout << "I do not subscribe any content, stop saving the file..." << std::endl;
            }
        }
        
};



int main(int argc, char** argv){
    ros::init(argc, argv, "odom_publish");
    OdomPublish odomPublish;
    std::cout << "Running..." << std::endl;
    ros::spin();

    return 0;
}