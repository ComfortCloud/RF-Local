#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <iostream>
#include "tf/transform_datatypes.h"
#include <fstream>

class OdomPublish{
    private:
        ros::NodeHandle nh;
        ros::Subscriber odom_sub;
        std::vector<nav_msgs::Odometry> odomVec;
        std::vector<double> time;
        std::vector<double> posX;
        std::vector<double> posY;
        std::vector<double> posZ;
//        std::vector<double> posW;
        std::vector<double> roll;
        std::vector<double> pitch;
        std::vector<double> yaw;

    public:
        OdomPublish():nh("~"){
            odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &OdomPublish::subCallback, this);
        }

        void subCallback(const nav_msgs::Odometry::ConstPtr& odom){
            time.push_back(odom->header.stamp.toSec());
            posX.push_back(odom->pose.pose.position.x);
            posY.push_back(odom->pose.pose.position.y);
//            posZ.push_back(odom->pose.pose.position.z);
//            posW.push_back(odom->pose.pose.orientation);
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
            double rollTemp = 0, pitchTemp = 0, yawTemp = 0;
            tf::Matrix3x3(quat).getRPY(rollTemp, pitchTemp, yawTemp);
            roll.push_back(rollTemp);
            pitch.push_back(pitchTemp);
            yaw.push_back(yawTemp);
        }

        

        ~OdomPublish(){
            if(time.size() != 0){
                std::cout << "Saving the file..." << std::endl;
                char fileName[200] = {0};
                unsigned int current_Time = ros::Time::now().toSec();
                std::ofstream outFile;
                sprintf(fileName, "//home//yunshu//data//test//20230304//odom//%d.txt", current_Time);
                outFile.open(fileName, std::ios::out);
                for (unsigned long i = 0; i < time.size(); ++i){
                    outFile << std::fixed << posX[i] << "\t" << std::fixed << posY[i] << "\t" << std::fixed << yaw[i] << "\t" << std::fixed << time[i]<< std::endl;
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