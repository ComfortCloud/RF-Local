#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <thread>
#include "impinj_control/RFIDdata.h"
#include "impinj_control/sendEnding.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <stdio.h>
#include <string>
#include "preprocessing.h"
#include "tf/transform_datatypes.h"
#include <ros/console.h>

class multiThreadListener
{

private:
    ros::NodeHandle n_main;
    ros::NodeHandle n_robot;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;


public:
    OdomData odom;
    std::vector<double> robotX_recv;
    std::vector<double> robotY_recv;
    std::vector<double> robotW_recv;
    std::vector<double> robotTimeStamp_recv;

    std::vector<EPC> RFIDBase;

    // count iteration time
    int count = 0;

    multiThreadListener(){}

    void startRunning();

    
    void chatterCallback1(const nav_msgs::Odometry::ConstPtr &msg);
    void chatterCallback2(const impinj_control::RFIDdata::ConstPtr &msg);
    void chatterCallback3(const impinj_control::sendEnding::ConstPtr &msg);

    void startCoreCalc();

};

// run spin
void multiThreadListener::startRunning()
{
    ros::CallbackQueue callback_queue_robot;
    n_robot.setCallbackQueue(&callback_queue_robot);
    sub1 = n_robot.subscribe<nav_msgs::Odometry>("/odom", 5, &multiThreadListener::chatterCallback1, this);
    sub2 = n_main.subscribe("Data_RFID", 20, &multiThreadListener::chatterCallback2, this);
    sub3 = n_main.subscribe("Ending_msg", 5, &multiThreadListener::chatterCallback3, this);
    std::thread spinner_thread_robot([&callback_queue_robot]()
                                     {
            ros::SingleThreadedSpinner spinner_robot;
            spinner_robot.spin(&callback_queue_robot); });
    ros::spin();
    spinner_thread_robot.join();
}


// callback of robot
void multiThreadListener::chatterCallback1(const nav_msgs::Odometry::ConstPtr &msg)
{
    //ROS_INFO("Receiving from robot.");

    // save robot's data
    robotTimeStamp_recv.push_back(msg->header.stamp.toSec());
    robotX_recv.push_back(msg->pose.pose.position.x);
    robotY_recv.push_back(msg->pose.pose.position.y);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double rollTemp = 0, pitchTemp = 0, yawTemp = 0;
    tf::Matrix3x3(quat).getRPY(rollTemp, pitchTemp, yawTemp);
    robotW_recv.push_back(yawTemp);
}


// call back of RFID
void multiThreadListener::chatterCallback2(const impinj_control::RFIDdata::ConstPtr &msg)
{
    ROS_INFO("Receiving from reader.");

    ROS_INFO("Receiving from tag: epc: %s, reader: %d, ant: %d", msg->epc.c_str(), msg->readerID,  msg->antID);

    // save RFID's data
    if (RFIDBase.empty())
    {
        ROS_INFO("Create new EPC object1.");
        EPC epc_new;
        strcpy(epc_new.epc, msg->epc.c_str());
        epc_new.reader[0].readerID = 1;
        epc_new.reader[1].readerID = 2;
        epc_new.reader[0].ant[0].antennaId = 1;
        epc_new.reader[0].ant[1].antennaId = 2;
        epc_new.reader[0].ant[2].antennaId = 3;
        epc_new.reader[1].ant[0].antennaId = 1;
        epc_new.reader[1].ant[1].antennaId = 2;
        epc_new.reader[1].ant[2].antennaId = 3;

        for (unsigned int i = 0; i < msg->phase.size(); i++) 
        {
            epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].phase.push_back(msg->phase[i]);
            epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].rssi.push_back(msg->rssi[i]);
            epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].timestamp.push_back(msg->timestamp[i]);
        }
            
        RFIDBase.push_back(epc_new);
        ROS_INFO("Succeed create new EPC object1.");
    }
    else
    {
        std::cout << "Append to existing EPC object?" << std::endl;
        int searchFlag = 0;
        for (unsigned int it = 0; it < RFIDBase.size(); it++)
        {
            if (strcmp(msg->epc.c_str(), RFIDBase[it].epc) == 0)
            {
                searchFlag = 1;
                std::cout << "Append to existing EPC object!" << std::endl;
                for (unsigned int i = 0; i < msg->phase.size(); i++)
                {
                    RFIDBase[it].reader[msg->readerID - 1].ant[msg->antID - 1].phase.push_back(msg->phase[i]);
                    RFIDBase[it].reader[msg->readerID - 1].ant[msg->antID - 1].rssi.push_back(msg->rssi[i]);
                    RFIDBase[it].reader[msg->readerID - 1].ant[msg->antID - 1].timestamp.push_back(msg->timestamp[i]);
                }
                break;
            }
            else
            {
                continue;
            }
        }
        if (!searchFlag)
        {
            ROS_INFO("Create new EPC object2.");
            EPC epc_new;
            strcpy(epc_new.epc, msg->epc.c_str());
            ROS_INFO("fallout1");
            epc_new.reader[0].readerID = 1;
            epc_new.reader[1].readerID = 2;
            ROS_INFO("fallout2");
            epc_new.reader[0].ant[0].antennaId = 1;
            epc_new.reader[0].ant[1].antennaId = 2;
            epc_new.reader[0].ant[2].antennaId = 3;
            epc_new.reader[1].ant[0].antennaId = 1;
            epc_new.reader[1].ant[1].antennaId = 2;
            epc_new.reader[1].ant[2].antennaId = 3;
            ROS_INFO("fallout3");
            if (msg->phase.size() == 0)
            {
                std::cout << "empty ant data" << std::endl;
            }
            else
            {
                for (unsigned int i = 0; i < msg->phase.size(); i++)
                {
                    epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].phase.push_back(msg->phase[i]);
                    epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].rssi.push_back(msg->rssi[i]);
                    epc_new.reader[msg->readerID - 1].ant[msg->antID - 1].timestamp.push_back(msg->timestamp[i]);
                }
            }
            ROS_INFO("fallout4");
            RFIDBase.push_back(epc_new);
        }
    }
    std::cout << "Have received " << RFIDBase.size() << " tags" << std::endl;
}

// call back of stop flag
void multiThreadListener::chatterCallback3(const impinj_control::sendEnding::ConstPtr &msg)
{
    ROS_INFO("Received ending info.");

    count++;
    // Data transport
    
    odom.timestamp.swap(robotTimeStamp_recv);
    odom.x.swap(robotX_recv);
    odom.y.swap(robotY_recv);
    odom.yaw.swap(robotW_recv);

    // for (int i = 0; i < RFIDBase[0].reader[0].ant[0].phase.size(); i++)
    // {
	// 	std::cout << RFIDBase[0].reader[0].ant[0].phase[i] << " ";
	// }
    std::cout << "Have received " << RFIDBase.size() << "tags" << std::endl;

    robotX_recv.clear();
    robotY_recv.clear();
    robotW_recv.clear();
    robotTimeStamp_recv.clear();

    startCoreCalc();

}

// main calculation core
void multiThreadListener::startCoreCalc()
{   
    char fileName1[200] = {0};
    std::ofstream data;
    sprintf(fileName1, "//home//yunshu//data//test//20230304//data.txt");
    if (data.fail())
    {
        ROS_INFO("Open file 1 failed");
    }
    else
    {
        data.open(fileName1, std::ios::out);
        ROS_INFO("file 1 is open");
        int cnt1 = 0;
        for (auto it1 = RFIDBase.begin(); it1 != RFIDBase.end(); it1++)
        {
            for (auto it2 = 0; it2 != 3; it2++)
            {
                std::cout << (*it1).epc << " " << (*it1).reader[0].readerID << " " << (*it1).reader[0].ant[it2].antennaId << " " << (*it1).reader[0].ant[it2].phase.size() << " " << (*it1).reader[0].ant[it2].rssi.size() << " " << (*it1).reader[0].ant[it2].timestamp.size() << std::endl;
                for (auto it3 = 0; it3 != it1->reader[0].ant[it2].phase.size(); it3++)
                {
                    data << (*it1).epc << " " << (*it1).reader[0].readerID << " " << std::fixed << (*it1).reader[0].ant[it2].antennaId << " " << (*it1).reader[0].ant[it2].phase[it3] << " " << (*it1).reader[0].ant[it2].rssi[it3] << " " << (*it1).reader[0].ant[it2].timestamp[it3]<< endl;
                    cnt1++;
                }
            }
            // data1 << ++cnt1 << " " << (*it1).epc << " " << (*it1).channelIndex << " " << ((*it1).channelIndex - 1) * 0.25 + 920.625 << " " << (*it1).phase << " " << (*it1).rssi << endl;
        }
        data.close();
    }

    char fileName2[200] = {0};
    std::ofstream Odom;
    sprintf(fileName2, "//home//yunshu//data//test//20230304//odom.txt");
    if (Odom.fail())
    {
        ROS_INFO("Open file 2 failed");
    }
    else
    {
        Odom.open(fileName2, std::ios::out);
        ROS_INFO("file 2 is open");
        //int cnt1 = 0;
        for (auto it1 = 0; it1 != odom.x.size(); it1++)
        {
            Odom << std::fixed << odom.x[it1] << " " << odom.y[it1] << " " << odom.yaw[it1] << " " << odom.timestamp[it1] << endl;
            //cnt1++;
            // data1 << ++cnt1 << " " << (*it1).epc << " " << (*it1).channelIndex << " " << ((*it1).channelIndex - 1) * 0.25 + 920.625 << " " << (*it1).phase << " " << (*it1).rssi << endl;
        }
        Odom.close();
    }


    std::cout << "---------------开始进行第" << count << "次定位计算---------------" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    ofstream EPCxyz("EPCxyz.txt");
    int count_it = 0;
    int index;
    for (int i = 0; i < RFIDBase.size(); i++)
    {
        LM EPC_localization;
        if (RFIDBase[i].reader[0].ant[0].phase.size() +
            RFIDBase[i].reader[0].ant[1].phase.size() + 
            RFIDBase[i].reader[0].ant[2].phase.size() < 
            RFIDBase[i].reader[1].ant[0].phase.size() + 
            RFIDBase[i].reader[1].ant[1].phase.size() + 
            RFIDBase[i].reader[1].ant[2].phase.size())
            index = 1;
        else
            index = 0;
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << RFIDBase[i].epc << std::endl;

        EPC_localization.MakeHessian(RFIDBase[i].reader[index], odom);
        if(EPC_localization.flag)
        {
            cout << "Succeed result!" << endl;
            EPCxyz << ++count_it << " " << RFIDBase[i].epc << " " << std::fixed << EPC_localization.a_new << " " << std::fixed << EPC_localization.b_new << " " << std::fixed << EPC_localization.c_new << std::endl;
        }
        else
        {
            std::cout << "No valid read tags." << std::endl;
        }
        std::cout << RFIDBase[i].epc << std::endl;
    }

    // clear vectors for the next time's calculation
    odom.x.clear();
    odom.y.clear();
    odom.yaw.clear();
    odom.timestamp.clear();
    RFIDBase.clear();

    std::cout << "---------------结束进行第" << count << "次定位计算---------------" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "multi_sub");

    multiThreadListener listener_obj;
    listener_obj.startRunning();

    ROS_INFO("Quit calculation core function.");

    return 0;
}
