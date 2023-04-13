#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

main(int argc, char **argv)
{
	ros::init(argc, argv, "map");
	ros::NodeHandle n;
	ros::Publisher EPC_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
	ros::Publisher EPCtext_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

	/**
	 * ifstream file;
	// 数据流读取文件
	file.open("//home//yunshu//data//EPCxyz");
	vector<string> data;

	while (!file.eof())
	{
		string s;
		getline(file, s);
		if (!s.empty())
		{
			data.push_back(s);
		}
	}
	 * **/

	std::cout << "-1-" << std::endl;
	string x, y, z, text;
	// Set our initial shape type to be a cube
	uint32_t EPC_shape = visualization_msgs::Marker::SPHERE;
	uint32_t text_shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
	visualization_msgs::Marker EPC;
	visualization_msgs::MarkerArray markerArray_EPC_msg;
	visualization_msgs::Marker EPCtext;
	visualization_msgs::MarkerArray markerArray_EPCtext_msg;

	std::cout << "-5-" << std::endl;

	Eigen::MatrixXd data(60,3);
	vector<string> epc = {
		"E200-001A-0411-0173-1040-96CE",
		"E200-001A-0411-0239-1040-D703",
		"E200-001A-0411-0251-1040-E41D",
		"E200-001A-0411-0210-1040-BE31",
		"E200-001A-0411-0252-1040-E28A",
		"E200-001A-0411-0253-1040-E41E",
		"E200-001A-0411-0250-1040-E289",
		"E200-001A-0411-2000-1080-0039",
		"E200-001A-0411-0236-1040-D50E",
		"E200-001A-0411-2000-1080-0042",
		"E200-001A-0411-0226-1040-CDC1",
		"E200-001A-0411-0235-1040-D701",
		"E200-001A-0411-0237-1040-D702",
		"E200-0019-340C-0062-0820-201F",
		"E200-001A-0411-2000-1080-0040",
		"E200-001A-0411-0213-1040-C0AE",
		"E200-001A-0411-0211-1040-C0AD",
		"E200-001A-0411-0225-1040-C870",
		"E200-001A-0411-2000-1080-0042",
		"E200-001A-0411-0196-1040-ADBE",
		"E200-001A-0411-0200-1040-ADC0",
		"E200-001A-0411-0197-1040-B05A",
		"E200-001A-0411-0212-1040-BE32",
		"E200-001A-0411-0223-1040-C86F",
		"E200-001A-0411-0160-1040-8274",
		"E200-001A-0411-0158-1040-8273",
		"E200-001A-0411-0172-1040-93F2",
		"E200-001A-0411-0170-1040-93F1",
		"E200-001A-0411-0199-1040-B05B",
		"E200-001A-0411-0186-1040-A549",
		"E200-001A-0411-2000-1080-0037",
		"E200-001A-0411-0182-1040-9CAF",
		"E200-001A-0411-0185-1040-9F6C",
		"E200-001A-0411-2171-1040-96CD",
		"E200-001A-0411-2000-1080-0041",
		"E200-001A-0411-0146-1040-79B1",
		"E200-001A-0411-0148-1040-79B2",
		"E200-001A-0411-0145-1040-73C4",
		"E200-001A-0411-0157-1040-8542",
		"E200-001A-0411-0183-1040-9F6B",
		"E200-001A-0411-0159-1040-8543",
		"E200-001A-0411-0135-1040-6B0F",
		"E200-001A-0411-0133-1040-6B0E",
		"E200-001A-0411-0136-1040-6834",
		"E200-001A-0411-0134-1040-6833",
		"E200-001A-0411-0147-1040-7C81",
		"E200-001A-0411-0264-1040-E89C",
		"E200-001A-0411-0266-1040-EDF1",
		"E200-0019-340C-0061-0820-215A",
		"E200-001A-0411-0265-1040-E9C0",
		"E200-001A-0411-2000-1080-0036",
		"E200-001A-0411-0276-1040-F2AA",
		"E200-001A-0411-0278-1040-F2AB",
		"E200-001A-0411-0279-1040-F30B",
		"E200-001A-0411-0277-1040-F30A",
		"E200-001A-0411-0275-1030-F315",
		"E200-001A-0411-0272-1030-EDF0",
		"E200-001A-0411-0274-1030-F2B1",
		"E200-001A-0411-0261-1030-E9B2",
		"E200-001A-0411-0273-1030-EED4"
	};

	std::cout << "-4-" << std::endl;


	data << 2.169,	1.03,	1.473,
 			2.596,	1.03,	1.588,
			2.599,	1.03,	1.369,
			3.15,	1.03,	1.515,
			3.304,	1.13,	1.442,
			1.976,	1.03,	0.639,
			1.966,	1.03,	0.431,
			2.579,	1.03,	0.647,
			3.449,	1.03,	0.62,
			3.286,	1.08,	0.613,
			4.681,	1.03,	1.549,
			4.661,	1.03,	1.41,
			5.09,	1.082,	1.553,
			5.065,	1.03,	1.406,
			5.233,	1.03,	1.57,
			6.118,	1.03,	1.405,
			4.836,	1.03,	0.644,
			6.101,	1.03,	0.512,
			5.923,	1.03,	0.598,
			4.63,	-1.251,	1.303,
			5.096,	-1.251,	1.306,
			4.766,	-1.281,	0.584,
			5.163,	-1.281,	0.597,
			5.588,	-1.281,	0.59,
			5.92,	-2.278,	1.396,
			5.505,	-2.278,	1.4,
			4.693,	-2.278,	1.402,
			6.18,	-2.278,	0.444,
			4.755,	-2.278,	0.573,
			1.861,	-1.281,	1.411,
			2.035,	-1.281,	1.571,
			2.844,	-1.281,	1.415,
			2.048,	-1.281,	0.478,
			2.463,	-1.281,	0.434,
			2.86,	-1.281,	0.613,
			1.976,	-2.278,	1.455,
			3.271,	-2.278,	1.421,
			2.098,	-2.278,	1.449,
			2.479,	-2.278,	0.622,
			2.832,	-2.278,	0.483,
			3.494,	-2.278,	0.437,
			1.954,	-4.614,	1.613,
			1.931,	-4.614,	1.405,
			2.124,	-4.614,	1.614,
			2.135,	-4.614,	1.419,
			2.788,	-4.614,	1.464,
			3.385,	-4.669,	1.434,
			1.935,	-4.667,	0.619,
			2.343,	-4.614,	0.626,
			2.898,	-4.614,	0.619,
			3.305,	-4.614,	0.618,
			4.732,	-4.614,	1.571,
			4.733,	-4.614,	1.398,
			4.88,	-4.549,	1.289,
			5.544,	-4.614,	1.438,
			5.556,	-4.614,	1.295,
			4.804,	-4.614,	0.646,
			5.642,	-4.614,	0.423,
			6.045,	-4.614,	0.596,
			6.045,	-4.614,	0.426;

	std::cout << "-2-" << std::endl;

	for (unsigned int i = 0; i < epc.size(); i++)
	{
		std::cout << i << std::endl;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		// 设置frame ID 和 时间戳
		EPC.header.frame_id = "EPC";
		EPC.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		// 为这个marker设置一个独一无二的ID，一个marker接收到相同ns和id就会用新的信息代替旧的
		EPC.ns = "EPC_shapes";
		EPC.id = i;
		EPC.type = EPC_shape;
		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		EPC.action = visualization_msgs::Marker::ADD;
		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		// 设置marker的位置
		EPC.pose.position.x = data(i,0);
		EPC.pose.position.y = data(i,1);
		EPC.pose.position.z = data(i,2);

		EPC.pose.orientation.x = 0.0;
		EPC.pose.orientation.y = 0.0;
		EPC.pose.orientation.z = 0.0;
		EPC.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		// 设置marker的大小
		EPC.scale.x = 0.05;
		EPC.scale.y = 0.05;
		EPC.scale.z = 0.05;
		// Set the color -- be sure to set alpha to something non-zero!
		// 设置marker的颜色
		EPC.color.r = 0.0f;
		EPC.color.g = 1.0f;
		EPC.color.b = 0.0f;
		EPC.color.a = 1.0;

		EPC.lifetime = ros::Duration();

		markerArray_EPC_msg.markers.push_back(EPC);



		EPCtext.header.frame_id = "EPC";
		EPCtext.header.stamp = ros::Time::now();

		EPCtext.ns = "EPC_text";
		EPCtext.id = i;
		EPCtext.type = text_shape;

		EPCtext.action = visualization_msgs::Marker::ADD;

		text = epc[i];
		EPCtext.text =text.c_str();
		ROS_INFO("%s  %f  %f  %f", EPCtext.text.c_str(), EPC.pose.position.x, EPC.pose.position.y, EPC.pose.position.z);

		EPCtext.pose.position.x = EPC.pose.position.x;
		EPCtext.pose.position.y = EPC.pose.position.y;
		EPCtext.pose.position.z = EPC.pose.position.z+0.05;

		EPCtext.pose.orientation.x = 0.0;
		EPCtext.pose.orientation.y = 0.0;
		EPCtext.pose.orientation.z = 0.0;
		EPCtext.pose.orientation.w = 1.0;

		EPCtext.scale.x = 0.05;
		EPCtext.scale.y = 0.05;
		EPCtext.scale.z = 0.05;

		EPCtext.color.r = 1.0f;
		EPCtext.color.g = 0.0f;
		EPCtext.color.b = 0.0f;
		EPCtext.color.a = 1.0;

		EPCtext.lifetime = ros::Duration();

		markerArray_EPCtext_msg.markers.push_back(EPCtext);
	}

	for (int i = 0; i < data.size(); i++)
	{
	}

	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		EPC_pub.publish(markerArray_EPC_msg);
		EPCtext_pub.publish(markerArray_EPCtext_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
