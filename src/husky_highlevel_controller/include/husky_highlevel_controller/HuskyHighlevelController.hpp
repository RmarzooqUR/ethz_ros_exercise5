#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "std_srvs/SetBool.h"
namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	void subCallback(const sensor_msgs::LaserScan& scan);
	bool statuscallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
	void publish_marker();
	ros::NodeHandle& nodeHandle_;
	ros::Subscriber sub_;
	ros::Publisher pub_vel_, pub_marker_;
	ros::ServiceServer service_;
	geometry_msgs::Twist husky_vel_command;

	std::string topic_name_;
	std::string publisher_topic_name_;
	std::string marker_publisherName;
	int marker_queueSize;
	int queue_size_;
	int pub_queue_size;
	float gain;
	float forward_vel;
	float pi = 3.14159;
	float min;
	float angle;
	bool status;
};

} /* namespace */
