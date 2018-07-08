#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle), angle(0.0), min(30.0), status(true) {
	if (!(nodeHandle.getParam("scan_topicName", topic_name_)
			& nodeHandle.getParam("scan_queueSize", queue_size_)
			& nodeHandle.getParam("twist_topic_name", publisher_topic_name_)
			& nodeHandle.getParam("pub_queueSize", pub_queue_size)
			& nodeHandle.getParam("controller_gain", gain)
			& nodeHandle.getParam("x_vel", forward_vel)
			& nodeHandle.getParam("marker_publiser_name", marker_publisherName)
			& nodeHandle.getParam("marker_queue_size", marker_queueSize))) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	sub_ = nodeHandle_.subscribe(topic_name_, queue_size_,
			&HuskyHighlevelController::subCallback, this);
	pub_vel_ = nodeHandle_.advertise<geometry_msgs::Twist>(
			publisher_topic_name_, pub_queue_size);
	pub_marker_ = nodeHandle_.advertise<visualization_msgs::Marker>(
			marker_publisherName, marker_queueSize);
	service_ = nodeHandle_.advertiseService("status_srv", &HuskyHighlevelController::statuscallback,this);
}

void HuskyHighlevelController::subCallback(const sensor_msgs::LaserScan& msg) {
	std::vector<float> ranges = msg.ranges;
	min = 30;
	int min_index = 0;
	for (int i = 0; i < ranges.size(); i++) {
		if (min > ranges[i]) {
			min = ranges[i];
			min_index = i;
		}
	}
//	float angle_increment = msg.angle_increment;
	angle = -0.785 + (min_index * 1.5 * pi / 720);
//	float req_angle = msg.angle_min - angle;
	if (status) {
		husky_vel_command.linear.x = forward_vel;
		husky_vel_command.angular.z = gain * (0 - (angle - pi / 2));
		pub_vel_.publish(husky_vel_command);
		publish_marker();
	} else {
		husky_vel_command.linear.x = 0;
		husky_vel_command.angular.z = 0;
		pub_vel_.publish(husky_vel_command);
		publish_marker();

	}
	//	ROS_INFO("\nmin : %f\nangle increment: %.3f\nangle: %.3f\nreq_angle: %.3f\n",min, angle_increment,angle,req_angle);
}

bool HuskyHighlevelController::statuscallback(
		std_srvs::SetBool::Request &request,
		std_srvs::SetBool::Response &response) {
	if (request.data) {
		response.success = true;
		response.message = ": robot started";
		status = true;
	} else {
		response.success = false;
		response.message = ": robot stopped";
		status = false;
	}
	ROS_INFO_STREAM("RESPONSE -"<<response.message);
	return true;
}
void HuskyHighlevelController::publish_marker() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_laser";
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = min * sin(angle);
	marker.pose.position.y = min * cos(angle);
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	pub_marker_.publish(marker);
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

} /* namespace */
