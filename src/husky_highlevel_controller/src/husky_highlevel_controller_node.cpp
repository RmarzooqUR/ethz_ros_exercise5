#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

using namespace husky_highlevel_controller;

int main(int argc, char** argv) {
	ros::init(argc, argv, "husky_highlevel_controller");
	ros::NodeHandle nodeHandle("~");

	HuskyHighlevelController huskyHighlevelController(nodeHandle);

	ros::spin();
	return 0;
}
