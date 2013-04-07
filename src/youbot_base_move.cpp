/*
 * youbot_base_move.cpp
 * Programa para mover la base del youbot a partir de mensajes de velocidad publicados en el topic 'cmd_ref_vel'
 * Autor: fsotosan
 * La parte de suscripción a un topic está extraída de https://raw.github.com/ros/ros_tutorials/groovy-devel/roscpp_tutorials/listener/listener.cpp
 * La parte de movimiento del youbot está extraída de https://github.com/youbot/youbot_applications.git (hello_world_demo)
 * */

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace youbot;
using namespace std;

void velCallback(const geometry_msgs::Twist::ConstPtr& inTwist);
int i = 0;
YouBotBase* myYouBotBase = 0;
bool myYouBotHasBase = false;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_base_move");
	ros::NodeHandle theNodeHandle;
	ros::Subscriber theSubscriber = theNodeHandle.subscribe("cmd_ref_vel", 10, velCallback);

	ROS_INFO("Programa youbot_base_move iniciado");
	ROS_INFO("YOUBOT_CONFIGURATIONS_DIR: %s",YOUBOT_CONFIGURATIONS_DIR);

	try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotBase->doJointCommutation();
		myYouBotHasBase = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		myYouBotHasBase = false;
	}

	ros::spin();

	cout << "Programa terminado" << endl;
	return 0;

}


void velCallback(const geometry_msgs::Twist::ConstPtr& inTwist) {

	ROS_INFO("%d - Recibido mensaje Twist: vLineal = (%g,%g,%g), vAngular = (%g,%g,%g)\n", i++, inTwist->linear.x, inTwist->linear.y, inTwist->linear.z, inTwist->angular.x, inTwist->angular.y, inTwist->angular.z);

	/*
	* la api OODL utiliza unidades boost
	*/
	quantity<si::velocity> longitudinalVelocity = inTwist->linear.x * meter_per_second;
	quantity<si::velocity> transversalVelocity = inTwist->linear.y * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = inTwist->angular.z * radian_per_second;

	if (myYouBotHasBase) {
		myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
	} else {
		ROS_INFO("No hay base");
	}

}
