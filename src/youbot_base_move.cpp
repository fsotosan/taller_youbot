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
void timerCallback(const ros::TimerEvent&);
void moveBase(float inLongV, float inTransV, float inAngV);
void stopBase();

int i = 0;
YouBotBase* myYouBotBase = 0;
bool myYouBotHasBase = false;
ros::Timer myTimer;
ros::NodeHandle* myNodeHandle = 0;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_base_move");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
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

	// Paramos el temporizador para evitar la detención automática de la base

	myTimer.stop();

	// Indicamos que se ha recibido un mensaje de velocidad

	ROS_INFO("%d - Recibido mensaje Twist: vLineal = (%g,%g,%g), vAngular = (%g,%g,%g)\n", i++, inTwist->linear.x, inTwist->linear.y, inTwist->linear.z, inTwist->angular.x, inTwist->angular.y, inTwist->angular.z);

	// Ejecutamos la orden

	moveBase(inTwist->linear.x, inTwist->linear.y, inTwist->angular.z);

	// Iniciamos un temporizador para detener el robot si no se reciben nuevas órdenes en un segundo

	myTimer = myNodeHandle->createTimer(ros::Duration(1.0),timerCallback, true);

}

void timerCallback(const ros::TimerEvent&) {

	ROS_INFO("Deteniendo la base por timeout");
	stopBase();
}

void stopBase() {

	moveBase(0.0,0.0,0.0);
}

void moveBase(float inLongV, float inTransV, float inAngV) {

	/*
	* la api OODL utiliza unidades boost
	*/
	quantity<si::velocity> longitudinalVelocity = inLongV * meter_per_second;
	quantity<si::velocity> transversalVelocity = inTransV * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = inAngV * radian_per_second;

	if (myYouBotHasBase) {
		myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
	} else {
		ROS_INFO("No hay base");
	}

}
