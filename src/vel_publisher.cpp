/*
 vel_publisher
 Autor: fsotosan
 Programa para publicar mensajes Twist en el topic ROS 'cmd_ref_vel' usando las teclas de flecha, la 'x' y la 'y'
 La lectura de teclado usando termios se ha extraido de turtlebot_teleop http://github.com/turtlebot/turtlebot_apps.git
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <termios.h>



const double LINEAR_VEL = 0.1; // metros por segundo
const double PI = 3.1415926;
const double ANGULAR_VEL = PI/8; // radianes por segundo


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


using namespace std;
int kfd = 0;
struct termios cooked, raw;


geometry_msgs::Twist getVelInput();
void initConsole();

int main(int argc, char **argv) {

  ros::init(argc, argv, "vel_publisher");

  ros::NodeHandle theNodeHandle;

  ros::Publisher thePublisher = theNodeHandle.advertise<geometry_msgs::Twist>("cmd_ref_vel",100); // Publicaremos en el canal 'cmd_vel_ref'. Buffer de 100 mensajes

  ros::Rate theLoopRate(10);  // 10 Hz 

  geometry_msgs::Twist theVel;

  initConsole();

  cout << "Pulsa las flechas para avanzar. Pulsa 'x' o 'y' para girar" << endl;

  while(ros::ok()) {
    theVel = getVelInput();
    thePublisher.publish(theVel);
    theLoopRate.sleep();
  }

  cout << "Programa terminado" << endl;
  tcsetattr(kfd, TCSANOW, &cooked);
  return 0;

}

geometry_msgs::Twist getVelInput() {

  int c = 0;
  geometry_msgs::Twist outVel;

  outVel.linear.x = 0.0;
  outVel.linear.y = 0.0;
  outVel.linear.z = 0.0;
  outVel.angular.x = 0.0;
  outVel.angular.y = 0.0;
  outVel.angular.z = 0.0;

  printf("\r");

  // get the next event from the keyboard  
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }
 
  switch (c) {
    case KEYCODE_D:
      outVel.linear.x = -LINEAR_VEL;
      break;
    case KEYCODE_U:
      outVel.linear.x = LINEAR_VEL;
      break;
    case KEYCODE_L:
      outVel.linear.y = -LINEAR_VEL;
      break;
    case KEYCODE_R:
      outVel.linear.y = LINEAR_VEL;
      break;
    case 'y':
      outVel.angular.z = ANGULAR_VEL;
      break;
    case 'x':
      outVel.angular.z = -ANGULAR_VEL;
      break;
    default:
      break;
  }

  return outVel;

}


void initConsole() {

  // Código extraído de turtlebot_teleop

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);


}
