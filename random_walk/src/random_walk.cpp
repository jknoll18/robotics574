#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <cmath>
class RandomWalk {
public:
// Construst a new RandomWalkobject and hook up this ROS node
// to the simulated robot's velocity control and laser topics
RandomWalk(ros::NodeHandle& nh) :
fsm(FSM_MOVE_FORWARD),
rotateStartTime(ros::Time::now()),
rotateDuration(0.f)
 {
// Initialize randomtime generator
srand(time(NULL));
// Advertise a new publisher for the simulated robot's velocity command topic
// (the second argument indicates that if multiple command messages are in
//  the queue to be sent, only the last command will be sent)
commandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
//commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
// Subscribe to the simulated robot's laser scan topic and tell ROS to call
// this->commandCallback() whenever a new message is published on that topic
//laserSub = nh.subscribe("base_scan",1,&RandomWalk::commandCallback,this);
laserSub = nh.subscribe("scan", 1, &RandomWalk::commandCallback, this);
};
// Send a velocity command 
void move(double linearVelMPS, double angularVelRadPS) {
geometry_msgs::Twist msg; // The default constructor will set all commands to 0
msg.linear.x = linearVelMPS;
msg.angular.z = angularVelRadPS;
commandPub.publish(msg);
ROS_INFO_STREAM("linear x: " << msg.linear.x);
//ROS_TOPIC_INFO("msg: " << msg);
};
// Process the incoming laser scan message
void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
if (fsm == FSM_MOVE_FORWARD) {


ROS_INFO_STREAM("In callback");
float AVG_RANGE = float(MIN_SCAN_ANGLE_RAD + MAX_SCAN_ANGLE_RAD) / 2.0f;
unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD -msg->angle_min) / msg->angle_increment);
ROS_INFO_STREAM("minIndex: "<< msg->angle_min);
ROS_INFO_STREAM("maxIndex: "<< msg->angle_max);
float closestRange = msg->ranges[minIndex];
for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
  float currAngle = msg->angle_min + msg->angle_increment*currIndex;
  if (!(currAngle <= msg->angle_max)){ 
    fsm = FSM_ROTATE;
  }
  if (msg->ranges[currIndex] <closestRange){
    closestRange = msg->ranges[currIndex];
  }
  if (isnan(closestRange)){
    closestRange = 2;
  }
}
ROS_INFO_STREAM("Range: " << closestRange);

if (closestRange < PROXIMITY_RANGE_M){
  rotateStartTime = ros::Time::now(); //ehhhh
  int direction = rand() % 2 + 1;
  if(direction == 1){
  rotateDuration = ros::Duration(4.0f - (3.0f/float(rand() % 4 + 1)));
  ROS_INFO_STREAM("neg: " << rotateDuration);
  }
  else{
  rotateDuration = ros::Duration(4.0f + (3.0f/float(rand() % 4 + 1)));
  ROS_INFO_STREAM("pos: " << rotateDuration);
  }
  fsm = FSM_ROTATE;
}
}
};
// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void spin() {
ros::Rate rate(10); // Specify the FSM loop rate in Hz
while(ros::ok()) { // Keep spinning loop until user presses Ctrl+C

if(fsm == FSM_MOVE_FORWARD) {
  ROS_INFO_STREAM("is forward:" << fsm);
  move(FORWARD_SPEED_MPS, 0);
}
else{
  ROS_INFO_STREAM("is rotate:" << fsm);
  move(0, ROTATE_SPEED_RADPS);
  if (ros::Time::now() >= rotateStartTime + rotateDuration) {
    fsm = FSM_MOVE_FORWARD;
  }
}
ros::spinOnce(); 
// Need to call this function often to allow ROS to process incoming messages
rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
}
};
//tunable parameters
enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
const static double FORWARD_SPEED_MPS = 0.2;
const static double ROTATE_SPEED_RADPS = M_PI/4;
protected:
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
enum FSM fsm; // Finite state machine for the random walk algorithm
ros::Time rotateStartTime; // Start time of the rotation
ros::Duration rotateDuration; // Duration of the rotation
};
int main(int argc, char**argv) {
ros::init(argc, argv, "random_walk"); // Initiate new ROS node named "random_walk"
ros::NodeHandle n;
ROS_INFO_STREAM("In main");
RandomWalk walker(n); // Create new random walk object
walker.spin(); // Execute FSM loop
return 0;
};
