
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

geometry_msgs::WrenchStamped wrench33;
geometry_msgs::WrenchStamped wrench34;
geometry_msgs::WrenchStamped wrench41;
   // A callback function.  Executed each time a new wrench
   // message arrives.

void wrenchMessageReceived33(const geometry_msgs::WrenchStamped& inputWrenchStamped)
{
  geometry_msgs::WrenchStamped wrench_copy;
  wrench_copy.wrench.force=inputWrenchStamped.wrench.force;
  wrench_copy.wrench.force.x=inputWrenchStamped.wrench.force.x;

  wrench_copy.wrench.torque=inputWrenchStamped.wrench.torque;
  wrench33.wrench.force=wrench_copy.wrench.force;
  wrench33.wrench.torque=wrench_copy.wrench.torque;
  //wrench33.header.stamp = ros::Time::now();
  //printf("HI");


}

void wrenchMessageReceived34(const geometry_msgs::WrenchStamped& inputWrenchStamped)
{
  geometry_msgs::WrenchStamped wrench_copy;
  wrench_copy.wrench.force=inputWrenchStamped.wrench.force;
  wrench_copy.wrench.force.x=inputWrenchStamped.wrench.force.x;

  wrench_copy.wrench.torque=inputWrenchStamped.wrench.torque;
  wrench34.wrench.force=wrench_copy.wrench.force;
  wrench34.wrench.torque=wrench_copy.wrench.torque;

}

void wrenchMessageReceived41(const geometry_msgs::WrenchStamped& inputWrenchStamped)
{
  geometry_msgs::WrenchStamped wrench_copy;
  wrench_copy.wrench.force=inputWrenchStamped.wrench.force;
  wrench_copy.wrench.force.x=inputWrenchStamped.wrench.force.x;

  wrench_copy.wrench.torque=inputWrenchStamped.wrench.torque;
  wrench41.wrench.force=wrench_copy.wrench.force;
  wrench41.wrench.torque=wrench_copy.wrench.torque;

}



int main(int argc, char **argv) {

     // Initialize the ROS system and become a node.
     ros::init(argc, argv, "wrench_listener");

     ros::NodeHandle n;

     ros::Rate loop_rate(120);
     ros::Publisher wrench_rePublisher33 = n.advertise<geometry_msgs::WrenchStamped>("WrenchRepub/33", 1000);
     ros::Publisher wrench_rePublisher34 = n.advertise<geometry_msgs::WrenchStamped>("WrenchRepub/34", 1000);
     ros::Publisher wrench_rePublisher41 = n.advertise<geometry_msgs::WrenchStamped>("WrenchRepub/41", 1000);
     ros::Subscriber sub = n.subscribe("/optoforce_node/wrench_UCE0B233", 1000, &wrenchMessageReceived33);
     ros::Subscriber sub2 = n.subscribe("/optoforce_node/wrench_UCE0B234", 1000, &wrenchMessageReceived34);
     ros::Subscriber sub3 = n.subscribe("/optoforce_node/wrench_UCE0B241", 1000, &wrenchMessageReceived41);

     while (ros::ok())
     {
       // Create a subscriber object.
       wrench33.header.stamp = ros::Time::now();
       wrench34.header.stamp = wrench33.header.stamp;
       wrench41.header.stamp = wrench33.header.stamp;

       wrench_rePublisher33.publish(wrench33);
       wrench_rePublisher34.publish(wrench34);
       wrench_rePublisher41.publish(wrench41);
       // Let ROS take over.
       ros::spinOnce();


       loop_rate.sleep();

     }

}
