#include <ros/ros.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "tvcloc_client/Encoder.h"

ros::Publisher enc_pub;
ros::Subscriber enc_sub;

void ticksCB(geometry_msgs::Vector3Stamped msg) {
  tvcloc_client::Encoder enc_msg;
  enc_msg.header = msg.header;
  enc_msg.encoder_left = msg.vector.x;
  enc_msg.encoder_right = msg.vector.y;
  ROS_INFO("Encoder: %i - %i", enc_msg.encoder_left, enc_msg.encoder_right);
  enc_pub.publish(enc_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "convert_encoder_topic");
  ros::NodeHandle node("~");
  enc_pub = node.advertise<tvcloc_client::Encoder>("/encoder", 100);
  enc_sub = node.subscribe<geometry_msgs::Vector3Stamped>("/ticks", 100, ticksCB);

  ros::Rate rate(50);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

}