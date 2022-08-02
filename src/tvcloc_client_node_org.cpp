#include "tvcloc_client/tvcloc_client_node.h"
#include "ModbusClient.h"
#include "ModbusTCPClient.h"
#include "ModbusRTUClient.h"
#include "tvcloc_client/LocResult.h"

#include <iostream>

#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

TvcLocClient::TvcLocClient():priv_node_("~")
{
  readConfigs();
  if (createConnection(tcp_)) {
    ROS_INFO("Connected to server!");
  }
  else {
    // Todo add re-connect here
    ROS_ERROR("Connect to Server fail! Try again.");
    if (!retryConnect()){
      ROS_ERROR("Re-Connect to Server fail!");
      return;
    }
  }
  loc_out_pub_ = node_.advertise<tvcloc_client::LocResult>("/loc_result", 100);
  if(msg_type_ == 0x01) {
    odom_sub_ = node_.subscribe<nav_msgs::Odometry>(odom_topic_, 100, &TvcLocClient::odomCallback, this);
  }
  else if (msg_type_ == 0x02)
  {
    enc_sub_ = node_.subscribe<tvcloc_client::Encoder>(encoder_topic_, 100, &TvcLocClient::encoderCallback, this);
  }

  ros::Rate rate(rate_);
  while (ros::ok())
  {
    // Read input register
    inputDataArr_.clear();
    int nr = client_->requestFrom(slave_id_, INPUT_REGISTERS, startInputRegsAddr_, NUM_INPUT_REGS);
    if (nr != NUM_INPUT_REGS) {
      ROS_INFO_STREAM(nr);
      ROS_ERROR("Not read enough registers!");
      retryConnect();
    }

    // Decode data
    uint16_t input_data[NUM_INPUT_REGS];
    int i = 0;
    while (client_->available() > 0)
    {
      long result = client_->read();
      if (result == -1) {
        return;
      }
      input_data[i] = result;
      i++;
    }
    // LocOutput loc_output;
    if(input_data[0] != lastestModbusCnt_){
      tvcloc_client::LocResult loc_msg;
      loc_msg.header.seq = input_data[0];
      uint16_t arr_timestamp[4] = {input_data[1], input_data[2], input_data[3], input_data[4]};
      loc_msg.header.stamp.fromNSec(combine2uint64(arr_timestamp));
      uint16_t x_arr[2] = {input_data[5], input_data[6]};
      loc_msg.pose.position.x = combine2int32(x_arr)/1000.0;
      uint16_t y_arr[2] = {input_data[7], input_data[8]};
      loc_msg.pose.position.y = combine2int32(y_arr)/1000.0;
      uint16_t yaw_arr[2] = {input_data[9], input_data[10]};
      double yaw = ((double)combine2int32(yaw_arr)/1000.0) * M_PI / 180.0;
      tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
      tf::quaternionTFToMsg(quat, loc_msg.pose.orientation);
      loc_msg.confident = (input_data[11]) >> 8;
      loc_msg.matching_score = (int8_t)(input_data[11] & 0xFF);
      loc_msg.loc_status = (input_data[12] >> 8);
      lastestModbusCnt_ = input_data[0];    //update conter
      ROS_DEBUG_STREAM("loc: " << loc_msg);
      loc_out_pub_.publish(loc_msg);
      publishTF(loc_msg);
    }
    ros::spinOnce();
    rate.sleep();
  }
  client_->end();
  return;
}

TvcLocClient::~TvcLocClient() {
  delete client_;
}

bool TvcLocClient::readConfigs() {
  ROS_DEBUG("read configurations...");
  priv_node_.param<std::string>("ip", ip_address_, "127.0.0.1");
  priv_node_.param("port", port_, 502);
  priv_node_.param("spin_rate", rate_, 30);
  priv_node_.param("slave_id", slave_id_, 0x01);
  priv_node_.param("tcp", tcp_, true);
  priv_node_.param("map_frame", map_frame, std::string("map"));
  priv_node_.param("pose_frame", pose_frame, std::string("laser"));
  priv_node_.param("pub_frame", pub_frame, std::string("odom"));

  if (!priv_node_.getParam("regs_addr", regs_addr_)){
    ROS_WARN("No reg addrs was given!");
  }
  priv_node_.param("start_input_register", startInputRegsAddr_, 3000);
  priv_node_.param("start_holding_register", startHoldingRegsAddr_, 4000);

  priv_node_.param("msg_type", msg_type_, 0x1);
  if(msg_type_ == 0x01) {
    priv_node_.param("msg_type_version", msg_type_version_, 0x5);
    priv_node_.param<std::string>("odom_topic", odom_topic_, "odom");
  }
  else if (msg_type_ == 0x02)
  {
    priv_node_.param<std::string>("encoder_topic", encoder_topic_, "encoder");
  }

  priv_node_.param("num_retry", num_retry_, -1);
}

bool TvcLocClient::createConnection(bool tcp_mode) {
  int mb_connection = 0;
  // if(client_ != nullptr) {delete client_;}
  if(tcp_mode) {
    mb_connection = connectTCP();
  }
  else {
    mb_connection = connectRTU();
  }
  // client_->setDebug(1);
  if(mb_connection) {
    timeSync();
  }
  return (bool)mb_connection;
}

bool TvcLocClient::retryConnect()
{
  bool infinityLoop = false;

  if (num_retry_ == -1) {
    infinityLoop = true;
  }
  while (retry_cnt_++ < num_retry_ || infinityLoop)
  {
    client_->end();
    delete client_;
    if (createConnection(tcp_)) {
      ROS_INFO("Re-conect successful after %d times", retry_cnt_);
      retry_cnt_ = 0;
      return 1;
    }
    else {
      ROS_WARN("Connection created fail! Try again after 5s.Try %i times - Remain %i times.", retry_cnt_, num_retry_<=-1 ? -1 : num_retry_);
      sleep(5);
    }
  }
  return 0;
}


int TvcLocClient::connectTCP() {
  ROS_INFO("Run modbus TCP client");
  client_ = new ModbusTCPClient();
  ModbusTCPClient* tcp_client = dynamic_cast<ModbusTCPClient*>(client_);
  if(tcp_client) {
    return tcp_client->begin(ip_address_.c_str(), port_);
  }
  else
    return 0;
}

int TvcLocClient::connectRTU() {
  ROS_INFO("Run modbus RTU client");
  client_ = new ModbusRTUClientClass();
  ModbusRTUClientClass* rtu_client = dynamic_cast<ModbusRTUClientClass*>(client_);
  if(rtu_client) {
    return rtu_client->begin();
  }
  else
    return 0;
}

void TvcLocClient::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (msg_type_ == 0x01 && msg_type_version_ == 0x05) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    double yaw = tf::getYaw(quat);

    uint64_t timestamp = msg->header.stamp.toNSec();
    int32_t x = msg->pose.pose.position.x * 1000;   //mm/s
    int32_t y = msg->pose.pose.position.y * 1000;   //mm/s
    int32_t yawmD = yaw/M_PI * 180 * 1000;          //mdeg/s

    uint16_t dataArr[NUM_HOLDING_REGS_ODOM15];
    dataArr[0] = odom_counter_++;
    dataArr[1] = (uint16_t)((timestamp & 0xFFFF000000000000) >> 48);
    dataArr[2] = (uint16_t)((timestamp & 0xFFFF00000000) >> 32);
    dataArr[3] = (uint16_t)((timestamp & 0xFFFF0000) >> 16);
    dataArr[4] = (uint16_t)(timestamp & 0xFFFF);
    dataArr[5] = msg_type_ << 8 | msg_type_version_;
    dataArr[6] = 0; // for source id, but dont use now
    dataArr[7] = (uint16_t)((x & 0xFFFF0000) >> 16);
    dataArr[8] = (uint16_t)(x & 0xFFFF);
    dataArr[9] = (uint16_t)((y & 0xFFFF0000) >> 16);
    dataArr[10] = (uint16_t)(y & 0xFFFF);
    dataArr[11] = (uint16_t)((yawmD & 0xFFFF0000) >> 16);
    dataArr[12] = (uint16_t)(yawmD & 0xFFFF);

    std::stringstream ss;
    for (int i = 0; i < NUM_HOLDING_REGS_ENC; i++)
    {
      ss << dataArr[i] << " ";
    }
    ROS_DEBUG_STREAM("Transfer position-based odom DataArr:\n " << ss.str());

    if (!client_->beginTransmission(slave_id_, HOLDING_REGISTERS, startHoldingRegsAddr_, NUM_HOLDING_REGS_ODOM15)) {
      ROS_ERROR("Begin transmission fail!");
      return;
    }
    for (int i = 0; i < NUM_HOLDING_REGS_ODOM15; i++) {
      if (!client_->write(dataArr[i])) return;
    }
    if (!client_->endTransmission()) {
      ROS_ERROR("Write holding register fail!");
      return;
    }
  }
  else if (msg_type_== 0x01 && msg_type_version_ == 0x04)
  {
    uint64_t timestamp = msg->header.stamp.toNSec();
    int16_t vx = msg->twist.twist.linear.x * 1000;
    int16_t vy = msg->twist.twist.linear.y * 1000;
    int16_t omega = msg->twist.twist.angular.z * 1000;

    uint16_t dataArr[NUM_HOLDING_REGS_ODOM14];
    dataArr[0] = odom_counter_++;
    dataArr[1] = (uint16_t)((timestamp & 0xFFFF000000000000) >> 48);
    dataArr[2] = (uint16_t)((timestamp & 0xFFFF00000000) >> 32);
    dataArr[3] = (uint16_t)((timestamp & 0xFFFF0000) >> 16);
    dataArr[4] = (uint16_t)(timestamp & 0xFFFF);
    dataArr[5] = msg_type_ << 8 | msg_type_version_;
    dataArr[6] = 0; // for source id, but dont use now
    dataArr[7] = vx;
    dataArr[8] = vy;
    dataArr[9] = omega;

    std::stringstream ss;
    for (int i = 0; i < NUM_HOLDING_REGS_ENC; i++)
    {
      ss << dataArr[i] << " ";
    }
    ROS_DEBUG_STREAM("Transfer velocity-based odom DataArr:\n " << ss.str());

    if(!client_->beginTransmission(slave_id_, HOLDING_REGISTERS, startHoldingRegsAddr_, NUM_HOLDING_REGS_ODOM14)) {
      ROS_ERROR("Begin transmission fail!");
      return;
    }
    for (int i = 0; i < NUM_HOLDING_REGS_ODOM14; i++){
      if(!client_->write(dataArr[i])) {
        ROS_ERROR("Write data fail");
        return;
      }
    }
    if (!client_->endTransmission()) {
      ROS_ERROR("Transmission data fail!");
      return;
    }
  }
}

void TvcLocClient::encoderCallback(const tvcloc_client::Encoder::ConstPtr &msg)
{
  if (msg_type_ == 0x02) {
    uint64_t timestamp = msg->header.stamp.toNSec();
    int64_t left = msg->encoder_left;
    int64_t right = msg->encoder_right;
    uint16_t dataArr[NUM_HOLDING_REGS_ENC];
    dataArr[0] = odom_counter_++;
    dataArr[1] = (uint16_t)((timestamp & 0xFFFF000000000000) >> 48);
    dataArr[2] = (uint16_t)((timestamp & 0xFFFF00000000) >> 32);
    dataArr[3] = (uint16_t)((timestamp & 0xFFFF0000) >> 16);
    dataArr[4] = (uint16_t)(timestamp & 0xFFFF);
    dataArr[5] = msg_type_ << 8;
    dataArr[6] = 0; // for source id, but dont use now
    dataArr[7] = (uint16_t)((left & 0xFFFF000000000000) >> 48);
    dataArr[8] = (uint16_t)((left & 0xFFFF00000000) >> 32);
    dataArr[9] = (uint16_t)((left & 0xFFFF0000) >> 16);
    dataArr[10] = (uint16_t)(left & 0xFFFF);
    dataArr[11] = (uint16_t)((right & 0xFFFF000000000000) >> 48);
    dataArr[12] = (uint16_t)((right & 0xFFFF00000000) >> 32);
    dataArr[13] = (uint16_t)((right & 0xFFFF0000) >> 16);
    dataArr[14] = (uint16_t)(right & 0xFFFF);

    std::stringstream ss;
    for (int i = 0; i < NUM_HOLDING_REGS_ENC; i++)
    {
      ss << dataArr[i] << " ";
    }
    ROS_DEBUG_STREAM("Transfer Encoder DataArr: " << ss.str());

    if(!client_->beginTransmission(slave_id_, HOLDING_REGISTERS, startHoldingRegsAddr_, NUM_HOLDING_REGS_ENC)) {
      ROS_ERROR("Begin transmission fail!");
      return;
    }
    for (int i = 0; i < NUM_HOLDING_REGS_ENC; i++){
      if(!client_->write(dataArr[i])) {
        ROS_ERROR("Write data fail");
        return;
      }
    }
    if (!client_->endTransmission()) {
      ROS_ERROR("Transmission data fail!");
      return;
    }
  }
}

void TvcLocClient::timeSync()
{
  while(true) {
    uint16_t data[5];
    uint64_t T1 = ros::Time::now().toNSec();
    ROS_INFO("Send T1 time: %lu", T1);
    data[0] = 1;
    data[1] = (uint16_t)((T1 & 0xFFFF000000000000) >> 48);
    data[2] = (uint16_t)((T1 & 0xFFFF00000000) >> 32);
    data[3] = (uint16_t)((T1 & 0xFFFF0000) >> 16);
    data[4] = (uint16_t)(T1 & 0xFFFF);
    if(!client_->beginTransmission(slave_id_, HOLDING_REGISTERS, startHoldingRegsAddr_ + 15, 5)) {
      ROS_ERROR("Begin transmission fail!");
      continue;
    }
    for (int i = 0; i < 5; i++){
      if(!client_->write(data[i])) {
        ROS_ERROR("Write data fail");
        continue;
      }
    }
    if (!client_->endTransmission()) {
      ROS_ERROR("Transmission data fail!");
      continue;
    }
    //Send T3 time
    uint64_t T3 = ros::Time::now().toNSec();
    ROS_INFO("Send T3 time: %lu", T3);
    data[0] = 2;
    data[1] = (uint16_t)((T3 & 0xFFFF000000000000) >> 48);
    data[2] = (uint16_t)((T3 & 0xFFFF00000000) >> 32);
    data[3] = (uint16_t)((T3 & 0xFFFF0000) >> 16);
    data[4] = (uint16_t)(T3 & 0xFFFF);
    if(!client_->beginTransmission(slave_id_, HOLDING_REGISTERS, startHoldingRegsAddr_ + 15, 5)) {
      ROS_ERROR("Begin transmission fail!");
      continue;
    }
    for (int i = 0; i < 5; i++){
      if(!client_->write(data[i])) {
        ROS_ERROR("Write data fail");
        continue;
      }
    }
    if (!client_->endTransmission()) {
      ROS_ERROR("Transmission data fail!");
      continue;
    }
    ROS_INFO("Finish time sync");
    break;
  }
}

void TvcLocClient::publishTF(tvcloc_client::LocResult pose)
{
  static tf::TransformBroadcaster br;
  tf::StampedTransform tf1, tf2, tf3;
  try{
    tf_listener_.lookupTransform(pose_frame, pub_frame, ros::Time(0), tf1);
  } catch (tf::TransformException ex) {
    ROS_WARN("%s",ex.what());
    return;
  }
  tf2.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  tf2.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
  tf3.mult(tf2, tf1);
  tf3.frame_id_ = map_frame;
  tf3.child_frame_id_ = pub_frame;
  pose.header.stamp + ros::Duration(0.1);
  br.sendTransform(tf3);
}

uint64_t TvcLocClient::combine2uint64(uint16_t bytes[4])
{
  uint32_t tmp1 = ((uint32_t)bytes[0] << 16) | bytes[1];
  uint32_t tmp2 = ((uint32_t)bytes[2] << 16) | bytes[3];
  return ((uint64_t)tmp1 << 32) | tmp2;
}

uint16_t* TvcLocClient::splitUint64(uint64_t u)
{
  uint16_t* out = new uint16_t[4];
  out[0] = (uint16_t)((u & 0xFFFF000000000000) >> 48);
  out[1] = (uint16_t)((u & 0xFFFF00000000) >> 32);
  out[2] = (uint16_t)((u & 0xFFFF0000) >> 16);
  out[3] = (uint16_t)(u & 0xFFFF);
  return out;
}

int32_t TvcLocClient::combine2int32(uint16_t bytes[2])
{
  return (uint32_t)bytes[0] << 16 | bytes[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tvcloc_client");
  ROS_INFO("Init tvcloc_client node");
  TvcLocClient tvc_client;
  return 0;
}
