#ifndef _TVCLOC_CLIENT_H__
#define _TVCLOC_CLIENT_H__

#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "tvcloc_client/LocResult.h"
#include "tvcloc_client/Encoder.h"

#include "modbus.h"
#include "ModbusClient.h"
#include "ModbusRTUClient.h"
#include "ModbusTCPClient.h"
#include "tvcloc_client/localization.h"

constexpr int NUM_INPUT_REGS = 12;
constexpr int NUM_HOLDING_REGS_ODOM15 = 13;
constexpr int NUM_HOLDING_REGS_ODOM14 = 10;
constexpr int NUM_HOLDING_REGS_ENC = 15;
constexpr int NUM_HOLDING_REGS_CMD = 8;

class TvcLocClient {
 public:
  TvcLocClient();
  ~TvcLocClient();

  uint64_t combine2uint64(uint16_t bytes[4]);
  uint16_t* splitUint64(uint64_t u);
  int32_t combine2int32(uint16_t bytes[2]);

 private:
  bool readConfigs();
  bool createConnection(bool tcp_mode);
  bool retryConnect();
  int connectTCP();
  int connectRTU();
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void encoderCallback(const tvcloc_client::Encoder::ConstPtr &msg);
  void timeSync();
  void publishTF(tvcloc_client::LocResult pose);
  bool triggerLocalization(tvcloc_client::localizationRequest &req,
    tvcloc_client::localizationResponse &res);

 private:
  ros::NodeHandle node_;
  ros::NodeHandle priv_node_;
  ros::Publisher loc_out_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber enc_sub_;
  ros::ServiceServer localization_service_;

  std::string ip_address_;
  int port_;
  int rate_;
  bool tcp_;
  int slave_id_;
  int startInputRegsAddr_;
  int startHoldingRegsAddr_;
  std::vector<int> regs_addr_;
  std::vector<uint16_t> inputDataArr_;
  int lastestModbusCnt_ = -1;
  uint16_t odom_counter_ = 0;
  int msg_type_;
  int msg_type_version_;
  std::string map_frame;
  std::string pose_frame;
  std::string pub_frame;

  int retry_cnt_ = 0;
  int num_retry_;

  std::string odom_topic_;
  std::string encoder_topic_;

  ModbusClient* client_;
  std::mutex mutex_;
  bool terminated_is_requested_;
  tf::TransformListener tf_listener_;
};

#endif // _TVCLOC_CLIENT_H__