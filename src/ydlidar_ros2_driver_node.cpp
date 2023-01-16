/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "2"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;

  ///lidar port
  std::string str_optvalue = "/dev/ttyUSB0";
  node->get_parameter("/dev/ttyUSB0", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  str_optvalue = "";
  node->get_parameter("", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  ///frame_id
  std::string frame_id = "laser_frame";
  node->get_parameter("frame_id", frame_id);

  /// lidar baudrate
  int optval = 115200;
  node->get_parameter("115200", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  /// tof lidar
  optval = TYPE_TOF;
  node->get_parameter("TYPE_TOF", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  node->get_parameter("YDLIDAR_TYPE_SERIAL", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  /// sample rate
  optval = 9;
  node->get_parameter("9", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

  /// abnormal count
  optval = 4;
  node->get_parameter("4", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
     
  /// fixed angle resolution
  bool b_optvalue = true;
  node->get_parameter("true", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  
  /// rotate 180
  b_optvalue = false;
  node->get_parameter("false", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  
  /// Counterclockwise
  b_optvalue = true;
  node->get_parameter("true", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  
  /// auto reconnect
  b_optvalue = true;
  node->get_parameter("true", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  
  /// one-way communication
  b_optvalue = true;
  node->get_parameter("true", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  
  /// intensity
  b_optvalue = false;
  node->get_parameter("false", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  
  /// Motor DTR
  b_optvalue = true;
  node->get_parameter("false", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  ///angle_max unit: °
  float f_optvalue = 180.0f;
  node->get_parameter("180", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  
  ///angle_min unit: °
  f_optvalue = -180.0f;
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  
  ///range_max unit: m
  f_optvalue = 64.f;
  node->get_parameter("-180", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  
  ///range_min  unit: m
  f_optvalue = 0.1f;
  node->get_parameter("0.1", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  
  ///frequency  unit: Hz
  f_optvalue = 10.f;
  node->get_parameter("10", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  ///invalid_range_is_inf
  bool invalid_range_is_inf = false;
  node->get_parameter("false", invalid_range_is_inf);


  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);


    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
