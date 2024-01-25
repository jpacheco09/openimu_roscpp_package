/**
 * @file openimu_attitude_publisher_node.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <bits/stdint-uintn.h>
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <libserial/SerialStream.h>

#define deg_to_rad 0.01745329251

template <typename packageType>
void unpackU4(uint8_t* idx, const void* buffer, packageType& package)
{
  const uint8_t* temp_buffer = (const uint8_t*)buffer;
  // ROS_INFO("| 0x%02X | 0x%02X | 0x%02X | 0x%02X |\n", temp_buffer[3], temp_buffer[2], temp_buffer[1], temp_buffer[0]);
  uint32_t u4_buffer = 0;
  *idx += 4;
  u4_buffer = ((temp_buffer[3] << 24) | (temp_buffer[2] << 16) | (temp_buffer[1] << 8) | (temp_buffer[0]));
  std::memcpy(&package, &u4_buffer, sizeof(packageType));
  // package = *((packageType*)&u4_buffer);
}

int main(int argc, char** argv)
{
  // init ros node
  ros::init(argc, argv, "openimu_attitude_publisher");
  ros::NodeHandle nh;
  // init publisher at 10hz rate
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("open_imu/data", 200);
  ros::Rate loop_rate(200);
  // open serial stream at ttyUSB0, 115200bps, 8bits, 1 stop bit, no parity and no flow control
  const std::string device_route("/dev/ttyUSB0");
  LibSerial::SerialStream serial_stream;
  serial_stream.Open(device_route);
  serial_stream.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  serial_stream.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_stream.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serial_stream.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

  if (serial_stream.IsOpen())
  {
    ROS_INFO("Succesfully opened %s", device_route.c_str());
  }
  else
  {
    ROS_ERROR("Error opening %s", device_route.c_str());
    return EXIT_FAILURE;
  }

  // openimu a2 package definitions
  const uint8_t buffer_size = 55;
  char rx_buffer[buffer_size];
  memset(rx_buffer, 0, sizeof(rx_buffer));

  uint32_t time_ms = 0;
  uint8_t buffer_idx = 0;
  float attitude_data[3], linear_rate[3], linear_accel[3];
  uint32_t packet_header = 0;
  std_msgs::String msg;
  msg.data = "A2 fetched!";

  // Imu msg
  sensor_msgs::Imu openimu_data;
  tf2::Quaternion openimu_attitude;
  ros::Time time_stamp;
  const std::string frame_id("open_imu_link"), tf_parent_frame_id("world"), tf_frame_id("open_imu_link");

  openimu_data.header.frame_id = frame_id;

  static tf::TransformBroadcaster tf_br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion open_imu_attitude_quaternion;

  while (ros::ok())
  {
    /*
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    imu_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    */
    if (serial_stream.IsDataAvailable())  // ask for input packages
    {                                     // read package
      serial_stream.read(rx_buffer, buffer_size);
      unpackU4<uint32_t>(&buffer_idx, &rx_buffer[buffer_idx], packet_header);
      // ROS_INFO("H: %d \t 0x%X", packet_header, packet_header);
      // header and pack type are sent in little endian fashion, however, attitude is sent in big endian, hence, the
      // below if should be read as 0x55556132 according to the openimu datasheet
      if (packet_header == 0x32615555)  // fetch openIMU a2 message as big endian
      {
        buffer_idx++;  // skip packet length
        // unpack time_ms
        unpackU4<uint32_t>(&(buffer_idx), &rx_buffer[buffer_idx], time_ms);
        buffer_idx += 8;  // skip time_s
        // unpack RP"Y" attitude info
        for (float& ptr : attitude_data)
        {
          unpackU4<float>(&buffer_idx, &rx_buffer[buffer_idx], ptr);
          ptr *= deg_to_rad;
        }
        // unpack linear rate info
        for (float& ptr : linear_rate)
          unpackU4<float>(&buffer_idx, &rx_buffer[buffer_idx], ptr);
        // unpack linear accel info
        for (float& ptr : linear_accel)
          unpackU4<float>(&buffer_idx, &rx_buffer[buffer_idx], ptr);
        // debug print to show attitude info
//        ROS_INFO("R: %f | P: %f | H: %f ", attitude_data[0], attitude_data[1], attitude_data[2]);
        openimu_attitude.setRPY(attitude_data[0], attitude_data[1], attitude_data[2]);
        openimu_attitude.normalize();
        time_stamp.fromSec(static_cast<double>(time_ms * 1e-3));
        openimu_data.header.stamp = time_stamp;
        //
        open_imu_attitude_quaternion.setRPY(attitude_data[1], attitude_data[0], attitude_data[2]);
        tf::quaternionTFToMsg(open_imu_attitude_quaternion, openimu_data.orientation);
        openimu_data.linear_acceleration.x = linear_accel[0];
        openimu_data.linear_acceleration.y = linear_accel[1];
        openimu_data.linear_acceleration.z = linear_accel[2];
        // reset buffer_idx
        buffer_idx = 0;
        serial_stream.FlushInputBuffer();
        imu_pub.publish(openimu_data);

        // tf broadcast

        transform.setRotation(open_imu_attitude_quaternion);
        tf_br.sendTransform(tf::StampedTransform(transform, time_stamp, tf_parent_frame_id, tf_frame_id));
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  serial_stream.Close();
  return 0;
}
