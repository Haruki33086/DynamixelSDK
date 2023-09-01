// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node2
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>
#include <chrono>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/get_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node2.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_LED 65
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
// #define BAUDRATE 1000000 // lab dynamixel
#define DEVICE_NAME "/dev/ttyACM0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"
// #define DEVICE_NAME "/dev/ttyUSB0" // lab dynamixel

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

ReadWriteNode2::ReadWriteNode2()
: Node("read_write_node2")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node2");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
  
  set_velocity_subscriber_ =
    this->create_subscription<SetVelocity>(
    "set_velocity",
    QOS_RKL10V,
    [this](const SetVelocity::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      int32_t goal_velocity = (int32_t)msg->velocity;  // Convert int32 -> uint32

      // Write Goal Velocity (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_VELOCITY,
        goal_velocity,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", msg->id, msg->velocity);
      }
    }
    );

  cmd_vel_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    QOS_RKL10V,
    [this](const geometry_msgs::msg::Twist::SharedPtr twist) -> void
    {
      uint8_t dxl_error = 0;
      // motor id // cmd_velだけじゃモーターのidがわからないので、ここで指定する
      uint8_t left_motor_id = 0; // モーターのidを指定
      uint8_t right_motor_id = 1; // モーターのidを指定

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.

      int32_t left_goal_velocity;
      int32_t right_goal_velocity;

      if ((twist->linear.x > 0) || (twist->linear.x < 0)) {
        left_goal_velocity = static_cast<int32_t>(twist->linear.x * 100);  // Convert int32 -> uint32
        right_goal_velocity = static_cast<int32_t>(twist->linear.x * 100);  // Convert int32 -> uint32
      } else if ((twist->angular.z > 0) || (twist->angular.z < 0)) {
        left_goal_velocity = static_cast<int32_t>(-twist->angular.z * 50);  // Convert int32 -> uint32
        right_goal_velocity = static_cast<int32_t>(twist->angular.z * 50);  // Convert int32 -> uint32
      } else {
        left_goal_velocity = 0.0;
        right_goal_velocity = 0.0;
      }

      // Write Goal Velocity (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) left_motor_id,
        ADDR_GOAL_VELOCITY,
        left_goal_velocity,
        &dxl_error
      );

      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) right_motor_id,
        ADDR_GOAL_VELOCITY,
        -right_goal_velocity,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", left_motor_id, left_goal_velocity);
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", right_motor_id, right_goal_velocity);
      }
    }
    );

  get_velocity_publisher_ =
    this->create_publisher<GetVelocity>(
    "get_velocity",
    QOS_RKL10V
    );
  timer_ = this->create_wall_timer(std::chrono::milliseconds{100}, std::bind(&ReadWriteNode2::timer_callback, this));
}

void ReadWriteNode2::timer_callback() // このように書かないといけないし、hppの方にも記述（private）←publicかどっちでもいいかも
{
  uint8_t dxl_error = 0;
    uint8_t left_motor_id = 0; // モーターのidを指定
    uint8_t right_motor_id = 1; // モーターのidを指定

    // Read Present Velocity (length : 4 bytes)
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    int32_t left_present_velocity = 0;
    int32_t right_present_velocity = 0;
    dxl_comm_result =
    packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) left_motor_id,
      ADDR_PRESENT_VELOCITY,
      (uint32_t*)&left_present_velocity, // decimal値
      &dxl_error
    );

    dxl_comm_result =
    packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) right_motor_id,
      ADDR_PRESENT_VELOCITY,
      (uint32_t*)&right_present_velocity,
      &dxl_error
    );

    auto msg = dynamixel_sdk_custom_interfaces::msg::GetVelocity();
    msg.left_vel = left_present_velocity * 0.229; // 1decimal = around 0.229rpm
    msg.right_vel = -right_present_velocity * 0.229; // 右モーターは逆回転なので、-をつける
    get_velocity_publisher_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Left Present Velocity: %d", left_present_velocity);
}

ReadWriteNode2::~ReadWriteNode2()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Velocity Control Mode おそらくpositionかvelocityのどちらかしか選択できない
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node2"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node2"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node2"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node2"), "Succeeded to enable torque.");
  }

  // Set LED of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_LED,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node2"), "Failed to set LED.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node2"), "Succeeded to set LED.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node2"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node2"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node2"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node2"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode2 = std::make_shared<ReadWriteNode2>();
  rclcpp::spin(readwritenode2);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  // Reset LED of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_LED,
    0,
    &dxl_error
  );

  return 0;
}