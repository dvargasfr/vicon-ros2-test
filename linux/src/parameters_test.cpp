// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>

#include <string>
#include <vector>
#include <memory>
#include "parameters_test/parameters_test.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using std::min;
using std::max;
using std::string;
using std::stringstream;

ParametersTest::ParametersTest(const rclcpp::NodeOptions node_options)
  : rclcpp_lifecycle::LifecycleNode("param_test_node", node_options)
{

}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ParametersTest::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /*
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN [%d]", lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED [%d]", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE [%d]", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE [%d]", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED [%d]", lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING);
  RCLCPP_INFO(get_logger(), "lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING [%d]", lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING);
  */

  /*...*/
  marker_pub_ = create_publisher<mocap4ros_msgs::msg::Markers>("/test/markers", 100);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/vicon2_driver/change_state");
  update_pub_ = create_publisher<std_msgs::msg::Empty>("/vicon2_driver/update_notify",
    rclcpp::QoS(100));

  initParameters();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParametersTest::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  marker_pub_->on_activate();
  startPublishing();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParametersTest::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParametersTest::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParametersTest::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParametersTest::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  RCLCPP_ERROR(get_logger(), "Error transition!\n");

  return CallbackReturnT::SUCCESS;
}

void ParametersTest::initParameters() {

  bool ust_param;

  declare_parameter<std::string>("test_param", "...");

  get_parameter<std::string>("test_param", myParam);
  get_parameter<bool>("use_sim_time", ust_param);

  RCLCPP_WARN(get_logger(), "Param test_param: %s", myParam.c_str());
  RCLCPP_WARN(get_logger(), "Param use_sim_time: %s", ust_param?"true":"false");
}

void ParametersTest::startPublishing() {
  int frame_number = 0;
  RCLCPP_WARN(get_logger(), "INIT startPublishing\n");
  while (rclcpp::ok()) {
    mocap4ros_msgs::msg::Markers markers_msg;
    rclcpp::Time now = this->now();
    // rclcpp::Clock now_clock = this->get_clock()->now();
    markers_msg.header.stamp = now;
    rclcpp::Time now2 = markers_msg.header.stamp;
    // RCLCPP_WARN(get_logger(), "[1] Publishing at %d %u\n", markers_msg.header.stamp.sec, markers_msg.header.stamp.nanosec);
    // RCLCPP_WARN(get_logger(), "[2] Publishing at %d %u\n", (float)now.seconds(), now.nanoseconds());
    // RCLCPP_WARN(get_logger(), "[3] Publishing at %d %u\n", (float)now2.seconds(), now2.nanoseconds());
    markers_msg.frame_number = frame_number++;

    mocap4ros_msgs::msg::Marker this_marker;
    this_marker.translation.x = 1.0;
    this_marker.translation.y = 2.0;
    this_marker.translation.z = 3.0;
    // this_marker.occluded = false;
    markers_msg.markers.push_back(this_marker);

    marker_pub_->publish(markers_msg);
    // clock_time_pub_->publish(now_clock);
  }
  RCLCPP_WARN(get_logger(), "END startPublishing\n");
}