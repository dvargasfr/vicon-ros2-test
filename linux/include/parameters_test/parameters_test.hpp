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

#ifndef PARAMETERS_TEST__PARAMETERS_TEST_HPP_
#define PARAMETERS_TEST__PARAMETERS_TEST_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <chrono>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/node_options.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "mocap4ros_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/markers.hpp"
#include "mocap4ros_msgs/srv/vicon_calib_seg.hpp"
#include "mocap4ros_msgs/srv/vicon_grab_pose.hpp"

class ParametersTest : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ParametersTest(const rclcpp::NodeOptions options = 
    rclcpp::NodeOptions().parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("use_sim_time", true)
      }
    )
  );

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
  void initParameters();
  void startPublishing();
private:
  std::string myParam;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4ros_msgs::msg::Markers>::SharedPtr marker_pub_;
  rclcpp::Time now_time;
  std::string stream_mode_;
  std::string host_name_;
  bool publish_markers_;
  bool marker_data_enabled;
  bool unlabeled_marker_data_enabled;
  unsigned int lastFrameNumber;
  unsigned int frameCount;
  unsigned int droppedFrameCount;
  unsigned int n_markers;
  unsigned int n_unlabeled_markers;

};
#endif  // PARAMETERS_TEST__PARAMETERS_TEST_HPP_
