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


class TestParamsListener : public rclcpp::Node
{
public:
  explicit TestParamsListener(const rclcpp::NodeOptions options = 
    rclcpp::NodeOptions().parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("use_sim_time", true)
      }
    )
  ) : Node("listener", options)
  {
      auto callback =
      [this](const mocap4ros_msgs::msg::Markers::SharedPtr msg) -> void
      {
        rclcpp::Time now = this->now();
        rclcpp::Time msg_time = msg->header.stamp;
        // RCLCPP_INFO(this->get_logger(), "now: %f . %u", now.seconds(), now.nanoseconds());
        // RCLCPP_INFO(this->get_logger(), "now_msg: %f . %u", msg_time.seconds(), msg_time.nanoseconds());
        
        auto diff_s = now.seconds() - msg_time.seconds();
          
        // RCLCPP_WARN(this->get_logger(), "now - msg: %f - %f = %f seconds", now.nanoseconds(), msg_time.nanoseconds(), diff_s);
        if (!initiated){
          last_frame_ = msg->frame_number;
          initiated = true;
        }
        samples_++;
        latency_sum_ += diff_s;
        med_ = latency_sum_ / samples_;
        if (last_frame_ + 1 != msg->frame_number && last_frame_ != msg->frame_number){
          pkgs_lost_+= (msg->frame_number-last_frame_);
          RCLCPP_INFO(this->get_logger(), "PKG LOST!! last: %d  ---  current: %d", last_frame_, msg->frame_number);
          RCLCPP_INFO(this->get_logger(), "total lost: %d --- total received %d", pkgs_lost_, samples_);
          RCLCPP_WARN(this->get_logger(), "PKG LOST!! last: %d  ---  current: %d", last_frame_, msg->frame_number);
          RCLCPP_WARN(this->get_logger(), "total lost: %d --- total received %d", pkgs_lost_, samples_);
        }
        last_frame_ = msg->frame_number;
        // RCLCPP_WARN(this->get_logger(), "latencia med: %f seconds", med_);
        // RCLCPP_WARN(this->get_logger(), "frame_number: %d", msg->frame_number);
        // RCLCPP_WARN(this->get_logger(), "tasa de perdida: %d / %d = %f", pkgs_lost_, samples_, (float)((float)pkgs_lost_/(float)samples_)*100.0);
        // RCLCPP_WARN(this->get_logger(), "paquetes perdidos: %d", pkgs_lost_);
      };
    test_sub_ = create_subscription<mocap4ros_msgs::msg::Markers>("/vicon/markers", 1000, callback);
  }

  float med_ = 0;
  float latency_sum_ = 0;
  int samples_ = 0;
  uint last_frame_ = 0;
  int pkgs_lost_ = 0;
  bool initiated = false;
private:
  rclcpp::Subscription<mocap4ros_msgs::msg::Markers>::SharedPtr test_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node_listener = std::make_shared<TestParamsListener>();
  rclcpp::spin(node_listener->get_node_base_interface());
  RCLCPP_INFO(node_listener->get_logger(), "total lost: %d --- total received %d", node_listener->pkgs_lost_, node_listener->samples_);
  rclcpp::shutdown();

  return 0;
}