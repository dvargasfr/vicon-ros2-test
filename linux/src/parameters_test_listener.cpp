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
          
        // RCLCPP_WARN(this->get_logger(), "Latency: %f seconds", diff_s);
        samples_++;
        latency_sum_ += diff_s;
        med_ = latency_sum_ / samples_;
        RCLCPP_WARN(this->get_logger(), "latencia med: %f seconds", med_);
        RCLCPP_WARN(this->get_logger(), "frame_number: %d", msg->frame_number);
        RCLCPP_WARN(this->get_logger(), "tasa de perdida: %f", (samples_/msg->frame_number)*100.0);
      };
    test_sub_ = create_subscription<mocap4ros_msgs::msg::Markers>("/test/markers", 10, callback);
  }

private:
  rclcpp::Subscription<mocap4ros_msgs::msg::Markers>::SharedPtr test_sub_;
  float med_ = 0;
  float latency_sum_ = 0;
  int samples_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node_listener = std::make_shared<TestParamsListener>();
  rclcpp::spin(node_listener->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}