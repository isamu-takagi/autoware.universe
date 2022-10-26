// Copyright 2022 Takagi, Isamu
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

#ifndef RCLCPPEX__TOPIC_RATE_MONITOR_HPP_
#define RCLCPPEX__TOPIC_RATE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <utility>

namespace rclcppex
{

struct TopicRateConfig
{
  double window_time = 5.0;
};

class TopicRateMonitor
{
public:
  explicit TopicRateMonitor(rclcpp::Clock::SharedPtr clock) { clock_ = std::move(clock); }

  double get_rate()
  {
    update_queue();
    return queue_.size() / config_.window_time;
  }

protected:
  void insert_queue()
  {
    queue_.push_back(clock_->now());
    update_queue();
  }

  void update_queue()
  {
    const auto timeout = clock_->now() - rclcpp::Duration::from_seconds(config_.window_time);
    while (!queue_.empty()) {
      if (timeout < queue_.front()) {
        break;
      }
      queue_.pop_front();
    }
  }

private:
  const TopicRateConfig config_;
  rclcpp::Clock::SharedPtr clock_;
  std::deque<rclcpp::Time> queue_;
};

}  // namespace rclcppex

#endif  // RCLCPPEX__TOPIC_RATE_MONITOR_HPP_
