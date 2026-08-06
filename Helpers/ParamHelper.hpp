#pragma once
#include <string>
#include <vector>
#include <type_traits>
#include <cstdint>

#ifdef ROS1
  #include <ros/ros.h>
#else
  #include <rclcpp/rclcpp.hpp>
#endif

// Minimal ROS1 / ROS2 parameter wrapper.
// - Flat parameters only (no nested struct support)
// - Read once at startup, no dynamic reconfigure / runtime watch
// - Same getParam<T>() call works for scalars (bool/int/double/string)
//   and arrays (std::vector<bool/int/double/string>) on both ROS versions.
class ParamHelper {
public:
#ifdef ROS1
  explicit ParamHelper(ros::NodeHandle nh) : nh_(nh) {}

  template <typename T>
  T getParam(const std::string& name, const T& default_value) {
    T value;
    nh_.param(name, value, default_value);
    return value;
  }

private:
  ros::NodeHandle nh_;

#else
  explicit ParamHelper(rclcpp::Node::SharedPtr node) : node_(node) {}

  template <typename T>
  T getParam(const std::string& name, const T& default_value) {
    // rclcpp stores integer *array* parameters internally as std::vector<int64_t>,
    // regardless of what element type you ask for. Asking for std::vector<int>
    // directly throws a bad_conversion error, so we go through int64_t and
    // convert back for that one case.
    if constexpr (std::is_same<T, std::vector<int>>::value) {
      std::vector<int64_t> default_i64(default_value.begin(), default_value.end());
      if (!node_->has_parameter(name)) {
        node_->declare_parameter(name, default_i64);
      }
      std::vector<int64_t> value_i64;
      node_->get_parameter(name, value_i64);
      return std::vector<int>(value_i64.begin(), value_i64.end());
    } else {
      // Guard against declaring the same parameter twice (throws in ROS2).
      if (!node_->has_parameter(name)) {
        node_->declare_parameter(name, default_value);
      }
      T value;
      node_->get_parameter(name, value);
      return value;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
#endif
};