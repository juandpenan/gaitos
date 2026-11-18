//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_COMMON__PARAMS_HPP_
#define GAITOS_COMMON__PARAMS_HPP_

#include <Eigen/Dense>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter_value.hpp"

namespace gaitos
{

using ParametersInterface = rclcpp::node_interfaces::NodeParametersInterface::SharedPtr;

/**
 * @class ParamsError
 * @brief Missing or malformed parameter.
 */
class ParamsError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/**
 * @class Params
 * @brief View of the ROS 2 parameters of a node under a prefix ("controllers.gait"...).
 *
 * The node is created with automatically_declare_parameters_from_overrides, so every value
 * of the parameter files exists; get() declares the missing ones with their default value.
 * Integers are accepted where doubles are expected. Keys may be nested with dots.
 */
class Params
{
public:
  Params() = default;
  Params(ParametersInterface params, std::string prefix = "")
  : params_(std::move(params)), prefix_(std::move(prefix)) {}

  /// The block has any parameter
  bool valid() const;

  /// A parameter or a block with that key exists
  bool has(const std::string & key) const;

  /**
   * @brief Value of a key, or default_value if it is not set (then declared with it)
   * @throws ParamsError if it is set with an incompatible type
   */
  template<typename T>
  T get(const std::string & key, const T & default_value) const;

  /**
   * @throws ParamsError if it is not set or has a wrong type
   */
  template<typename T>
  T require(const std::string & key) const
  {
    if (!has_value(key)) {
      throw ParamsError("Missing parameter " + full_key(key));
    }
    return get<T>(key, T{});
  }

  /// Whether the key is a string (not a list), e.g. "joints: all"
  bool is_string(const std::string & key) const;

  /// 3 element vector ([x, y, z]); a scalar is repeated
  Eigen::Vector3d get_vector3(const std::string & key, const Eigen::Vector3d & default_value) const;

  /// Per name doubles of a block ({name: value, ...})
  std::map<std::string, double> get_map(const std::string & key) const;

  /// Sub block
  Params sub(const std::string & key) const;

  /// Names of the direct children of the block
  std::vector<std::string> keys() const;

  std::string full_key(const std::string & key) const
  {
    return prefix_.empty() ? key : prefix_ + "." + key;
  }
  const std::string & prefix() const {return prefix_;}
  const ParametersInterface & interface() const {return params_;}

private:
  bool has_value(const std::string & key) const;
  rclcpp::ParameterValue value(const std::string & key, const rclcpp::ParameterValue & def) const;

  ParametersInterface params_;
  std::string prefix_;
};

namespace detail
{
template<typename T>
T convert(const rclcpp::ParameterValue & v, const std::string & name);
}  // namespace detail

template<typename T>
T Params::get(const std::string & key, const T & default_value) const
{
  if (!params_) {
    return default_value;
  }
  return detail::convert<T>(value(key, rclcpp::ParameterValue(default_value)), full_key(key));
}

/**
 * @brief Read a file (URDF, SRDF...) or resolve a package:// URI to a path
 */
std::string resolve_uri(const std::string & uri);
std::string read_file(const std::string & path);

}  // namespace gaitos

#endif  // GAITOS_COMMON__PARAMS_HPP_
