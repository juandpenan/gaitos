//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_common/Params.hpp"

#include <fstream>
#include <set>
#include <sstream>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace gaitos
{

namespace detail
{
using rclcpp::ParameterType;

[[noreturn]] void wrong(const std::string & name, const rclcpp::ParameterValue & v, const char * expected)
{
  throw ParamsError("Parameter " + name + ": expected " + expected + ", got " +
          rclcpp::to_string(v.get_type()));
}

template<>
double convert<double>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_DOUBLE) {return v.get<double>();}
  if (v.get_type() == ParameterType::PARAMETER_INTEGER) {return static_cast<double>(v.get<int64_t>());}
  wrong(name, v, "a number");
}

template<>
int convert<int>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_INTEGER) {return static_cast<int>(v.get<int64_t>());}
  wrong(name, v, "an integer");
}

template<>
bool convert<bool>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_BOOL) {return v.get<bool>();}
  wrong(name, v, "a boolean");
}

template<>
std::string convert<std::string>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_STRING) {return v.get<std::string>();}
  wrong(name, v, "a string");
}

template<>
std::vector<double> convert<std::vector<double>>(const rclcpp::ParameterValue & v, const std::string & name)
{
  switch (v.get_type()) {
    case ParameterType::PARAMETER_DOUBLE_ARRAY: return v.get<std::vector<double>>();
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
        const auto i = v.get<std::vector<int64_t>>();
        return {i.begin(), i.end()};
      }
    case ParameterType::PARAMETER_DOUBLE: return {v.get<double>()};
    case ParameterType::PARAMETER_INTEGER: return {static_cast<double>(v.get<int64_t>())};
    case ParameterType::PARAMETER_NOT_SET: return {};
    default: wrong(name, v, "a list of numbers");
  }
}

template<>
std::vector<bool> convert<std::vector<bool>>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_BOOL_ARRAY) {return v.get<std::vector<bool>>();}
  if (v.get_type() == ParameterType::PARAMETER_NOT_SET) {return {};}
  wrong(name, v, "a list of booleans");
}

template<>
std::vector<std::string> convert<std::vector<std::string>>(const rclcpp::ParameterValue & v, const std::string & name)
{
  if (v.get_type() == ParameterType::PARAMETER_STRING_ARRAY) {return v.get<std::vector<std::string>>();}
  if (v.get_type() == ParameterType::PARAMETER_STRING) {return {v.get<std::string>()};}
  if (v.get_type() == ParameterType::PARAMETER_NOT_SET) {return {};}
  wrong(name, v, "a list of strings");
}
}  // namespace detail

bool Params::has_value(const std::string & key) const
{
  return params_ && params_->has_parameter(full_key(key)) &&
         params_->get_parameter(full_key(key)).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

bool Params::has(const std::string & key) const
{
  if (!params_) {
    return false;
  }
  if (has_value(key)) {
    return true;
  }
  const auto r = params_->list_parameters({full_key(key)}, 0);
  return !r.names.empty() || !r.prefixes.empty();
}

bool Params::valid() const
{
  if (!params_) {
    return false;
  }
  if (prefix_.empty()) {
    return true;
  }
  const auto r = params_->list_parameters({prefix_}, 0);
  return !r.names.empty();
}

bool Params::is_string(const std::string & key) const
{
  return has_value(key) &&
         params_->get_parameter(full_key(key)).get_type() == rclcpp::ParameterType::PARAMETER_STRING;
}

rclcpp::ParameterValue Params::value(const std::string & key, const rclcpp::ParameterValue & def) const
{
  const auto name = full_key(key);
  if (params_->has_parameter(name)) {
    const auto v = params_->get_parameter(name).get_parameter_value();
    if (v.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return v;
    }
    return def;
  }
  rcl_interfaces::msg::ParameterDescriptor d;
  d.dynamic_typing = true;
  if (def.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return def;
  }
  params_->declare_parameter(name, def, d, false);
  return def;
}

Eigen::Vector3d Params::get_vector3(const std::string & key, const Eigen::Vector3d & default_value) const
{
  const auto v = get<std::vector<double>>(key, {default_value.x(), default_value.y(), default_value.z()});
  if (v.size() == 1) {
    return Eigen::Vector3d::Constant(v[0]);
  }
  if (v.size() != 3) {
    throw ParamsError("Parameter " + full_key(key) + " must have 3 elements");
  }
  return {v[0], v[1], v[2]};
}

std::map<std::string, double> Params::get_map(const std::string & key) const
{
  std::map<std::string, double> out;
  const Params block = sub(key);
  for (const auto & k : block.keys()) {
    out[k] = block.get<double>(k, 0.0);
  }
  return out;
}

Params Params::sub(const std::string & key) const
{
  return Params(params_, full_key(key));
}

std::vector<std::string> Params::keys() const
{
  std::vector<std::string> out;
  if (!params_) {
    return out;
  }
  const auto r = params_->list_parameters(prefix_.empty() ? std::vector<std::string>{} :
      std::vector<std::string>{prefix_}, 0);
  const std::string start = prefix_.empty() ? "" : prefix_ + ".";
  std::set<std::string> seen;
  for (const auto & name : r.names) {
    if (name.rfind(start, 0) != 0 || name.size() <= start.size()) {
      continue;
    }
    const auto rest = name.substr(start.size());
    const auto child = rest.substr(0, rest.find('.'));
    if (seen.insert(child).second) {
      out.push_back(child);
    }
  }
  return out;
}

std::string resolve_uri(const std::string & uri)
{
  const std::string scheme = "package://";
  if (uri.rfind(scheme, 0) != 0) {
    return uri;
  }
  const auto rest = uri.substr(scheme.size());
  const auto slash = rest.find('/');
  const auto package = rest.substr(0, slash);
  try {
    return ament_index_cpp::get_package_share_path(package).string() +
           (slash == std::string::npos ? "" : rest.substr(slash));
  } catch (const std::exception & e) {
    throw ParamsError("Cannot resolve " + uri + ": " + e.what());
  }
}

std::string read_file(const std::string & path)
{
  std::ifstream f(resolve_uri(path));
  if (!f) {
    throw ParamsError("Cannot read " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace gaitos
