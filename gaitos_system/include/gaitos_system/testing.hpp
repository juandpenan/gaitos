//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// Test helpers (gtest only): parameter files of a robot and backend, overrides.

#ifndef GAITOS_SYSTEM__TESTING_HPP_
#define GAITOS_SYSTEM__TESTING_HPP_

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gaitos_common/testing.hpp"

namespace gaitos
{
namespace testutil
{

/**
 * @brief Parameter files: gaitos/params/gaitos.yaml, the robot file and the backend files
 * @param source gaitos source folder
 * @param robot go2, r1, nao
 * @param backend files relative to the source folder (e.g. "gaitos/params/go2_fake.yaml")
 */
inline std::vector<std::string> robot_files(
  const std::string & source, const std::string & robot, const std::vector<std::string> & backend)
{
  std::vector<std::string> f{source + "/gaitos/params/gaitos.yaml",
    source + "/gaitos_description/robots/" + robot + "_description/config/" + robot + ".yaml"};
  for (const auto & b : backend) {
    f.push_back(source + "/" + b);
  }
  return f;
}

/**
 * @brief A parameter file with these values (YAML of the ros__parameters block)
 */
inline std::string overrides(const std::string & yaml)
{
  static int count = 0;
  const auto path = std::filesystem::temp_directory_path() /
    ("gaitos_test_overrides_" + std::to_string(::getpid()) + "_" + std::to_string(count++) + ".yaml");
  std::ofstream out(path);
  out << "/**:\n  ros__parameters:\n";
  std::istringstream in(yaml);
  std::string line;
  while (std::getline(in, line)) {
    out << "    " << line << "\n";
  }
  return path.string();
}

}  // namespace testutil
}  // namespace gaitos

#endif  // GAITOS_SYSTEM__TESTING_HPP_
