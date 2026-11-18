//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <tinyxml2.h>

#include <sstream>
#include <stdexcept>
#include <string>

#include "gaitos_common/Math.hpp"
#include "gaitos_model/RobotModel.hpp"

namespace gaitos
{

namespace
{
Eigen::Vector3d parse_vector(const char * text, const Eigen::Vector3d & default_value)
{
  if (!text) {
    return default_value;
  }
  std::stringstream ss(text);
  Eigen::Vector3d v;
  if (!(ss >> v.x() >> v.y() >> v.z())) {
    throw std::runtime_error(std::string("Malformed vector '") + text + "'");
  }
  return v;
}

Eigen::Isometry3d parse_origin(const tinyxml2::XMLElement * parent)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  const auto * origin = parent ? parent->FirstChildElement("origin") : nullptr;
  if (origin) {
    t.translation() = parse_vector(origin->Attribute("xyz"), Eigen::Vector3d::Zero());
    t.linear() = rpy_to_matrix(parse_vector(origin->Attribute("rpy"), Eigen::Vector3d::Zero()));
  }
  return t;
}

double attr(const tinyxml2::XMLElement * e, const char * name, double default_value)
{
  double v = default_value;
  if (e) {
    e->QueryDoubleAttribute(name, &v);
  }
  return v;
}

KinematicTree parse(tinyxml2::XMLDocument & doc)
{
  const auto * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("URDF without <robot> element");
  }
  KinematicTree tree;
  for (const auto * l = robot->FirstChildElement("link"); l; l = l->NextSiblingElement("link")) {
    TreeLink link;
    link.name = l->Attribute("name") ? l->Attribute("name") : "";
    if (const auto * inertial = l->FirstChildElement("inertial")) {
      const Eigen::Isometry3d origin = parse_origin(inertial);
      link.com = origin.translation();
      link.mass = attr(inertial->FirstChildElement("mass"), "value", 0.0);
      if (const auto * in = inertial->FirstChildElement("inertia")) {
        Eigen::Matrix3d i;
        i << attr(in, "ixx", 0), attr(in, "ixy", 0), attr(in, "ixz", 0),
          attr(in, "ixy", 0), attr(in, "iyy", 0), attr(in, "iyz", 0),
          attr(in, "ixz", 0), attr(in, "iyz", 0), attr(in, "izz", 0);
        link.inertia = origin.linear() * i * origin.linear().transpose();
      }
    }
    tree.add_link(link);
  }
  for (const auto * j = robot->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    TreeJoint joint;
    joint.name = j->Attribute("name") ? j->Attribute("name") : "";
    const std::string type = j->Attribute("type") ? j->Attribute("type") : "";
    if (type == "floating" || type == "planar") {
      continue;   // The floating base is the root of the tree
    }
    if (type == "revolute") {
      joint.type = JointType::REVOLUTE;
    } else if (type == "continuous") {
      joint.type = JointType::CONTINUOUS;
    } else if (type == "prismatic") {
      joint.type = JointType::PRISMATIC;
    } else if (type == "fixed") {
      joint.type = JointType::FIXED;
    } else {
      throw std::runtime_error("Joint " + joint.name + " has unsupported type '" + type + "'");
    }
    const auto * parent = j->FirstChildElement("parent");
    const auto * child = j->FirstChildElement("child");
    if (!parent || !child || !parent->Attribute("link") || !child->Attribute("link")) {
      throw std::runtime_error("Joint " + joint.name + " without parent or child");
    }
    joint.parent = parent->Attribute("link");
    joint.child = child->Attribute("link");
    joint.origin = parse_origin(j);
    if (const auto * axis = j->FirstChildElement("axis")) {
      joint.axis = parse_vector(axis->Attribute("xyz"), Eigen::Vector3d::UnitX()).normalized();
    }
    if (const auto * limit = j->FirstChildElement("limit")) {
      joint.effort = attr(limit, "effort", 0.0);
      joint.velocity = attr(limit, "velocity", 0.0);
      if (joint.type == JointType::REVOLUTE || joint.type == JointType::PRISMATIC) {
        joint.lower = attr(limit, "lower", joint.lower);
        joint.upper = attr(limit, "upper", joint.upper);
      }
    }
    if (const auto * mimic = j->FirstChildElement("mimic")) {
      joint.mimic = mimic->Attribute("joint") ? mimic->Attribute("joint") : "";
      joint.mimic_multiplier = attr(mimic, "multiplier", 1.0);
      joint.mimic_offset = attr(mimic, "offset", 0.0);
    }
    tree.add_joint(joint);
  }
  tree.finalize();
  return tree;
}
}  // namespace

KinematicTree parse_urdf_file(const std::string & file)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot read URDF " + file + ": " + doc.ErrorStr());
  }
  return parse(doc);
}

KinematicTree parse_urdf_string(const std::string & xml)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error(std::string("Cannot parse URDF: ") + doc.ErrorStr());
  }
  return parse(doc);
}

}  // namespace gaitos
