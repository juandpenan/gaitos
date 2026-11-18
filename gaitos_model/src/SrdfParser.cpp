//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <tinyxml2.h>

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>

#include "gaitos_model/RobotModel.hpp"

namespace gaitos
{

namespace
{
std::string name_of(const tinyxml2::XMLElement * e)
{
  const char * n = e->Attribute("name");
  return n ? n : "";
}

SrdfData parse(tinyxml2::XMLDocument & doc, const KinematicTree & tree)
{
  const auto * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("SRDF without <robot> element");
  }
  SrdfData data;
  data.robot_name = name_of(robot);

  // Groups may reference groups defined later: keep the elements and resolve recursively
  std::map<std::string, const tinyxml2::XMLElement *> elements;
  for (const auto * g = robot->FirstChildElement("group"); g; g = g->NextSiblingElement("group")) {
    elements[name_of(g)] = g;
  }
  std::function<SrdfGroup(const std::string &, int)> resolve =
    [&](const std::string & name, int depth) -> SrdfGroup {
      if (depth > 16) {
        throw std::runtime_error("SRDF group recursion too deep at " + name);
      }
      const auto it = elements.find(name);
      if (it == elements.end()) {
        throw std::runtime_error("SRDF references unknown group " + name);
      }
      SrdfGroup group;
      group.name = name;
      auto add_joint = [&](const std::string & j) {
          if (std::find(group.joints.begin(), group.joints.end(), j) == group.joints.end()) {
            group.joints.push_back(j);
          }
        };
      for (const auto * c = it->second->FirstChildElement(); c; c = c->NextSiblingElement()) {
        const std::string tag = c->Name();
        if (tag == "joint") {
          const auto jt = tree.joints().find(name_of(c));
          if (jt == tree.joints().end()) {
            throw std::runtime_error("SRDF group " + name + " references unknown joint " +
                    name_of(c));
          }
          if (jt->second.movable() && jt->second.mimic.empty()) {
            add_joint(jt->first);
          }
        } else if (tag == "link") {
          group.links.push_back(name_of(c));
        } else if (tag == "chain") {
          group.chain_base = c->Attribute("base_link") ? c->Attribute("base_link") : "";
          group.chain_tip = c->Attribute("tip_link") ? c->Attribute("tip_link") : "";
          for (const auto & j : tree.chain_joints(group.chain_base, group.chain_tip)) {
            if (tree.joints().at(j).movable() && tree.joints().at(j).mimic.empty()) {
              add_joint(j);
            }
          }
        } else if (tag == "group") {
          for (const auto & j : resolve(name_of(c), depth + 1).joints) {
            add_joint(j);
          }
        }
      }
      return group;
    };
  for (const auto & [name, element] : elements) {
    (void)element;
    data.groups[name] = resolve(name, 0);
  }

  for (const auto * s = robot->FirstChildElement("group_state"); s;
    s = s->NextSiblingElement("group_state"))
  {
    auto & pose = data.poses[name_of(s)];
    for (const auto * j = s->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
      double value = 0.0;
      if (j->QueryDoubleAttribute("value", &value) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("SRDF group_state " + name_of(s) + ": joint " + name_of(j) +
                " without a numeric value");
      }
      if (tree.joints().count(name_of(j)) == 0) {
        throw std::runtime_error("SRDF group_state " + name_of(s) + " references unknown joint " +
                name_of(j));
      }
      pose[name_of(j)] = value;
    }
  }
  for (const auto * e = robot->FirstChildElement("end_effector"); e;
    e = e->NextSiblingElement("end_effector"))
  {
    data.end_effectors[name_of(e)] = e->Attribute("parent_link") ? e->Attribute("parent_link") : "";
  }
  return data;
}
}  // namespace

SrdfData parse_srdf_file(const std::string & file, const KinematicTree & tree)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot read SRDF " + file + ": " + doc.ErrorStr());
  }
  return parse(doc, tree);
}

SrdfData parse_srdf_string(const std::string & xml, const KinematicTree & tree)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error(std::string("Cannot parse SRDF: ") + doc.ErrorStr());
  }
  return parse(doc, tree);
}

}  // namespace gaitos
