//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// snapshot: runs gaitos on the MuJoCo backend with scheduled commands and saves offscreen
// renders (EGL, no window) at given robot times. Used to make the documentation images.
//
//   ros2 run gaitos_mujoco snapshot --at 0.5 "mode stand_up" --shot 3 stand.png
//     [--width 1280 --height 720] [--azimuth 135 --elevation -15 --distance 1.6]
//     --ros-args --params-file gaitos.yaml --params-file go2.yaml --params-file go2_mujoco.yaml
//   (a --shot may carry its own view: --shot 3 stand.png --azimuth 90 ...)
//   --frames <t0> <t1> <fps> <prefix>: a shot every 1/fps s from t0 to t1, <prefix>_0000.png...
//     (with the view given before it), for animations
// Commands: mode <name> | vel <vx> <vy> <wz> | stop | pose <x> <y> <z> <r> <p> <y> | goal <x> <y> <yaw>
//   | target <joint> <position> [speed]  (a joint over the controller, Gaitos::set_joint_targets)

#include <EGL/egl.h>
#include <zlib.h>

#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "gaitos_mujoco/MujocoHardware.hpp"
#include "gaitos_common/Math.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

namespace
{
struct View
{
  double azimuth{135.0};
  double elevation{-15.0};
  double distance{1.6};
  double lookat_z{0.0};
};

struct Shot
{
  std::string file;
  View view;
};

void put32(std::vector<unsigned char> & v, uint32_t x)
{
  for (int s = 24; s >= 0; s -= 8) {
    v.push_back(static_cast<unsigned char>((x >> s) & 0xFF));
  }
}

void chunk(std::FILE * f, const char * type, const std::vector<unsigned char> & data)
{
  std::vector<unsigned char> buf;
  put32(buf, static_cast<uint32_t>(data.size()));
  buf.insert(buf.end(), type, type + 4);
  buf.insert(buf.end(), data.begin(), data.end());
  uLong crc = crc32(0L, buf.data() + 4, static_cast<uInt>(buf.size() - 4));
  put32(buf, static_cast<uint32_t>(crc));
  std::fwrite(buf.data(), 1, buf.size(), f);
}

bool write_png(const std::string & file, const unsigned char * rgb, int w, int h)
{
  std::vector<unsigned char> raw;
  raw.reserve(static_cast<std::size_t>((w * 3 + 1) * h));
  for (int y = h - 1; y >= 0; --y) {   // OpenGL rows go bottom up
    raw.push_back(0);
    raw.insert(raw.end(), rgb + y * w * 3, rgb + (y + 1) * w * 3);
  }
  uLongf size = compressBound(static_cast<uLong>(raw.size()));
  std::vector<unsigned char> z(size);
  if (compress2(z.data(), &size, raw.data(), static_cast<uLong>(raw.size()), 6) != Z_OK) {
    return false;
  }
  z.resize(size);
  std::FILE * f = std::fopen(file.c_str(), "wb");
  if (!f) {
    return false;
  }
  const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  std::fwrite(sig, 1, 8, f);
  std::vector<unsigned char> ihdr;
  put32(ihdr, static_cast<uint32_t>(w));
  put32(ihdr, static_cast<uint32_t>(h));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
  chunk(f, "IHDR", ihdr);
  chunk(f, "IDAT", z);
  chunk(f, "IEND", {});
  std::fclose(f);
  return true;
}

bool init_egl()
{
  EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (display == EGL_NO_DISPLAY || !eglInitialize(display, nullptr, nullptr)) {
    return false;
  }
  const EGLint attribs[] = {EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
    EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig config;
  EGLint n = 0;
  if (!eglChooseConfig(display, attribs, &config, 1, &n) || n < 1 || !eglBindAPI(EGL_OPENGL_API)) {
    return false;
  }
  EGLContext ctx = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
  return ctx != EGL_NO_CONTEXT && eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx);
}

/// Text command for the gaitos core
std::string execute(gaitos::Gaitos & g, const std::string & line, bool & twist_on,
  Eigen::Vector3d & lin, Eigen::Vector3d & ang)
{
  std::istringstream in(line);
  std::string cmd;
  in >> cmd;
  if (cmd == "mode") {
    std::string m;
    in >> m;
    auto f = g.request_mode(m);
    g.step();
    const auto r = f.get();
    return r.success ? "ok" : r.message;
  }
  if (cmd == "vel") {
    double vx = 0, vy = 0, wz = 0;
    in >> vx >> vy >> wz;
    lin = {vx, vy, 0.0};
    ang = {0.0, 0.0, wz};
    twist_on = true;
    g.set_twist(lin, ang);
    return "ok";
  }
  if (cmd == "stop") {
    twist_on = false;
    g.set_twist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return "ok";
  }
  if (cmd == "target") {
    gaitos::JointTarget t;
    in >> t.name >> t.position;
    if (!(in >> t.speed)) {
      t.speed = 0.0;
    }
    return g.set_joint_targets({t}) ? "ok" : "unknown joint";
  }
  if (cmd == "pose" || cmd == "goal") {
    double v[6] = {0, 0, 0, 0, 0, 0};
    for (auto & x : v) {
      in >> x;
    }
    if (cmd == "goal") {
      g.set_pose_goal({v[0], v[1], 0.0}, gaitos::rpy_to_quaternion({0.0, 0.0, v[2]}));
    } else {
      g.set_pose_goal({v[0], v[1], v[2]}, gaitos::rpy_to_quaternion({v[3], v[4], v[5]}));
    }
    return "ok";
  }
  return "unknown command";
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::multimap<double, std::string> commands;
  std::multimap<double, Shot> shots;
  int width = 1280, height = 720;
  View view;
  View * current = &view;
  const int n = static_cast<int>(args.size());
  for (int i = 1; i < n; ++i) {
    const std::string a = args[static_cast<std::size_t>(i)];
    auto next = [&]() {return std::string(i + 1 < n ? args[static_cast<std::size_t>(++i)] : "");};
    if (a == "--at") {
      const double t = std::stod(next());
      commands.emplace(t, next());
    } else if (a == "--shot") {
      const double t = std::stod(next());
      auto it = shots.emplace(t, Shot{next(), view});
      current = &it->second.view;
    } else if (a == "--frames") {
      const double t0 = std::stod(next());
      const double t1 = std::stod(next());
      const double fps = std::stod(next());
      const std::string prefix = next();
      for (int k = 0; t0 + k / fps <= t1 + 1e-9; ++k) {
        char name[32];
        std::snprintf(name, sizeof(name), "_%04d.png", k);
        shots.emplace(t0 + k / fps, Shot{prefix + name, view});
      }
    } else if (a == "--width") {
      width = std::stoi(next());
    } else if (a == "--height") {
      height = std::stoi(next());
    } else if (a == "--azimuth") {
      current->azimuth = std::stod(next());
    } else if (a == "--elevation") {
      current->elevation = std::stod(next());
    } else if (a == "--distance") {
      current->distance = std::stod(next());
    } else if (a == "--lookat-z") {
      current->lookat_z = std::stod(next());
    }
  }
  if (shots.empty()) {
    std::cerr << "usage: snapshot [--at <t> \"<cmd>\"]... --shot <t> <file.png> "
      "[--azimuth a --elevation e --distance d --lookat-z z]... --ros-args --params-file ...\n";
    return 2;
  }
  if (!init_egl()) {
    std::cerr << "Cannot create an EGL OpenGL context\n";
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>(
    "snapshot", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  gaitos::Gaitos g;
  try {
    g.configure(gaitos::Params(node->get_node_parameters_interface()));
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  if (!g.start()) {
    return 1;
  }
  auto * hw = dynamic_cast<gaitos_mujoco::MujocoHardware *>(&g.hardware());
  if (!hw) {
    std::cerr << "The hardware is not MujocoHardware\n";
    return 1;
  }
  const mjModel * m = hw->mj_model();
  mjvScene scn;
  mjvCamera cam;
  mjvOption opt;
  mjrContext con;
  mjv_defaultScene(&scn);
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_100);
  mjr_setBuffer(mjFB_OFFSCREEN, &con);
  width = std::min(width, con.offWidth);
  height = std::min(height, con.offHeight);
  std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3));
  scn.flags[mjRND_SHADOW] = 1;

  // Track the floating base
  int base = -1;
  for (int j = 0; j < m->njnt; ++j) {
    if (m->jnt_type[j] == mjJNT_FREE) {
      base = m->jnt_bodyid[j];
      break;
    }
  }

  bool twist_on = false;
  Eigen::Vector3d lin = Eigen::Vector3d::Zero(), ang = Eigen::Vector3d::Zero();
  const double t0 = g.hardware().now();
  double last_tick = -1.0;
  const double end = shots.rbegin()->first + 1e-6;
  while (g.hardware().now() - t0 <= end) {
    const double t = g.hardware().now() - t0;
    while (!commands.empty() && commands.begin()->first <= t) {
      std::cout << "[t=" << t << "] " << commands.begin()->second << ": " <<
        execute(g, commands.begin()->second, twist_on, lin, ang) << std::endl;
      commands.erase(commands.begin());
    }
    if (t - last_tick >= 0.1 && twist_on) {
      g.set_twist(lin, ang);
      last_tick = t;
    }
    g.step();
    while (!shots.empty() && shots.begin()->first <= g.hardware().now() - t0) {
      const auto & s = shots.begin()->second;
      cam.type = mjCAMERA_FREE;
      mjData * d = hw->mj_data();
      for (int k = 0; k < 3; ++k) {
        cam.lookat[k] = base >= 0 ? d->xpos[3 * base + k] : 0.0;
      }
      cam.lookat[2] += s.view.lookat_z;
      cam.azimuth = s.view.azimuth;
      cam.elevation = s.view.elevation;
      cam.distance = s.view.distance;
      mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
      const mjrRect viewport{0, 0, width, height};
      mjr_render(viewport, &scn, &con);
      mjr_readPixels(rgb.data(), nullptr, viewport, &con);
      const bool ok = write_png(s.file, rgb.data(), width, height);
      if (!ok || s.file.find("_0") == std::string::npos || s.file.find("_0000") != std::string::npos) {
        std::cout << (ok ? "saved " : "FAILED ") << s.file << " (t=" << g.hardware().now() - t0 <<
          ", mode " << g.state().mode << ")" << std::endl;
      }
      shots.erase(shots.begin());
    }
  }
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  g.stop();
  rclcpp::shutdown();
  return 0;
}
