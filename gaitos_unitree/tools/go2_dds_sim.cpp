//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// gaitos_go2_dds_sim: headless MuJoCo Go2 that talks unitree_sdk2 DDS like the real robot
// (rt/lowcmd in, rt/lowstate out), to test the UnitreeGoHardware backend without the robot
// and without the GUI of unitree_mujoco.
//
//   gaitos_go2_dds_sim <scene.xml> [--domain 1] [--interface lo] [--duration 0]

#include <mujoco/mujoco.h>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

namespace
{
std::atomic<bool> running{true};
std::mutex mutex;
unitree_go::msg::dds_::LowCmd_ last_cmd;
std::atomic<uint64_t> cmds{0};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <scene.xml> [--domain 1] [--interface lo] [--duration 0]\n", argv[0]);
    return 2;
  }
  int domain = 1;
  std::string iface = "lo";
  double duration = 0.0;
  for (int i = 2; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--domain") {domain = std::stoi(argv[i + 1]);}
    if (a == "--interface") {iface = argv[i + 1];}
    if (a == "--duration") {duration = std::stod(argv[i + 1]);}
  }
  std::signal(SIGINT, [](int) {running = false;});
  char err[1000];
  mjModel * m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) {
    std::fprintf(stderr, "%s\n", err);
    return 1;
  }
  mjData * d = mj_makeData(m);
  const int key = mj_name2id(m, mjOBJ_KEY, "home");
  if (key >= 0) {
    mj_resetDataKeyframe(m, d, key);
  }
  d->qpos[2] = 0.15;   // lying-ish start
  mj_forward(m, d);

  unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_> pub("rt/lowstate");
  pub.InitChannel();
  unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowCmd_> sub("rt/lowcmd");
  sub.InitChannel([](const void * msg) {
      std::lock_guard<std::mutex> lock(mutex);
      last_cmd = *static_cast<const unitree_go::msg::dds_::LowCmd_ *>(msg);
      ++cmds;
    }, 1);

  // Unitree motor order of the go2 MJCF actuators (FR, FL, RR, RL); feet geoms FR, FL, RR, RL
  const char * feet[4] = {"FR", "FL", "RR", "RL"};
  int foot_geom[4];
  for (int i = 0; i < 4; ++i) {
    foot_geom[i] = mj_name2id(m, mjOBJ_GEOM, feet[i]);
  }
  const int quat = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "imu_quat")];
  const int gyro = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "imu_gyro")];
  const int acc = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "imu_acc")];
  unitree_go::msg::dds_::LowState_ state;
  auto next = std::chrono::steady_clock::now();
  const auto period = std::chrono::microseconds(static_cast<int>(m->opt.timestep * 1e6));
  int step = 0;
  while (running && (duration <= 0.0 || d->time < duration)) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (int i = 0; i < m->nu && i < 12; ++i) {
        const auto & c = last_cmd.motor_cmd()[static_cast<std::size_t>(i)];
        const int jnt = m->actuator_trnid[2 * i];
        const double q = d->qpos[m->jnt_qposadr[jnt]];
        const double dq = d->qvel[m->jnt_dofadr[jnt]];
        const double q_ref = std::abs(c.q()) > 1e8 ? q : c.q();
        const double dq_ref = std::abs(c.dq()) > 1e4 ? 0.0 : c.dq();
        d->ctrl[i] = c.kp() * (q_ref - q) + c.kd() * (dq_ref - dq) + c.tau();
      }
    }
    mj_step(m, d);
    if (++step % 2 == 0) {   // 500 Hz
      for (int i = 0; i < 12; ++i) {
        const int jnt = m->actuator_trnid[2 * i];
        auto & ms = state.motor_state()[static_cast<std::size_t>(i)];
        ms.q() = static_cast<float>(d->qpos[m->jnt_qposadr[jnt]]);
        ms.dq() = static_cast<float>(d->qvel[m->jnt_dofadr[jnt]]);
        ms.tau_est() = static_cast<float>(d->actuator_force[i]);
      }
      for (int k = 0; k < 4; ++k) {
        state.imu_state().quaternion()[static_cast<std::size_t>(k)] = static_cast<float>(d->sensordata[quat + k]);
      }
      for (int k = 0; k < 3; ++k) {
        state.imu_state().gyroscope()[static_cast<std::size_t>(k)] = static_cast<float>(d->sensordata[gyro + k]);
        state.imu_state().accelerometer()[static_cast<std::size_t>(k)] = static_cast<float>(d->sensordata[acc + k]);
      }
      std::array<double, 4> force{0, 0, 0, 0};
      for (int c = 0; c < d->ncon; ++c) {
        for (int f = 0; f < 4; ++f) {
          if (d->contact[c].geom1 == foot_geom[f] || d->contact[c].geom2 == foot_geom[f]) {
            mjtNum w[6];
            mj_contactForce(m, d, c, w);
            force[static_cast<std::size_t>(f)] += std::abs(w[0]);
          }
        }
      }
      for (int f = 0; f < 4; ++f) {
        state.foot_force()[static_cast<std::size_t>(f)] = static_cast<int16_t>(force[static_cast<std::size_t>(f)]);
      }
      state.tick() = static_cast<uint32_t>(d->time * 1000);
      pub.Write(state);
    }
    if (step % 2000 == 0) {
      std::printf("t %.1f base z %.3f x %.3f lowcmd received %lu\n", d->time, d->qpos[2], d->qpos[0],
        static_cast<unsigned long>(cmds.load()));
      std::fflush(stdout);
    }
    next += period;
    std::this_thread::sleep_until(next);
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
