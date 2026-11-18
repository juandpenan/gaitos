//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_mujoco/MujocoHardware.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace gaitos_mujoco
{

MujocoHardware::~MujocoHardware()
{
  stop();
}

bool MujocoHardware::on_initialize()
{
  file_ = gaitos::resolve_uri(params().require<std::string>("model"));
  period_ = params().get<double>("period", period_);
  hold_kp_ = params().get<double>("hold_gains.kp", hold_kp_);
  hold_kd_ = params().get<double>("hold_gains.kd", hold_kd_);
  ground_truth_ = params().get<bool>("ground_truth", ground_truth_);
  if (!load_model()) {
    return false;
  }
  const double timestep = params().get<double>("timestep", 0.0);
  if (timestep > 0.0) {
    model_->opt.timestep = timestep;
  }
  substeps_ = std::max(1, static_cast<int>(std::lround(period_ / model_->opt.timestep)));
  if (std::abs(substeps_ * model_->opt.timestep - period_) > 1e-9) {
    logger().warn() << "period " << period_ << " is not a multiple of the timestep " <<
      model_->opt.timestep << ": using " << substeps_ * model_->opt.timestep;
    period_ = substeps_ * model_->opt.timestep;
  }

  // Floating base
  const auto base = params().get<std::string>("base_body", "");
  if (!base.empty()) {
    base_body_ = mj_name2id(model_, mjOBJ_BODY, base.c_str());
  } else {
    for (int j = 0; j < model_->njnt; ++j) {
      if (model_->jnt_type[j] == mjJNT_FREE) {
        base_body_ = model_->jnt_bodyid[j];
        break;
      }
    }
  }
  if (base_body_ < 0) {
    logger().error() << "No floating base body found (base_body)";
    return false;
  }
  const int free_joint = model_->body_jntadr[base_body_];
  if (free_joint < 0 || model_->jnt_type[free_joint] != mjJNT_FREE) {
    logger().error() << "Body " << base << " has no free joint";
    return false;
  }
  base_qpos_ = model_->jnt_qposadr[free_joint];
  base_dof_ = model_->jnt_dofadr[free_joint];

  // Joints
  const gaitos::Params joint_map = params().sub("joint_map");
  joints_.assign(model().joint_count(), JointMap());
  std::vector<bool> used(static_cast<std::size_t>(model_->njnt), false);
  std::string missing;
  for (std::size_t i = 0; i < model().joint_count(); ++i) {
    std::string name = model().joint(i).name;
    if (joint_map.has(name)) {
      name = joint_map.get<std::string>(name, name);
    }
    const int jid = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    if (jid < 0) {
      missing += name + " ";
      continue;
    }
    used[static_cast<std::size_t>(jid)] = true;
    auto & jm = joints_[i];
    jm.qpos = model_->jnt_qposadr[jid];
    jm.dof = model_->jnt_dofadr[jid];
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == jid) {
        jm.actuator = a;
        jm.gear = model_->actuator_gear[6 * a] != 0.0 ? model_->actuator_gear[6 * a] : 1.0;
        break;
      }
    }
  }
  if (!missing.empty()) {
    logger().warn() << "Joints not in the simulation (they follow the command): " << missing;
  }
  // Mimic joints of the URDF (not in the model joint vector): they follow their master
  mimic_.clear();
  std::string mimics;
  for (const auto & [name, tj] : model().tree().joints()) {
    if (tj.mimic.empty()) {
      continue;
    }
    const auto master = model().tree().joints().find(tj.mimic);
    const int jid = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
    if (jid < 0 || master == model().tree().joints().end() || master->second.index < 0 ||
      used[static_cast<std::size_t>(jid)])
    {
      continue;
    }
    used[static_cast<std::size_t>(jid)] = true;
    MimicJoint m{model_->jnt_qposadr[jid], model_->jnt_dofadr[jid], -1, 1.0,
      static_cast<std::size_t>(master->second.index), tj.mimic_multiplier, tj.mimic_offset};
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == jid) {
        m.actuator = a;
        m.gear = model_->actuator_gear[6 * a] != 0.0 ? model_->actuator_gear[6 * a] : 1.0;
      }
    }
    mimic_.push_back(m);
    mimics += name + " ";
  }
  if (!mimics.empty()) {
    logger().info() << "Simulated mimic joints (follow their master): " << mimics;
  }

  // Simulated joints that gaitos does not control: held where they start
  held_.clear();
  std::string held;
  for (int j = 0; j < model_->njnt; ++j) {
    if (used[static_cast<std::size_t>(j)] || (model_->jnt_type[j] != mjJNT_HINGE &&
      model_->jnt_type[j] != mjJNT_SLIDE))
    {
      continue;
    }
    HeldJoint h{model_->jnt_qposadr[j], model_->jnt_dofadr[j], -1, 1.0, 0.0};
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == j) {
        h.actuator = a;
        h.gear = model_->actuator_gear[6 * a] != 0.0 ? model_->actuator_gear[6 * a] : 1.0;
      }
    }
    held_.push_back(h);
    held += std::string(mj_id2name(model_, mjOBJ_JOINT, j)) + " ";
  }
  if (!held.empty()) {
    logger().info() << "Simulated joints held in place: " << held;
  }

  // IMU sensors
  imu_quat_ = sensor_adr(params().get<std::string>("imu.orientation", "imu_quat"), 4);
  imu_gyro_ = sensor_adr(params().get<std::string>("imu.gyro", "imu_gyro"), 3);
  imu_acc_ = sensor_adr(params().get<std::string>("imu.accelerometer", "imu_acc"), 3);

  // Feet: geoms whose contacts count for each leg
  geom_foot_.assign(static_cast<std::size_t>(model_->ngeom), -1);
  foot_geoms_.assign(model().leg_count(), {});
  const gaitos::Params feet = params().sub("feet");
  auto add_body = [this](std::size_t leg, int body) {
      for (int g = model_->body_geomadr[body];
        g >= 0 && g < model_->body_geomadr[body] + model_->body_geomnum[body]; ++g)
      {
        if (model_->geom_contype[g] || model_->geom_conaffinity[g]) {
          foot_geoms_[leg].push_back(g);
        }
      }
    };
  for (std::size_t l = 0; l < model().leg_count(); ++l) {
    const auto & leg = model().leg(l);
    const gaitos::Params f = feet.sub(leg.name);
    for (const auto & g : f.get<std::vector<std::string>>("geoms", {})) {
      const int id = mj_name2id(model_, mjOBJ_GEOM, g.c_str());
      if (id < 0) {
        logger().error() << "Unknown foot geom " << g;
        return false;
      }
      foot_geoms_[l].push_back(id);
    }
    for (const auto & b : f.get<std::vector<std::string>>("bodies", {})) {
      const int id = mj_name2id(model_, mjOBJ_BODY, b.c_str());
      if (id < 0) {
        logger().error() << "Unknown foot body " << b;
        return false;
      }
      add_body(l, id);
    }
    if (foot_geoms_[l].empty()) {
      // Default: the body of the foot link, or its parent if it has no collision geometry
      int body = mj_name2id(model_, mjOBJ_BODY, leg.foot.c_str());
      if (body >= 0) {
        add_body(l, body);
        if (foot_geoms_[l].empty()) {
          add_body(l, model_->body_parentid[body]);
        }
      }
    }
    if (foot_geoms_[l].empty()) {
      logger().warn() << "Leg " << leg.name << ": no foot geometry, no contact force";
    }
    for (const int g : foot_geoms_[l]) {
      geom_foot_[static_cast<std::size_t>(g)] = static_cast<int>(l);
    }
  }
  command_.resize(model().joint_count());
  fake_q_.assign(model().joint_count(), 0.0);
  logger().info() << "Loaded " << file_ << ": timestep " << model_->opt.timestep << " s, " <<
    substeps_ << " steps per cycle";
  return true;
}

bool MujocoHardware::load_model()
{
  char error[1024] = "";
  const bool urdf = std::filesystem::path(file_).extension() == ".urdf";
  if (!urdf) {
    model_ = mj_loadXML(file_.c_str(), nullptr, error, sizeof(error));
  } else {
    // URDF: add the free joint of the base and a floor
    mjSpec * spec = mj_parseXML(file_.c_str(), nullptr, error, sizeof(error));
    if (spec) {
      mjsBody * world = mjs_findBody(spec, "world");
      mjsBody * root = world ? mjs_asBody(mjs_firstChild(world, mjOBJ_BODY, 0)) : nullptr;
      if (root) {
        mjs_addFreeJoint(root);
      }
      mjsGeom * floor = mjs_addGeom(world, nullptr);
      floor->type = mjGEOM_PLANE;
      floor->size[0] = floor->size[1] = 0.0;
      floor->size[2] = 0.05;
      model_ = mj_compile(spec, nullptr);
      mj_deleteSpec(spec);
    }
  }
  if (!model_) {
    logger().error() << "Cannot load " << file_ << ": " << error;
    return false;
  }
  data_ = mj_makeData(model_);
  return data_ != nullptr;
}

int MujocoHardware::sensor_adr(const std::string & name, int dim) const
{
  const int id = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
  if (id < 0 || model_->sensor_dim[id] != dim) {
    return -1;
  }
  return model_->sensor_adr[id];
}

void MujocoHardware::place_robot()
{
  const auto keyframe = params().get<std::string>("keyframe", "");
  const int key = keyframe.empty() ? -1 : mj_name2id(model_, mjOBJ_KEY, keyframe.c_str());
  if (key >= 0) {
    mj_resetDataKeyframe(model_, data_, key);
  } else {
    mj_resetData(model_, data_);
  }
  const auto pose = params().get<std::string>("initial_pose", "");
  if (!pose.empty()) {
    const auto q = model().pose(pose);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (joints_[i].qpos >= 0) {
        data_->qpos[joints_[i].qpos] = q[i];
      }
    }
    fake_q_ = q;
  }
  const double height = params().get<double>("initial_height", -1.0);
  if (key < 0 || height >= 0.0) {
    data_->qpos[base_qpos_ + 2] = height >= 0.0 ? height : 1.0;
  }
  mj_forward(model_, data_);
  if (key < 0 && height < 0.0) {
    // Lowest point of the robot collision geometry on the floor
    double min_z = std::numeric_limits<double>::infinity();
    for (int g = 0; g < model_->ngeom; ++g) {
      if (model_->geom_bodyid[g] == 0 || !(model_->geom_contype[g] || model_->geom_conaffinity[g])) {
        continue;
      }
      const mjtNum * p = data_->geom_xpos + 3 * g;
      const mjtNum * r = data_->geom_xmat + 9 * g;   // row major
      const mjtNum * s = model_->geom_size + 3 * g;
      double z = p[2] - model_->geom_rbound[g];
      switch (model_->geom_type[g]) {
        case mjGEOM_SPHERE: z = p[2] - s[0]; break;
        case mjGEOM_CAPSULE: z = p[2] - s[0] - std::abs(r[8]) * s[1]; break;
        case mjGEOM_BOX:
          z = p[2] - std::abs(r[6]) * s[0] - std::abs(r[7]) * s[1] - std::abs(r[8]) * s[2];
          break;
        case mjGEOM_MESH: {
            const int mesh = model_->geom_dataid[g];
            const float * v = model_->mesh_vert + 3 * model_->mesh_vertadr[mesh];
            z = std::numeric_limits<double>::infinity();
            for (int k = 0; k < model_->mesh_vertnum[mesh]; ++k) {
              z = std::min(z, p[2] + r[6] * v[3 * k] + r[7] * v[3 * k + 1] + r[8] * v[3 * k + 2]);
            }
            break;
          }
        default: break;
      }
      min_z = std::min(min_z, z);
    }
    if (std::isfinite(min_z)) {
      data_->qpos[base_qpos_ + 2] -= min_z - 0.002;
    }
    mj_forward(model_, data_);
  }
  for (auto & h : held_) {
    h.target = data_->qpos[h.qpos];
  }
  last_velocity_.setZero();
  logger().info() << "Robot placed at height " << data_->qpos[base_qpos_ + 2] << " m" <<
    (pose.empty() ? std::string("") : " in pose " + pose);
}

bool MujocoHardware::start()
{
  if (!model_ || !data_) {
    return false;
  }
  place_robot();
  // Hold the initial pose until the first command
  command_.set_zero();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    command_.position[i] = joints_[i].qpos >= 0 ? data_->qpos[joints_[i].qpos] : fake_q_[i];
  }
  return true;
}

void MujocoHardware::stop()
{
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

void MujocoHardware::apply_torques()
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & jm = joints_[i];
    if (jm.qpos < 0) {
      continue;
    }
    double tau = command_.torque(i, data_->qpos[jm.qpos], data_->qvel[jm.dof]);
    const double limit = model().joint(i).effort;
    if (limit > 0.0) {
      tau = std::clamp(tau, -limit, limit);
    }
    if (jm.actuator >= 0) {
      data_->ctrl[jm.actuator] = tau / jm.gear;
    } else {
      data_->qfrc_applied[jm.dof] = tau;
    }
  }
  for (const auto & m : mimic_) {
    const auto i = m.master;
    if (i >= command_.position.size() || i >= command_.kp.size()) {
      continue;
    }
    const double target = m.multiplier * command_.position[i] + m.offset;
    const double target_v = i < command_.velocity.size() ? m.multiplier * command_.velocity[i] : 0.0;
    double tau = command_.kp[i] * (target - data_->qpos[m.qpos]) +
      (i < command_.kd.size() ? command_.kd[i] : 0.0) * (target_v - data_->qvel[m.dof]);
    const double limit = model().joint(i).effort;
    if (limit > 0.0) {
      tau = std::clamp(tau, -limit, limit);
    }
    if (m.actuator >= 0) {
      data_->ctrl[m.actuator] = tau / m.gear;
    } else {
      data_->qfrc_applied[m.dof] = tau;
    }
  }
  for (const auto & h : held_) {
    const double tau = hold_kp_ * (h.target - data_->qpos[h.qpos]) - hold_kd_ * data_->qvel[h.dof];
    if (h.actuator >= 0) {
      data_->ctrl[h.actuator] = tau / h.gear;
    } else {
      data_->qfrc_applied[h.dof] = tau;
    }
  }
}

bool MujocoHardware::read(gaitos::RobotState & state)
{
  if (!data_) {
    return false;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & jm = joints_[i];
    if (jm.qpos < 0) {
      state.joints.position[i] = fake_q_[i];
      state.joints.velocity[i] = 0.0;
      state.joints.effort[i] = 0.0;
      continue;
    }
    state.joints.position[i] = data_->qpos[jm.qpos];
    state.joints.velocity[i] = data_->qvel[jm.dof];
    state.joints.effort[i] = jm.actuator >= 0 ? data_->actuator_force[jm.actuator] * jm.gear :
      data_->qfrc_applied[jm.dof];
  }

  // Base
  const mjtNum * q = data_->qpos + base_qpos_;
  const Eigen::Quaterniond orientation(q[3], q[4], q[5], q[6]);
  const Eigen::Vector3d v_world(data_->qvel[base_dof_], data_->qvel[base_dof_ + 1],
    data_->qvel[base_dof_ + 2]);
  const Eigen::Vector3d w_body(data_->qvel[base_dof_ + 3], data_->qvel[base_dof_ + 4],
    data_->qvel[base_dof_ + 5]);

  auto & imu = state.imu_raw;
  const mjtNum * s = data_->sensordata;
  imu.orientation = imu_quat_ >= 0 ?
    Eigen::Quaterniond(s[imu_quat_], s[imu_quat_ + 1], s[imu_quat_ + 2], s[imu_quat_ + 3]) :
    orientation;
  imu.angular_velocity = imu_gyro_ >= 0 ?
    Eigen::Vector3d(s[imu_gyro_], s[imu_gyro_ + 1], s[imu_gyro_ + 2]) : w_body;
  if (imu_acc_ >= 0) {
    imu.linear_acceleration = Eigen::Vector3d(s[imu_acc_], s[imu_acc_ + 1], s[imu_acc_ + 2]);
  } else {
    const Eigen::Vector3d a = (v_world - last_velocity_) / period_ + Eigen::Vector3d(0.0, 0.0, 9.81);
    imu.linear_acceleration = orientation.conjugate() * a;
  }
  last_velocity_ = v_world;
  imu.has_orientation = true;
  imu.stamp = data_->time;

  // Contact forces of the feet
  state.foot_force_raw.assign(model().leg_count(), 0.0);
  for (int c = 0; c < data_->ncon; ++c) {
    const auto & con = data_->contact[c];
    int leg = con.geom1 >= 0 ? geom_foot_[static_cast<std::size_t>(con.geom1)] : -1;
    if (leg < 0 && con.geom2 >= 0) {
      leg = geom_foot_[static_cast<std::size_t>(con.geom2)];
    }
    if (leg < 0) {
      continue;
    }
    mjtNum f[6];
    mj_contactForce(model_, data_, c, f);
    state.foot_force_raw[static_cast<std::size_t>(leg)] += std::abs(f[0]);
  }

  if (ground_truth_) {
    auto & gt = state.external_odometry["ground_truth"];
    gt.odometry.stamp = data_->time;
    gt.odometry.position = Eigen::Vector3d(q[0], q[1], q[2]);
    gt.odometry.orientation = orientation;
    gt.odometry.linear_velocity = orientation.conjugate() * v_world;
    gt.odometry.angular_velocity = w_body;
    gt.reception = data_->time;
    gt.seq = ++seq_;
  }
  return true;
}

bool MujocoHardware::write(const gaitos::JointCommand & command)
{
  if (!data_) {
    return false;
  }
  command_ = command;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].qpos < 0) {
      fake_q_[i] = command.position[i];
    }
  }
  for (int k = 0; k < substeps_; ++k) {
    apply_torques();
    mj_step(model_, data_);
  }
  for (int i = 0; i < model_->nq; ++i) {
    if (!std::isfinite(data_->qpos[i])) {
      logger().error() << "Simulation diverged";
      return false;
    }
  }
  return true;
}

}  // namespace gaitos_mujoco

PLUGINLIB_EXPORT_CLASS(gaitos_mujoco::MujocoHardware, gaitos::HardwareInterface)
