//  Copyright 2026 Intelligent Robotics Lab (URJC)
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_mujoco/MujocoRenderer.hpp"

#include <EGL/egl.h>
#ifdef GAITOS_MUJOCO_GLFW
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace gaitos_mujoco
{

struct MujocoRenderer::Gl
{
  EGLDisplay display{EGL_NO_DISPLAY};
  EGLContext context{EGL_NO_CONTEXT};
#ifdef GAITOS_MUJOCO_GLFW
  GLFWwindow * window{nullptr};
#endif
  bool window_open{false};
  mjvScene scene;
  mjrContext context_mj;
  mjvOption option;
  mjvCamera free_camera;
  int track_body{-1};
  bool tracking{false};
  // mouse
  bool left{false};
  bool right{false};
  double last_x{0.0};
  double last_y{0.0};
  std::chrono::steady_clock::time_point next_frame;
  std::vector<unsigned char> rgb;
  std::vector<float> depth;
};

namespace
{

std::array<double, 9> rotation_rpy(const std::vector<double> & rpy)
{
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  // Rz(y) Ry(p) Rx(r), row major (URDF fixed-axis rpy)
  return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr};
}

std::vector<double> vector3(const gaitos::Params & p, const std::string & key)
{
  auto v = p.get<std::vector<double>>(key, {0.0, 0.0, 0.0});
  if (v.size() != 3) {
    throw std::runtime_error("MujocoRenderer: " + p.full_key(key) + " needs 3 values");
  }
  return v;
}

builtin_interfaces::msg::Time stamp_of(double t)
{
  builtin_interfaces::msg::Time s;
  const auto ns = static_cast<int64_t>(std::llround(t * 1e9));
  s.sec = static_cast<int32_t>(ns / 1000000000);
  s.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return s;
}

}  // namespace

MujocoRenderer::~MujocoRenderer()
{
  deactivate();
  if (model_) {
    mj_deleteModel(model_);
  }
}

double MujocoRenderer::linear_depth(float buffer, double near, double far)
{
  // Read with mjDEPTH_ZEROFAR: the reversed depth of a perspective projection, 1 at the near
  // plane and 0 at the far one, r = near (far - z) / (z (far - near))
  const double r = static_cast<double>(buffer);
  return r > 0.0 ? near * far / (r * (far - near) + near) : 0.0;
}

void MujocoRenderer::camera_pose(
  const Camera & c, const mjtNum * p, const mjtNum * R, float pos[3], float forward[3],
  float up[3])
{
  const auto & C = c.rotation;   // body <- optical
  for (int i = 0; i < 3; ++i) {
    pos[i] = static_cast<float>(p[i] + R[3 * i] * c.xyz[0] + R[3 * i + 1] * c.xyz[1] +
      R[3 * i + 2] * c.xyz[2]);
    double z = 0.0, y = 0.0;   // world <- optical: columns z (forward) and y (down)
    for (int k = 0; k < 3; ++k) {
      z += R[3 * i + k] * C[3 * k + 2];
      y += R[3 * i + k] * C[3 * k + 1];
    }
    forward[i] = static_cast<float>(z);
    up[i] = static_cast<float>(-y);
  }
}

void MujocoRenderer::configure(
  rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core, const gaitos::Params & params)
{
  logger_ = node.get_logger().get_child("render");
  hardware_ = dynamic_cast<MujocoHardware *>(&core.hardware());
  if (!hardware_ || !hardware_->mj_model()) {
    throw std::runtime_error("MujocoRenderer: the hardware is not gaitos_mujoco/MujocoHardware");
  }
  viewer_ = params.get<bool>("viewer", false);
  const auto size = params.get<std::vector<double>>("viewer_size", {1280.0, 800.0});
  if (size.size() == 2) {
    viewer_size_ = {static_cast<int>(size[0]), static_cast<int>(size[1])};
  }
  viewer_fps_ = std::max(1.0, params.get<double>("viewer_fps", 30.0));
  viewer_track_ = params.get<std::string>("viewer_track", "pelvis");

  const mjModel * m = hardware_->mj_model();
  cameras_.clear();
  tf_static_.transforms.clear();
  const gaitos::Params block = params.sub("cameras");
  int offwidth = m->vis.global.offwidth, offheight = m->vis.global.offheight;
  for (const auto & name : block.keys()) {
    const gaitos::Params p = block.sub(name);
    Camera c;
    c.name = name;
    const auto body = p.require<std::string>("body");
    c.body = mj_name2id(m, mjOBJ_BODY, body.c_str());
    if (c.body < 0) {
      throw std::runtime_error("MujocoRenderer: camera " + name + ": no body " + body);
    }
    const auto xyz = vector3(p, "xyz");
    std::copy(xyz.begin(), xyz.end(), c.xyz.begin());
    c.rotation = rotation_rpy(vector3(p, "rpy"));
    c.width = static_cast<int>(p.get<int>("width", 640));
    c.height = static_cast<int>(p.get<int>("height", 480));
    c.fy = p.get<double>("fy", 0.5 * c.height / std::tan(0.5 * 45.0 * M_PI / 180.0));
    c.cx = p.get<double>("cx", 0.5 * c.width);
    c.cy = p.get<double>("cy", 0.5 * c.height);
    c.rate = p.get<double>("rate", 15.0);
    c.depth = p.get<bool>("depth", true);
    c.min_depth = p.get<double>("min_depth", 0.1);
    c.max_depth = p.get<double>("max_depth", 10.0);
    c.near = std::min(0.02, 0.5 * c.min_depth);
    c.far = std::max(50.0, 2.0 * c.max_depth);
    c.frame_id = p.get<std::string>("frame_id", name + "_optical_frame");
    if (c.width <= 0 || c.height <= 0 || c.fy <= 0.0 || c.rate <= 0.0) {
      throw std::runtime_error("MujocoRenderer: camera " + name +
              ": width, height, fy and rate must be > 0");
    }
    const auto ns = p.get<std::string>("namespace", "/" + name);
    const auto qos = rclcpp::SensorDataQoS();
    c.color_pub = node.create_publisher<sensor_msgs::msg::Image>(ns + "/color/image_raw", qos);
    c.color_info_pub = node.create_publisher<sensor_msgs::msg::CameraInfo>(
      ns + "/color/camera_info", qos);
    if (c.depth) {
      c.depth_pub = node.create_publisher<sensor_msgs::msg::Image>(
        ns + "/aligned_depth_to_color/image_raw", qos);
      c.depth_info_pub = node.create_publisher<sensor_msgs::msg::CameraInfo>(
        ns + "/aligned_depth_to_color/camera_info", qos);
    }
    const auto parent = p.get<std::string>("tf_parent", "");
    if (!parent.empty()) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = parent;
      t.child_frame_id = c.frame_id;
      const auto txyz = vector3(p, "tf_xyz");
      const auto trpy = vector3(p, "tf_rpy");
      t.transform.translation.x = txyz[0];
      t.transform.translation.y = txyz[1];
      t.transform.translation.z = txyz[2];
      const double cr = std::cos(trpy[0] / 2), sr = std::sin(trpy[0] / 2);
      const double cp = std::cos(trpy[1] / 2), sp = std::sin(trpy[1] / 2);
      const double cy = std::cos(trpy[2] / 2), sy = std::sin(trpy[2] / 2);
      t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
      t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
      t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
      t.transform.rotation.z = cr * cp * sy - sr * sp * cy;
      tf_static_.transforms.push_back(t);
    }
    offwidth = std::max(offwidth, c.width);
    offheight = std::max(offheight, c.height);
    RCLCPP_INFO(logger_, "camera %s on %s: %dx%d at %.1f Hz, %s/color/image_raw%s", name.c_str(),
      body.c_str(), c.width, c.height, c.rate, ns.c_str(), c.depth ? " and depth" : "");
    cameras_.push_back(std::move(c));
  }
  if (!tf_static_.transforms.empty()) {
    tf_static_pub_ = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",
        rclcpp::QoS(1).reliable().transient_local());
  }
  if (model_) {
    mj_deleteModel(model_);
  }
  model_ = mj_copyModel(nullptr, m);
  model_->vis.global.offwidth = offwidth;
  model_->vis.global.offheight = offheight;
#ifndef GAITOS_MUJOCO_GLFW
  if (viewer_) {
    RCLCPP_WARN(logger_, "viewer: gaitos_mujoco was built without GLFW, no window");
    viewer_ = false;
  }
#endif
}

void MujocoRenderer::activate()
{
  if (running_) {
    return;
  }
  if (tf_static_pub_) {
    tf_static_pub_->publish(tf_static_);
  }
  running_ = true;
  thread_ = std::thread([this]() {run();});
}

void MujocoRenderer::deactivate()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool MujocoRenderer::init_gl()
{
  gl_ = std::make_unique<Gl>();
#ifdef GAITOS_MUJOCO_GLFW
  if (viewer_) {
    if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      RCLCPP_WARN(logger_, "viewer: no DISPLAY, rendering headless");
    } else if (!glfwInit()) {
      RCLCPP_WARN(logger_, "viewer: GLFW cannot start, rendering headless");
    } else {
      glfwWindowHint(GLFW_SAMPLES, 4);
      gl_->window = glfwCreateWindow(viewer_size_[0], viewer_size_[1], "gaitos MuJoCo", nullptr,
          nullptr);
      if (gl_->window) {
        glfwMakeContextCurrent(gl_->window);
        glfwSwapInterval(0);
        gl_->window_open = true;
        return true;
      }
      RCLCPP_WARN(logger_, "viewer: no window, rendering headless");
      glfwTerminate();
    }
  }
#endif
  gl_->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (gl_->display == EGL_NO_DISPLAY || !eglInitialize(gl_->display, nullptr, nullptr)) {
    return false;
  }
  const EGLint attribs[] = {EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE,
    8, EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
  EGLConfig config;
  EGLint n = 0;
  if (!eglChooseConfig(gl_->display, attribs, &config, 1, &n) || n < 1 ||
    !eglBindAPI(EGL_OPENGL_API))
  {
    return false;
  }
  gl_->context = eglCreateContext(gl_->display, config, EGL_NO_CONTEXT, nullptr);
  return gl_->context != EGL_NO_CONTEXT &&
         eglMakeCurrent(gl_->display, EGL_NO_SURFACE, EGL_NO_SURFACE, gl_->context);
}

void MujocoRenderer::release_gl()
{
  if (!gl_) {
    return;
  }
  mjr_freeContext(&gl_->context_mj);
  mjv_freeScene(&gl_->scene);
#ifdef GAITOS_MUJOCO_GLFW
  if (gl_->window) {
    glfwDestroyWindow(gl_->window);
    glfwTerminate();
  }
#endif
  if (gl_->context != EGL_NO_CONTEXT) {
    eglMakeCurrent(gl_->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(gl_->display, gl_->context);
    eglTerminate(gl_->display);
  }
  gl_.reset();
}

void MujocoRenderer::run()
{
  if (!init_gl()) {
    RCLCPP_ERROR(logger_, "Cannot create an OpenGL context (EGL): no images. In a container "
      "without a GPU, set EGL_PLATFORM=surfaceless and LIBGL_ALWAYS_SOFTWARE=1");
    gl_.reset();
    return;
  }
  data_ = mj_makeData(model_);
  auto & g = *gl_;
  mjv_defaultScene(&g.scene);
  mjv_makeScene(model_, &g.scene, 20000);
  mjr_defaultContext(&g.context_mj);
  mjr_makeContext(model_, &g.context_mj, mjFONTSCALE_150);
  g.context_mj.readDepthMap = mjDEPTH_ZEROFAR;
  mjv_defaultOption(&g.option);
  mjv_defaultFreeCamera(model_, &g.free_camera);
  g.track_body = viewer_track_.empty() ? -1 : mj_name2id(model_, mjOBJ_BODY, viewer_track_.c_str());
  if (g.track_body >= 0) {
    g.tracking = true;
    g.free_camera.type = mjCAMERA_TRACKING;
    g.free_camera.trackbodyid = g.track_body;
    g.free_camera.distance = 3.0;
    g.free_camera.azimuth = 150.0;
    g.free_camera.elevation = -20.0;
  }
#ifdef GAITOS_MUJOCO_GLFW
  if (g.window) {
    glfwSetWindowUserPointer(g.window, this);
    glfwSetMouseButtonCallback(g.window, [](GLFWwindow * w, int, int, int) {
        auto * self = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(w));
        self->gl_->left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        self->gl_->right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        glfwGetCursorPos(w, &self->gl_->last_x, &self->gl_->last_y);
      });
    glfwSetCursorPosCallback(g.window, [](GLFWwindow * w, double x, double y) {
        auto * self = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(w));
        auto & gl = *self->gl_;
        const double dx = x - gl.last_x, dy = y - gl.last_y;
        gl.last_x = x;
        gl.last_y = y;
        if (!gl.left && !gl.right) {
          return;
        }
        int width, height;
        glfwGetWindowSize(w, &width, &height);
        const bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
          glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        const int action = gl.right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
          (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
        mjv_moveCamera(self->model_, action, dx / height, dy / height, &gl.scene, &gl.free_camera);
      });
    glfwSetScrollCallback(g.window, [](GLFWwindow * w, double, double dy) {
        auto * self = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(w));
        mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * dy, &self->gl_->scene,
          &self->gl_->free_camera);
      });
    glfwSetKeyCallback(g.window, [](GLFWwindow * w, int key, int, int act, int) {
        auto * self = static_cast<MujocoRenderer *>(glfwGetWindowUserPointer(w));
        auto & gl = *self->gl_;
        if (act == GLFW_PRESS && key == GLFW_KEY_T && gl.track_body >= 0) {
          gl.tracking = !gl.tracking;
          gl.free_camera.type = gl.tracking ? mjCAMERA_TRACKING : mjCAMERA_FREE;
          gl.free_camera.trackbodyid = gl.track_body;
        }
      });
  }
#endif
  MujocoHardware::RenderState state;
  g.next_frame = std::chrono::steady_clock::now();
  while (running_) {
    if (hardware_->render_state(state)) {
      std::copy(state.qpos.begin(), state.qpos.end(), data_->qpos);
      std::copy(state.mocap_pos.begin(), state.mocap_pos.end(), data_->mocap_pos);
      std::copy(state.mocap_quat.begin(), state.mocap_quat.end(), data_->mocap_quat);
      data_->time = state.time;
      mj_kinematics(model_, data_);
      mj_camlight(model_, data_);
      for (auto & c : cameras_) {
        if (state.time < c.last) {   // the simulation restarted
          c.next = state.time;
        }
        if (state.time + 1e-9 >= c.next) {
          render_camera(c, state.time);
          c.last = state.time;
          c.next = std::max(c.next + 1.0 / c.rate, state.time);
        }
      }
    }
    if (g.window_open && std::chrono::steady_clock::now() >= g.next_frame) {
      render_viewer();
      g.next_frame += std::chrono::microseconds(static_cast<int64_t>(1e6 / viewer_fps_));
      g.next_frame = std::max(g.next_frame, std::chrono::steady_clock::now() -
          std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  mj_deleteData(data_);
  data_ = nullptr;
  release_gl();
}

void MujocoRenderer::render_camera(Camera & c, double time)
{
  auto & g = *gl_;
  mjr_setBuffer(mjFB_OFFSCREEN, &g.context_mj);
  mjv_updateScene(model_, data_, &g.option, nullptr, &g.free_camera, mjCAT_ALL, &g.scene);
  float pos[3], forward[3], up[3];
  camera_pose(c, data_->xpos + 3 * c.body, data_->xmat + 9 * c.body, pos, forward, up);
  for (int k = 0; k < 2; ++k) {
    auto & cam = g.scene.camera[k];
    std::copy(pos, pos + 3, cam.pos);
    std::copy(forward, forward + 3, cam.forward);
    std::copy(up, up + 3, cam.up);
    // Image row 0 is the top; the frustum is at the near plane
    cam.frustum_near = static_cast<float>(c.near);
    cam.frustum_far = static_cast<float>(c.far);
    cam.frustum_top = static_cast<float>(c.near * c.cy / c.fy);
    cam.frustum_bottom = static_cast<float>(-c.near * (c.height - c.cy) / c.fy);
    cam.frustum_center = static_cast<float>(c.near * (0.5 * c.width - c.cx) / c.fy);
    cam.frustum_width = static_cast<float>(c.near * 0.5 * c.width / c.fy);
    cam.orthographic = 0;
  }
  g.scene.stereo = mjSTEREO_NONE;
  const mjrRect viewport{0, 0, c.width, c.height};
  mjr_render(viewport, &g.scene, &g.context_mj);
  const auto n = static_cast<std::size_t>(c.width) * static_cast<std::size_t>(c.height);
  g.rgb.resize(3 * n);
  g.depth.resize(n);
  mjr_readPixels(g.rgb.data(), c.depth ? g.depth.data() : nullptr, viewport, &g.context_mj);

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp_of(time);
  info.header.frame_id = c.frame_id;
  info.width = static_cast<uint32_t>(c.width);
  info.height = static_cast<uint32_t>(c.height);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {c.fy, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {c.fy, 0.0, c.cx, 0.0, 0.0, c.fy, c.cy, 0.0, 0.0, 0.0, 1.0, 0.0};

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header = info.header;
  image->width = info.width;
  image->height = info.height;
  image->encoding = "rgb8";
  image->step = 3 * info.width;
  image->data.resize(3 * n);
  const std::size_t row = 3 * static_cast<std::size_t>(c.width);
  for (int y = 0; y < c.height; ++y) {   // OpenGL rows start at the bottom
    std::memcpy(image->data.data() + row * y, g.rgb.data() + row * (c.height - 1 - y), row);
  }
  c.color_pub->publish(std::move(image));
  c.color_info_pub->publish(info);
  if (!c.depth) {
    return;
  }
  auto depth = std::make_unique<sensor_msgs::msg::Image>();
  depth->header = info.header;
  depth->width = info.width;
  depth->height = info.height;
  depth->encoding = "16UC1";
  depth->step = 2 * info.width;
  depth->data.resize(2 * n);
  auto * out = reinterpret_cast<uint16_t *>(depth->data.data());
  for (int y = 0; y < c.height; ++y) {
    const float * in = g.depth.data() + static_cast<std::size_t>(c.width) * (c.height - 1 - y);
    for (int x = 0; x < c.width; ++x) {
      const double z = linear_depth(in[x], c.near, c.far);
      out[static_cast<std::size_t>(y) * c.width + x] = (z < c.min_depth || z > c.max_depth) ? 0 :
        static_cast<uint16_t>(std::lround(z * 1000.0));
    }
  }
  c.depth_pub->publish(std::move(depth));
  c.depth_info_pub->publish(info);
}

void MujocoRenderer::render_viewer()
{
#ifdef GAITOS_MUJOCO_GLFW
  auto & g = *gl_;
  if (glfwWindowShouldClose(g.window)) {
    glfwHideWindow(g.window);
    g.window_open = false;
    RCLCPP_INFO(logger_, "viewer closed (the cameras go on)");
    return;
  }
  mjr_setBuffer(mjFB_WINDOW, &g.context_mj);
  mjrRect viewport{0, 0, 0, 0};
  glfwGetFramebufferSize(g.window, &viewport.width, &viewport.height);
  mjv_updateScene(model_, data_, &g.option, nullptr, &g.free_camera, mjCAT_ALL, &g.scene);
  mjr_render(viewport, &g.scene, &g.context_mj);
  char text[64];
  std::snprintf(text, sizeof(text), "%.1f s", data_->time);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, text,
    g.tracking ? "T: free camera" : "T: follow the robot", &g.context_mj);
  glfwSwapBuffers(g.window);
  glfwPollEvents();
#endif
}

}  // namespace gaitos_mujoco

PLUGINLIB_EXPORT_CLASS(gaitos_mujoco::MujocoRenderer, gaitos::NodeExtension)
