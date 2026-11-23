//  Copyright 2026 Intelligent Robotics Lab (URJC)
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MUJOCO__MUJOCORENDERER_HPP_
#define GAITOS_MUJOCO__MUJOCORENDERER_HPP_

#include <mujoco/mujoco.h>

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gaitos_mujoco/MujocoHardware.hpp"
#include "gaitos_system/NodeExtension.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace gaitos_mujoco
{

/**
 * @class MujocoRenderer
 * @brief Node extension that renders the MuJoCo simulation of the gaitos node
 * (MujocoHardware), in a thread of its own so the control loop never waits for it: simulated
 * cameras published as ROS images, and optionally a viewer window.
 *
 * Each camera is fixed to a body of the model and given like a ROS optical frame (z forward,
 * x right, y down) with pinhole intrinsics, so it can copy a real one. It publishes, stamped
 * with the simulated time, `<namespace>/color/image_raw` (rgb8), `<namespace>/color/camera_info`
 * and, with `depth`, `<namespace>/aligned_depth_to_color/image_raw` (16UC1, mm, 0 = no data) and
 * its camera_info: the topics of a RealSense with aligned depth. The images see the visual
 * geometry (groups 0-2) of the model.
 *
 * @code{.yaml}
 * extensions:
 *   render:
 *     plugin: gaitos_mujoco/MujocoRenderer
 *     viewer: false              # window (needs a display and GLFW); else rendering is headless (EGL)
 *     viewer_size: [1280, 800]
 *     viewer_fps: 30.0           # [Hz] wall time
 *     viewer_track: pelvis       # body the viewer camera follows ("": free camera). Mouse: left
 *                                # rotate, right move, wheel zoom; T toggles tracking
 *     cameras:                   # any number, by name
 *       chest:
 *         body: torso_link         # body the camera is fixed to
 *         xyz: [0.0, 0.0, 0.0]     # [m] optical frame in the body
 *         rpy: [0.0, 0.0, 0.0]     # [rad] optical frame in the body (fixed axes x, y, z)
 *         width: 640
 *         height: 480
 *         fy: 620.0                # [px] (fx = fy: square pixels)
 *         cx: 320.0
 *         cy: 240.0
 *         rate: 15.0               # [Hz] simulated time
 *         namespace: /camera       # topic prefix
 *         frame_id: camera_color_optical_frame
 *         depth: true
 *         min_depth: 0.1           # [m] nearer or farther: 0 (no data)
 *         max_depth: 10.0
 *         tf_parent: ""            # if set, static TF tf_parent -> frame_id (tf_xyz, tf_rpy)
 *         tf_xyz: [0.0, 0.0, 0.0]
 *         tf_rpy: [0.0, 0.0, 0.0]
 * @endcode
 */
class MujocoRenderer : public gaitos::NodeExtension
{
public:
  ~MujocoRenderer() override;

  void configure(
    rclcpp_lifecycle::LifecycleNode & node, gaitos::Gaitos & core,
    const gaitos::Params & params) override;
  void activate() override;
  void deactivate() override;

  struct Camera
  {
    std::string name;
    int body{-1};
    std::array<double, 3> xyz{};
    std::array<double, 9> rotation{};   ///< body <- optical, row major
    int width{640};
    int height{480};
    double fy{500.0};
    double cx{320.0};
    double cy{240.0};
    double rate{15.0};
    double near{0.02};
    double far{50.0};
    bool depth{true};
    double min_depth{0.1};
    double max_depth{10.0};
    std::string frame_id;
    double next{0.0};                   ///< simulated time of the next image
    double last{-1.0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
  };

  /// Depth buffer read with mjDEPTH_ZEROFAR -> distance along the optical axis [m], 0: none
  static double linear_depth(float buffer, double near, double far);
  /// Pose of the OpenGL camera of `camera` (MuJoCo: forward, up) from the body pose
  static void camera_pose(
    const Camera & camera, const mjtNum * body_pos, const mjtNum * body_mat,
    float pos[3], float forward[3], float up[3]);

private:
  void run();
  bool init_gl();
  void render_camera(Camera & camera, double time);
  void render_viewer();
  void release_gl();

  MujocoHardware * hardware_{nullptr};
  mjModel * model_{nullptr};          ///< own copy (larger offscreen buffer)
  mjData * data_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("render")};

  bool viewer_{false};
  std::array<int, 2> viewer_size_{1280, 800};
  double viewer_fps_{30.0};
  std::string viewer_track_;
  std::vector<Camera> cameras_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
  tf2_msgs::msg::TFMessage tf_static_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  // GL state, only touched in the render thread
  struct Gl;
  std::unique_ptr<Gl> gl_;
};

}  // namespace gaitos_mujoco

#endif  // GAITOS_MUJOCO__MUJOCORENDERER_HPP_
