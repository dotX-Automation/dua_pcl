/**
 * DUA PCL structs.
 *
 * dotX Automation <info@dotxautomation.com>
 *
 * November 19, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <dua_pcl/visibility_control.h>

#include <cstddef>
#include <pose_kit/pose.hpp>

namespace dua_pcl
{

struct DUA_PCL_PUBLIC CropSphereParams
{
  bool do_crop_sphere = false;
  float min_range = 0.0f;  // [m]
  float max_range = 0.0f;  // [m]

  CropSphereParams(
    bool do_crop_sphere = false,
    float min_range = 0.0f,
    float max_range = 0.0f)
  : do_crop_sphere(do_crop_sphere),
    min_range(min_range),
    max_range(max_range)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_crop_sphere) {
      oss << "- CropSphereParams:\n"
          << "  - Range (m): ["
          << min_range << ", "
          << max_range << "]";
    } else {
      oss << "- CropSphereParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC CropBoxParams
{
  bool do_crop_box = false;
  float half_len_x = 0.0f;  // [m]
  float half_len_y = 0.0f;  // [m]
  float half_len_z = 0.0f;  // [m]

  CropBoxParams(
    bool do_crop_box = false,
    float half_len_x = 0.0f,
    float half_len_y = 0.0f,
    float half_len_z = 0.0f)
  : do_crop_box(do_crop_box),
    half_len_x(half_len_x),
    half_len_y(half_len_y),
    half_len_z(half_len_z)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_crop_box) {
      oss << "- CropBoxParams:\n"
          << "  - Half len (m): ["
          << half_len_x << ", "
          << half_len_y << ", "
          << half_len_z << "]";
    } else {
      oss << "- CropBoxParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC CropFovParams
{
  bool do_crop_fov = false;
  float min_azim = 0.0f;  // [deg]
  float max_azim = 0.0f;  // [deg]
  float off_azim = 0.0f;  // [deg]
  float step_azim = 0.0f; // [deg]
  float min_elev = 0.0f;  // [deg]
  float max_elev = 0.0f;  // [deg]
  float off_elev = 0.0f;  // [deg]
  float step_elev = 0.0f; // [deg]

  CropFovParams(
    bool do_crop_fov = false,
    float min_azim = 0.0f,
    float max_azim = 0.0f,
    float off_azim = 0.0f,
    float step_azim = 0.0f,
    float min_elev = 0.0f,
    float max_elev = 0.0f,
    float off_elev = 0.0f,
    float step_elev = 0.0f)
  : do_crop_fov(do_crop_fov),
    min_azim(min_azim),
    max_azim(max_azim),
    off_azim(off_azim),
    step_azim(step_azim),
    min_elev(min_elev),
    max_elev(max_elev),
    off_elev(off_elev),
    step_elev(step_elev)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_crop_fov) {
      oss << "- CropFovParams:\n"
          << "  - Azim (deg): "
          << off_azim << " + ["
          << min_azim << ", "
          << max_azim << "] : "
          << step_azim << "\n"
          << "  - Elev (deg): "
          << off_elev << " + ["
          << min_elev << ", "
          << max_elev << "] : "
          << step_elev;
    } else {
      oss << "- CropFovParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC TransformParams
{
  bool do_transform = false;
  std::string frame_id = "";
  pose_kit::Pose pose = pose_kit::Pose();

  TransformParams(
    bool do_transform = false,
    const std::string & frame_id = "",
    const pose_kit::Pose & pose = pose_kit::Pose())
  : do_transform(do_transform),
    frame_id(frame_id),
    pose(pose)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_transform) {
      oss << "- TransformParams:\n"
          << "  - Frame ID: " << frame_id;
    } else {
      oss << "- TransformParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC DownsampleParams
{
  bool do_downsample = false;
  float leaf_size = 0.0f;  // [m]
  size_t min_points_per_voxel = 1u;

  DownsampleParams(
    bool do_downsample = false,
    float leaf_size = 0.0f,
    size_t min_points_per_voxel = 1u)
  : do_downsample(do_downsample),
    leaf_size(leaf_size),
    min_points_per_voxel(min_points_per_voxel)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_downsample) {
      oss << "- DownsampleParams:\n"
          << "  - Leaf size (m): " << leaf_size << "\n"
          << "  - Min points per voxel: " << min_points_per_voxel;
    } else {
      oss << "- DownsampleParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC RemoveGroundParams
{
  bool do_remove_ground = false;
  float eps_angle = 0.0f;  // [deg]
  float distance_threshold = 0.0f;  // [m]

  RemoveGroundParams(
    bool do_remove_ground = false,
    float eps_angle = 0.0f,
    float distance_threshold = 0.0f)
  : do_remove_ground(do_remove_ground),
    eps_angle(eps_angle),
    distance_threshold(distance_threshold)
  {}

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_remove_ground) {
      oss << "- RemoveGroundParams:\n"
          << "  - Eps angle (deg): " << eps_angle << "\n"
          << "  - Distance threshold (m): " << distance_threshold;
    } else {
      oss << "- RemoveGroundParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC PreprocessParams
{
  CropSphereParams crop_sphere_params;
  CropBoxParams crop_box_params;
  CropFovParams crop_fov_params;
  TransformParams transform_params;
  DownsampleParams downsample_params;
  RemoveGroundParams remove_ground_params;

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    oss << "PreprocessParams:\n"
        << crop_sphere_params.to_string() << "\n"
        << crop_box_params.to_string() << "\n"
        << crop_fov_params.to_string() << "\n"
        << transform_params.to_string() << "\n"
        << downsample_params.to_string() << "\n"
        << remove_ground_params.to_string();
    return oss.str();
  }
};

} // namespace dua_pcl
