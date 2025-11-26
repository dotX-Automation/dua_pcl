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
  float min_radius = 0.0f;  // [m]
  float max_radius = 0.0f;  // [m]
};

struct DUA_PCL_PUBLIC CropBoxParams
{
  bool do_crop_box = false;
  float half_len_x = 0.0f;  // [m]
  float half_len_y = 0.0f;  // [m]
  float half_len_z = 0.0f;  // [m]
};

struct DUA_PCL_PUBLIC CropAngularParams
{
  bool do_crop_angular = false;
  float min_elev = 0.0f;  // [deg]
  float max_elev = 0.0f;  // [deg]
  float min_azim = 0.0f;  // [deg]
  float max_azim = 0.0f;  // [deg]
};

struct DUA_PCL_PUBLIC TransformParams
{
  bool do_transform = false;
  pose_kit::Pose pose = pose_kit::Pose();
};

struct DUA_PCL_PUBLIC DownsampleParams
{
  bool do_downsample = false;
  float leaf_size = 0.0f;  // [m]
  size_t min_points_per_voxel = 1u;
};

struct DUA_PCL_PUBLIC RemoveGroundParams
{
  bool do_remove_ground = false;
  float eps_angle = 0.0f;  // [deg]
  float distance_threshold = 0.0f;  // [m]
};

struct DUA_PCL_PUBLIC PreprocessParams
{
  CropSphereParams   crop_sphere_params;
  CropBoxParams      crop_box_params;
  CropAngularParams  crop_angular_params;
  TransformParams    transform_params;
  DownsampleParams   downsample_params;
  RemoveGroundParams ground_params;
};

} // namespace dua_pcl
