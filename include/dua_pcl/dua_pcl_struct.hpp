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

#include <Eigen/Geometry>

namespace dua_pcl
{

struct DUA_PCL_PUBLIC TransformParams
{
  bool do_transform;
  Eigen::Isometry3d transform;

  TransformParams()
  : do_transform(false),
    transform(Eigen::Isometry3d::Identity())
  {}
};

struct DUA_PCL_PUBLIC CropSphereParams
{
  bool do_crop_sphere;
  double min_radius;
  double max_radius;

  CropSphereParams()
  : do_crop_sphere(false),
    min_radius(0.0f),
    max_radius(0.0f)
  {}
};

struct DUA_PCL_PUBLIC CropAngularParams
{
  bool do_crop_angular;
  double min_elevation_angle;
  double max_elevation_angle;
  double min_azimuth_angle;
  double max_azimuth_angle;

  CropAngularParams()
  : do_crop_angular(false),
    min_elevation_angle(0.0f),
    max_elevation_angle(0.0f),
    min_azimuth_angle(0.0f),
    max_azimuth_angle(0.0f)
  {}
};

struct DUA_PCL_PUBLIC CropBoxParams
{
  bool do_crop_box;
  double len_x;
  double len_y;
  double len_z;

  CropBoxParams()
  : do_crop_box(false),
    len_x(0.0f),
    len_y(0.0f),
    len_z(0.0f)
  {}
};

struct DUA_PCL_PUBLIC RemoveGroundParams
{
  bool do_remove_ground;

  RemoveGroundParams()
  : do_remove_ground(false)
  {}
};

struct DUA_PCL_PUBLIC DownsampleParams
{
  bool do_downsample;
  double leaf_size;
  int min_points_per_voxel;

  DownsampleParams()
  : do_downsample(false),
    leaf_size(0.0f),
    min_points_per_voxel(1)
  {}
};

struct DUA_PCL_PUBLIC PreprocessParams
{
  TransformParams transform_params;
  CropSphereParams crop_sphere_params;
  CropAngularParams crop_angular_params;
  CropBoxParams crop_box_params;
  RemoveGroundParams ground_params;
  DownsampleParams downsample_params;
};

} // namespace dua_pcl
