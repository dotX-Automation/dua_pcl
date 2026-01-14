/**
 * DUA PCL structs
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

#include <cstddef>
#include <dua_pcl/visibility_control.h>
#include <pose_kit/pose.hpp>

namespace dua_pcl
{

struct DUA_PCL_PUBLIC CropSphereParams
{
  bool do_crop_sphere = false;
  float min_range = 0.0f; // [m]
  float max_range = 0.0f; // [m]

  CropSphereParams(
    bool do_crop_sphere = false,
    float min_range = 0.0f,
    float max_range = 0.0f)
  : do_crop_sphere(do_crop_sphere),
    min_range(min_range),
    max_range(max_range)
  {
    validate();
  }

  void validate() const
  {
    if (do_crop_sphere) {
      if (min_range < 0.0f) {
        throw std::invalid_argument("CropSphereParams: min_range must be >= 0.");
      }
      if (max_range <= 0.0f) {
        throw std::invalid_argument("CropSphereParams: max_range must be > 0.");
      }
      if (max_range <= min_range) {
        throw std::invalid_argument("CropSphereParams: max_range must be > min_range.");
      }
    }
  }

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
  float half_len_x = 0.0f; // [m]
  float half_len_y = 0.0f; // [m]
  float half_len_z = 0.0f; // [m]

  CropBoxParams(
    bool do_crop_box = false,
    float half_len_x = 0.0f,
    float half_len_y = 0.0f,
    float half_len_z = 0.0f)
  : do_crop_box(do_crop_box),
    half_len_x(half_len_x),
    half_len_y(half_len_y),
    half_len_z(half_len_z)
  {
    validate();
  }

  void validate() const
  {
    if (do_crop_box) {
      if (half_len_x <= 0.0f) {
        throw std::invalid_argument("CropBoxParams: half_len_x must be > 0.");
      }
      if (half_len_y <= 0.0f) {
        throw std::invalid_argument("CropBoxParams: half_len_y must be > 0.");
      }
      if (half_len_z <= 0.0f) {
        throw std::invalid_argument("CropBoxParams: half_len_z must be > 0.");
      }
    }
  }

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
  float min_azim = 0.0f;  // [rad]
  float max_azim = 0.0f;  // [rad]
  float off_azim = 0.0f;  // [rad]
  float step_azim = 0.0f; // [rad]
  float min_elev = 0.0f;  // [rad]
  float max_elev = 0.0f;  // [rad]
  float off_elev = 0.0f;  // [rad]
  float step_elev = 0.0f; // [rad]

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
  {
    validate();
  }

  void validate() const
  {
    if (do_crop_fov) {
      if (min_azim < -M_PI || min_azim > M_PI) {
        throw std::invalid_argument("CropFovParams: min_azim must be in [-180, 180] deg.");
      }
      if (max_azim < -M_PI || max_azim > M_PI) {
        throw std::invalid_argument("CropFovParams: max_azim must be in [-180, 180] deg.");
      }
      if (max_azim <= min_azim) {
        throw std::invalid_argument("CropFovParams: max_azim must be > min_azim.");
      }
      if (off_azim < -M_PI || off_azim > M_PI) {
        throw std::invalid_argument("CropFovParams: off_azim must be in [-180, 180] deg.");
      }
      if (step_azim < 0.0f) {
        throw std::invalid_argument("CropFovParams: step_azim must be >= 0.");
      }
      if (min_elev < -M_PI || min_elev > M_PI) {
        throw std::invalid_argument("CropFovParams: min_elev must be in [-180, 180] deg.");
      }
      if (max_elev < -M_PI || max_elev > M_PI) {
        throw std::invalid_argument("CropFovParams: max_elev must be in [-180, 180] deg.");
      }
      if (max_elev <= min_elev) {
        throw std::invalid_argument("CropFovParams: max_elev must be > min_elev.");
      }
      if (off_elev < -M_PI || off_elev > M_PI) {
        throw std::invalid_argument("CropFovParams: off_elev must be in [-180, 180] deg.");
      }
      if (step_elev < 0.0f) {
        throw std::invalid_argument("CropFovParams: step_elev must be >= 0.");
      }
    }
  }

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_crop_fov) {
      oss << "- CropFovParams:\n"
          << "  - Azim (deg): "
          << dua_math::rad_to_deg(off_azim) << " + ["
          << dua_math::rad_to_deg(min_azim) << ", "
          << dua_math::rad_to_deg(max_azim) << "] : "
          << dua_math::rad_to_deg(step_azim) << "\n"
          << "  - Elev (deg): "
          << dua_math::rad_to_deg(off_elev) << " + ["
          << dua_math::rad_to_deg(min_elev) << ", "
          << dua_math::rad_to_deg(max_elev) << "] : "
          << dua_math::rad_to_deg(step_elev);
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
  {
    validate();
  }

  void validate() const
  {
    if (do_transform) {
      if (frame_id.empty()) {
        throw std::invalid_argument("TransformParams: frame_id must be non-empty.");
      }
    }
  }

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_transform) {
      Eigen::Vector3d pos;
      pose.get_position(pos);
      Eigen::Vector3d rpy;
      pose.get_rpy(rpy);
      oss << "- TransformParams:\n"
          << "  - Frame ID: " << frame_id << "\n"
          << "  - Pose: " << pose.header().frame_id << " @ "
          << "xyz: ("
          << pos.x() << ", "
          << pos.y() << ", "
          << pos.z() << "), "
          << "rpy: ["
          << dua_math::rad_to_deg(rpy.x()) << ", "
          << dua_math::rad_to_deg(rpy.y()) << ", "
          << dua_math::rad_to_deg(rpy.z()) << "]";
    } else {
      oss << "- TransformParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC DownsampleParams
{
  bool do_downsample = false;
  float leaf_size = 0.0f; // [m]
  size_t min_points = 1u;

  DownsampleParams(
    bool do_downsample = false,
    float leaf_size = 0.0f,
    size_t min_points = 1u)
  : do_downsample(do_downsample),
    leaf_size(leaf_size),
    min_points(min_points)
  {
    validate();
  }

  void validate() const
  {
    if (do_downsample) {
      if (leaf_size <= 0.0f) {
        throw std::invalid_argument("DownsampleParams: leaf_size must be > 0.");
      }
      if (min_points == 0u) {
        throw std::invalid_argument("DownsampleParams: min_points must be > 0.");
      }
    }
  }

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_downsample) {
      oss << "- DownsampleParams:\n"
          << "  - Leaf size (m): " << leaf_size << "\n"
          << "  - Min points per voxel: " << min_points;
    } else {
      oss << "- DownsampleParams: False";
    }
    return oss.str();
  }
};

struct DUA_PCL_PUBLIC RemoveGroundParams
{
  bool do_remove_ground = false;
  float eps_angle = 0.0f;  // [rad]
  float dist_thres = 0.0f; // [m]

  RemoveGroundParams(
    bool do_remove_ground = false,
    float eps_angle = 0.0f,
    float dist_thres = 0.0f)
  : do_remove_ground(do_remove_ground),
    eps_angle(eps_angle),
    dist_thres(dist_thres)
  {
    validate();
  }

  void validate() const
  {
    if (do_remove_ground) {
      if (eps_angle < 0.0f || eps_angle > M_PI_2) {
        throw std::invalid_argument("RemoveGroundParams: eps_angle must be in [0, 90] deg.");
      }
      if (dist_thres <= 0.0f) {
        throw std::invalid_argument("RemoveGroundParams: dist_thres must be > 0.");
      }
    }
  }

  [[nodiscard]] std::string to_string() const
  {
    std::ostringstream oss;
    if (do_remove_ground) {
      oss << "- RemoveGroundParams:\n"
          << "  - Eps angle (deg): " << dua_math::rad_to_deg(eps_angle) << "\n"
          << "  - Distance threshold (m): " << dist_thres;
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
