/**
 * DUA PCL
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

#include <dua_math/dua_math.hpp>
#include <dua_pcl/dua_pcl_struct.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dua_pcl
{

template<typename PointT>
inline void resize_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  std::size_t new_size)
{
  if (!cloud) {
    return;
  }
  cloud->resize(new_size);
  cloud->width = static_cast<uint32_t>(new_size);
  cloud->height = 1;
}

template<typename PointT>
[[nodiscard]] inline bool is_finite(const PointT & point) noexcept
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

template<typename PointT>
[[nodiscard]] inline bool is_within_range_sq(
  const PointT & point,
  float min_range_sq, float max_range_sq) noexcept
{
  const float radius_sq = point.x * point.x + point.y * point.y + point.z * point.z;
  return radius_sq >= min_range_sq && radius_sq <= max_range_sq;
}

template<typename PointT>
[[nodiscard]] inline bool is_within_box(
  const PointT & point,
  float half_len_x, float half_len_y, float half_len_z) noexcept
{
  return -half_len_x <= point.x && point.x <= half_len_x &&
         -half_len_y <= point.y && point.y <= half_len_y &&
         -half_len_z <= point.z && point.z <= half_len_z;
}

template<typename PointT>
[[nodiscard]] inline bool is_within_fov(
  const PointT & point,
  float min_azim, float max_azim, float off_azim,
  float min_elev, float max_elev, float off_elev) noexcept
{
  const float azim = dua_math::normalize_angle(std::atan2(point.y, point.x) - off_azim);
  if (azim < min_azim || azim > max_azim) {
    return false;
  }

  const float planar = std::hypot(point.x, point.y);
  const float elev = dua_math::normalize_angle(std::atan2(point.z, planar) - off_elev);
  if (elev < min_elev || elev > max_elev) {
    return false;
  }

  return true;
}

template<typename PointT>
inline void transform_point(
  PointT & point,
  const Eigen::Matrix3f & R, const Eigen::Vector3f & t) noexcept
{
  const float x = point.x;
  const float y = point.y;
  const float z = point.z;

  point.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t(0);
  point.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t(1);
  point.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t(2);
}

template<typename PointT>
void DUA_PCL_PUBLIC clean_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  if (!cloud || cloud->empty() || cloud->is_dense) {
    return;
  }

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    if (is_finite(point)) {
      cloud->points[idx++] = point;
    }
  }

  resize_cloud<PointT>(cloud, idx);
  cloud->is_dense = true;
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_sphere_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropSphereParams & crop_sphere_params)
{
  if (!cloud || cloud->empty() || !crop_sphere_params.do_crop_sphere) {
    return;
  }

  const float min_range_sq = crop_sphere_params.min_range * crop_sphere_params.min_range;
  const float max_range_sq = crop_sphere_params.max_range * crop_sphere_params.max_range;

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    if (is_within_range_sq(point, min_range_sq, max_range_sq)) {
      cloud->points[idx++] = point;
    }
  }

  resize_cloud<PointT>(cloud, idx);
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_box_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropBoxParams & crop_box_params)
{
  if (!cloud || cloud->empty() || !crop_box_params.do_crop_box) {
    return;
  }

  const float half_len_x = crop_box_params.half_len_x;
  const float half_len_y = crop_box_params.half_len_y;
  const float half_len_z = crop_box_params.half_len_z;

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    if (is_within_box(point, half_len_x, half_len_y, half_len_z)) {
      cloud->points[idx++] = point;
    }
  }

  resize_cloud<PointT>(cloud, idx);
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_fov_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropFovParams & crop_fov_params)
{
  if (!cloud || cloud->empty() || !crop_fov_params.do_crop_fov) {
    return;
  }

  const float min_azim = crop_fov_params.min_azim;
  const float max_azim = crop_fov_params.max_azim;
  const float off_azim = crop_fov_params.off_azim;
  const float min_elev = crop_fov_params.min_elev;
  const float max_elev = crop_fov_params.max_elev;
  const float off_elev = crop_fov_params.off_elev;

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    if (is_within_fov(point,
          min_azim, max_azim, off_azim,
          min_elev, max_elev, off_elev))
    {
      cloud->points[idx++] = point;
    }
  }

  resize_cloud<PointT>(cloud, idx);
}

template<typename PointT>
void DUA_PCL_PUBLIC transform_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const TransformParams & transform_params)
{
  if (!cloud || cloud->empty() || !transform_params.do_transform) {
    return;
  }

  const pose_kit::Pose & pose = transform_params.pose;
  pose.get_frame_id(cloud->header.frame_id);
  cloud->header.stamp = pose.get_timestamp_us();

  Eigen::Isometry3d iso;
  pose.get_isometry(iso);
  Eigen::Matrix4f T = iso.matrix().cast<float>();
  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  Eigen::Vector3f t = T.block<3, 1>(0, 3);

  for (auto & point : cloud->points) {
    transform_point(point, R, t);
  }
}

template<typename PointT>
void DUA_PCL_PUBLIC downsample_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const DownsampleParams & downsample_params)
{
  if (!cloud || cloud->empty() || !downsample_params.do_downsample) {
    return;
  }

  const float leaf_size = downsample_params.leaf_size;
  if (leaf_size <= 0.0f) {
    return;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setMinimumPointsNumberPerVoxel(downsample_params.min_points);
  voxel.setInputCloud(cloud);

  pcl::PointCloud<PointT> tmp;
  voxel.filter(tmp);
  cloud->swap(tmp);
}

template<typename PointT>
void DUA_PCL_PUBLIC remove_ground(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const RemoveGroundParams & remove_ground_params)
{
  if (!cloud || cloud->empty() || !remove_ground_params.do_remove_ground) {
    return;
  }

  const float dist_thr = remove_ground_params.dist_thr;
  const float eps_angle = remove_ground_params.eps_angle;

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    const float radius = std::hypot(point.x, point.y);
    const float thr = dist_thr + radius * std::tan(eps_angle);
    if (point.z > thr) {
      cloud->points[idx++] = point;
    }
  }

  resize_cloud<PointT>(cloud, idx);
}

template<typename PointT>
void DUA_PCL_PUBLIC preprocess_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const PreprocessParams & params)
{
  if (!cloud || cloud->empty()) {
    return;
  }

  const bool do_clean = !cloud->is_dense;
  const bool do_crop_sphere = params.crop_sphere_params.do_crop_sphere;
  const bool do_crop_box = params.crop_box_params.do_crop_box;
  const bool do_crop_fov = params.crop_fov_params.do_crop_fov;
  const bool do_transform = params.transform_params.do_transform;

  const float min_range_sq =
    params.crop_sphere_params.min_range * params.crop_sphere_params.min_range;
  const float max_range_sq =
    params.crop_sphere_params.max_range * params.crop_sphere_params.max_range;

  const float half_len_x = params.crop_box_params.half_len_x;
  const float half_len_y = params.crop_box_params.half_len_y;
  const float half_len_z = params.crop_box_params.half_len_z;

  const float min_azim = params.crop_fov_params.min_azim;
  const float max_azim = params.crop_fov_params.max_azim;
  const float off_azim = params.crop_fov_params.off_azim;
  const float min_elev = params.crop_fov_params.min_elev;
  const float max_elev = params.crop_fov_params.max_elev;
  const float off_elev = params.crop_fov_params.off_elev;

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f::Zero();
  if (do_transform) {
    const pose_kit::Pose & pose = params.transform_params.pose;
    pose.get_frame_id(cloud->header.frame_id);
    cloud->header.stamp = pose.get_timestamp_us();

    Eigen::Isometry3d iso;
    pose.get_isometry(iso);
    Eigen::Matrix4f T = iso.matrix().cast<float>();
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
  }

  std::size_t idx = 0;
  for (const auto & point : cloud->points) {
    if (do_clean && !is_finite(point)) {
      continue;
    }

    if (do_crop_sphere &&
      !is_within_range_sq(point, min_range_sq, max_range_sq))
    {
      continue;
    }

    if (do_crop_box &&
      !is_within_box(point, half_len_x, half_len_y, half_len_z))
    {
      continue;
    }

    if (do_crop_fov &&
      !is_within_fov(point,
        min_azim, max_azim, off_azim,
        min_elev, max_elev, off_elev))
    {
      continue;
    }

    cloud->points[idx] = point;
    if (do_transform) {
      transform_point(cloud->points[idx], R, t);
    }
    idx++;
  }

  resize_cloud<PointT>(cloud, idx);
  cloud->is_dense = true;

  if (params.downsample_params.do_downsample && !cloud->empty()) {
    dua_pcl::downsample_cloud<PointT>(cloud, params.downsample_params);
  }

  if (params.remove_ground_params.do_remove_ground && !cloud->empty()) {
    dua_pcl::remove_ground<PointT>(cloud, params.remove_ground_params);
  }
}

} // namespace dua_pcl
