/**
 * DUA PCL APIs.
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

#include <dua_pcl/dua_pcl_struct.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace dua_pcl
{

template<typename PointT>
void DUA_PCL_PUBLIC clean_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  if (!cloud || cloud->empty() || cloud->is_dense) {
    return;
  }

  size_t idx = 0;
  for (const auto & point : *cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      (*cloud)[idx++] = point;
    }
  }
  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_sphere_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropSphereParams & params)
{
  if (!cloud || cloud->empty() || !params.do_crop_sphere) {
    return;
  }

  const float min_radius = params.min_radius;
  const float max_radius = params.max_radius;
  const float min_radius_sq = min_radius * min_radius;
  const float max_radius_sq = max_radius * max_radius;

  size_t idx = 0;
  for (const auto & point : *cloud) {
    const float radius_sq = point.x * point.x + point.y * point.y + point.z * point.z;
    if (radius_sq >= min_radius_sq && radius_sq <= max_radius_sq) {
      (*cloud)[idx++] = point;
    }
  }
  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_box_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropBoxParams & params)
{
  if (!cloud || cloud->empty() || !params.do_crop_box) {
    return;
  }

  const float half_len_x = params.half_len_x;
  const float half_len_y = params.half_len_y;
  const float half_len_z = params.half_len_z;

  size_t idx = 0;
  for (const auto & point : *cloud) {
    if (std::abs(point.x) <= half_len_x &&
      std::abs(point.y) <= half_len_y &&
      std::abs(point.z) <= half_len_z)
    {
      (*cloud)[idx++] = point;
    }
  }
  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

template<typename PointT>
void DUA_PCL_PUBLIC crop_angular_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const CropAngularParams & params)
{
  if (!cloud || cloud->empty() || !params.do_crop_angular) {
    return;
  }

  const float min_elev_rad = params.min_elev * M_PI / 180.0f;
  const float max_elev_rad = params.max_elev * M_PI / 180.0f;
  const float min_azim_rad = params.min_azim * M_PI / 180.0f;
  const float max_azim_rad = params.max_azim * M_PI / 180.0f;

  size_t idx = 0;
  for (const auto & point : *cloud) {
    const float azim = std::atan2(point.y, point.x);
    if (azim < min_azim_rad || azim > max_azim_rad) {
      continue;
    }
    const float planar = std::hypot(point.x, point.y);
    const float elev = std::atan2(point.z, planar);
    if (elev < min_elev_rad || elev > max_elev_rad) {
      continue;
    }
    (*cloud)[idx++] = point;
  }
  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;
}

template<typename PointT>
void DUA_PCL_PUBLIC transform_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const TransformParams & params)
{
  if (!cloud || cloud->empty() || !params.do_transform) {
    return;
  }

  const pose_kit::Pose & pose = params.pose;
  pose.get_frame_id(cloud->header.frame_id);
  cloud->header.stamp = pose.get_timestamp_us();

  Eigen::Isometry3d iso;
  pose.get_isometry(iso);
  Eigen::Matrix4f T = iso.matrix().cast<float>();
  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  Eigen::Vector3f t = T.block<3, 1>(0, 3);
  for (auto & point : *cloud) {
    const float x = point.x;
    const float y = point.y;
    const float z = point.z;
    point.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t(0);
    point.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t(1);
    point.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t(2);
  }
}

template<typename PointT>
void DUA_PCL_PUBLIC downsample_cloud(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const DownsampleParams & params)
{
  if (!cloud || cloud->empty() || !params.do_downsample) {
    return;
  }

  const float leaf_size = params.leaf_size;
  if (leaf_size <= 0.0f) {
    return;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setMinimumPointsNumberPerVoxel(params.min_points_per_voxel);
  voxel.setInputCloud(cloud);
  pcl::PointCloud<PointT> tmp;
  voxel.filter(tmp);
  cloud->swap(tmp);
}

template<typename PointT>
void DUA_PCL_PUBLIC remove_ground(
  typename pcl::PointCloud<PointT>::Ptr cloud,
  const RemoveGroundParams & params)
{
  if (!cloud || cloud->empty() || !params.do_remove_ground || cloud->size() <= 100) {
    return;
  }

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr ground(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(params.eps_angle * M_PI / 180.0f);
  seg.setDistanceThreshold(params.distance_threshold);
  seg.setMaxIterations(100);
  seg.setProbability(0.99);
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(cloud);
  seg.segment(*ground, *coeffs);
  if (!ground->indices.empty()) {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.setNegative(true);
    pcl::PointCloud<PointT> tmp;
    extract.filter(tmp);
    cloud->swap(tmp);
  }
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
  const float min_radius = params.crop_sphere_params.min_radius;
  const float max_radius = params.crop_sphere_params.max_radius;
  const float min_radius_sq = min_radius * min_radius;
  const float max_radius_sq = max_radius * max_radius;

  const bool do_crop_box = params.crop_box_params.do_crop_box;
  const float half_len_x = params.crop_box_params.half_len_x;
  const float half_len_y = params.crop_box_params.half_len_y;
  const float half_len_z = params.crop_box_params.half_len_z;

  const bool do_crop_angular = params.crop_angular_params.do_crop_angular;
  const float min_elev_rad = params.crop_angular_params.min_elev * M_PI / 180.0f;
  const float max_elev_rad = params.crop_angular_params.max_elev * M_PI / 180.0f;
  const float min_azim_rad = params.crop_angular_params.min_azim * M_PI / 180.0f;
  const float max_azim_rad = params.crop_angular_params.max_azim * M_PI / 180.0f;

  const bool do_transform = params.transform_params.do_transform;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
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

  size_t idx = 0;
  for (const auto & point : cloud->points) {
    const float x = point.x;
    const float y = point.y;
    const float z = point.z;

    if (do_clean) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
    }

    if (do_crop_sphere) {
      const float radius_sq = x * x + y * y + z * z;
      if (radius_sq < min_radius_sq || radius_sq > max_radius_sq) {
        continue;
      }
    }

    if (do_crop_box) {
      if (std::abs(x) > half_len_x || std::abs(y) > half_len_y || std::abs(z) > half_len_z) {
        continue;
      }
    }

    if (do_crop_angular) {
      const float azim = std::atan2(y, x);
      if (azim < min_azim_rad || azim > max_azim_rad) {
        continue;
      }
      const float planar = std::hypot(x, y);
      const float elev = std::atan2(z, planar);
      if (elev < min_elev_rad || elev > max_elev_rad) {
        continue;
      }
    }

    auto & tmp = (*cloud)[idx++];
    tmp = point;
    if (do_transform) {
      tmp.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t(0);
      tmp.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t(1);
      tmp.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t(2);
    }
  }
  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;

  if (params.downsample_params.do_downsample && !cloud->empty()) {
    dua_pcl::downsample_cloud<PointT>(cloud, params.downsample_params);
  }

  if (params.ground_params.do_remove_ground && cloud->size() > 100) {
    dua_pcl::remove_ground<PointT>(cloud, params.ground_params);
  }
}

} // namespace dua_pcl
