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

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dua_pcl
{

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
  const float box_len_x = params.crop_box_params.len_x;
  const float box_len_y = params.crop_box_params.len_y;
  const float box_len_z = params.crop_box_params.len_z;

  const bool do_crop_angular = params.crop_angular_params.do_crop_angular;
  const float min_elev = params.crop_angular_params.min_elevation_angle;
  const float max_elev = params.crop_angular_params.max_elevation_angle;
  const float min_azim = params.crop_angular_params.min_azimuth_angle;
  const float max_azim = params.crop_angular_params.max_azimuth_angle;

  const bool do_transform = params.transform_params.do_transform;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  if (do_transform) {
    Eigen::Matrix4f T = params.transform_params.transform.matrix().cast<float>();
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
  }

  const bool do_downsample = params.downsample_params.do_downsample;
  const float leaf_size = params.downsample_params.leaf_size;
  const size_t min_points_per_voxel = params.downsample_params.min_points_per_voxel;

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
      if (std::abs(x) > box_len_x || std::abs(y) > box_len_y || std::abs(z) > box_len_z) {
        continue;
      }
    }

    if (do_crop_angular) {
      const float azim = std::atan2(y, x);
      if (azim < min_azim || azim > max_azim) {
        continue;
      }
      const float planar = std::sqrt(x * x + y * y);
      const float elev = std::atan2(z, planar);
      if (elev < min_elev || elev > max_elev) {
        continue;
      }
    }

    float tx = x, ty = y, tz = z;
    if (do_transform) {
      tx = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t(0);
      ty = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t(1);
      tz = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t(2);
    }

    (*cloud)[idx].x = tx;
    (*cloud)[idx].y = ty;
    (*cloud)[idx].z = tz;
    ++idx;
  }

  cloud->resize(idx);
  cloud->width = static_cast<uint32_t>(idx);
  cloud->height = 1;
  cloud->is_dense = true;

  if (do_downsample) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
    voxel.setInputCloud(cloud);
    pcl::PointCloud<PointT> tmp;
    voxel.filter(tmp);
    cloud->swap(tmp);
  }
}

/**
 * @brief Removes NaN or infinite points from the point cloud.
 * @param cloud Point cloud to process.
 */
template<typename PointT>
void DUA_PCL_PUBLIC clean_cloud(pcl::PointCloud<PointT> & cloud)
{
  // Check if the point cloud is dense
  if (cloud.is_dense) {
    return;
  }

  // Remove NaN or infinite points
  size_t idx = 0;
  for (const auto & point : cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      cloud[idx++] = point;
    }
  }

  // Update the point cloud information
  cloud.resize(idx);
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

/**
 * @brief Removes points outside the specified range from the point cloud.
 * @param cloud Point cloud to process.
 * @param len_x Maximum length along the x-axis.
 * @param len_y Maximum length along the y-axis.
 * @param len_z Maximum length along the z-axis.
 */
template<typename PointT>
void DUA_PCL_PUBLIC crop_box_cloud(
  pcl::PointCloud<PointT> & cloud, float len_x, float len_y,
  const float len_z)
{
  // Check if the point cloud is empty
  if (cloud.empty()) {
    return;
  }

  // Remove points outside the box
  size_t idx = 0;
  for (const auto & point : cloud) {
    if (std::abs(point.x) <= len_x && std::abs(point.y) <= len_y && std::abs(point.z) <= len_z) {
      cloud[idx++] = point;
    }
  }

  // Update the point cloud information
  cloud.resize(idx);
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

/**
 * @brief Removes points outside the specified sphere from the point cloud.
 * @param cloud Point cloud to process.
 * @param radius Sphere radius.
 */
template<typename PointT>
void DUA_PCL_PUBLIC crop_sphere_cloud(pcl::PointCloud<PointT> & cloud, float radius)
{
  // Check if the point cloud is empty
  if (cloud.empty()) {
    return;
  }

  // Remove points outside the sphere
  size_t idx = 0;
  float radius_sq = radius * radius;
  for (const auto & point : cloud) {
    if ((point.x * point.x + point.y * point.y + point.z * point.z) <= radius_sq) {
      cloud[idx++] = point;
    }
  }

  // Update the point cloud information
  cloud.resize(idx);
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

/**
 * @brief Downsamples the point cloud using a voxel grid filter.
 * @param cloud Point cloud to process.
 * @param leaf_size Voxel grid leaf size.
 * @param min_points_per_voxel Minimum number of points per voxel.
 */
template<typename PointT>
void DUA_PCL_PUBLIC downsample_cloud(
  pcl::PointCloud<PointT> & cloud, float leaf_size,
  int min_points_per_voxel)
{
  // Check if the point cloud is empty
  if (cloud.empty()) {
    return;
  }

  // Downsample the point cloud
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud.makeShared());
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
  voxel_filter.filter(cloud);
}

template<typename PointT>
void DUA_PCL_PUBLIC remove_ground(pcl::PointCloud<PointT> & cloud)
{
  // Check if the point cloud is empty
  if (cloud.empty()) {
    return;
  }

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr ground(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1f);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(20.0f * M_PI / 180.0f);
  seg.setInputCloud(cloud.makeShared());
  seg.segment(*ground, *coefficients);
  if (ground->indices.empty()) {
    return;
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(ground);
  extract.setNegative(true);

  pcl::PointCloud<PointT> filtered_cloud;
  extract.filter(filtered_cloud);
  cloud.swap(filtered_cloud);
}

} // namespace dua_pcl
