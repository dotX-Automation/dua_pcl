/**
 * DUA PCL utilities functions.
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

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dua_pcl
{

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
void DUA_PCL_PUBLIC crop_box_cloud(pcl::PointCloud<PointT> & cloud, float len_x, float len_y, const float len_z)
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
  double radius_sq = radius * radius;
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
void DUA_PCL_PUBLIC downsample_cloud(pcl::PointCloud<PointT> & cloud, float leaf_size, int min_points_per_voxel)
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
