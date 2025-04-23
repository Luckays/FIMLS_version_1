#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H

#include <pcl/PCLPointCloud2.h>
#include <memory>

/**
 * Downsample point cloud using VoxelGrid filter
 * @param input Input PCLPointCloud2
 * @param output Downsampled output cloud
 * @param leaf_size Size of voxel grid (in meters), e.g. 0.01 = 1 cm
 * @return true if successful
 */
bool downsamplePointCloud2(const pcl::PCLPointCloud2::Ptr& input,
                           pcl::PCLPointCloud2::Ptr& output,
                           float leaf_size);

#endif // DOWNSAMPLE_H
