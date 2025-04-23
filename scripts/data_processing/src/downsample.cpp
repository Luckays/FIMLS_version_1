#include "downsample.h"

#include <pcl/filters/voxel_grid.h>

bool downsamplePointCloud2(const pcl::PCLPointCloud2::Ptr& input,
                           pcl::PCLPointCloud2::Ptr& output,
                           float leaf_size)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*output);
    return true;
}
