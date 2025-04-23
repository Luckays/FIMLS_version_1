#ifndef AUGMENTATION_H
#define AUGMENTATION_H

#include <pcl/PCLPointCloud2.h>
#include <memory>

/**
 * Apply geometric augmentations to a point cloud
 * (rotation, translation, scale, noise, optional downsampling)
 * 
 * @param input Input PCLPointCloud2
 * @param output Augmented output
 * @param apply_rotation Apply random Z rotation
 * @param apply_translation Apply random translation
 * @param apply_scale Apply random scale
 * @param apply_noise Add Gaussian jitter
 * @param downsample_voxel_size Optional voxel grid size (0 = no downsampling)
 * @return true if successful
 */
bool augmentPointCloud2(const pcl::PCLPointCloud2::Ptr& input,
                        pcl::PCLPointCloud2::Ptr& output,
                        bool apply_rotation = true,
                        bool apply_translation = true,
                        bool apply_scale = true,
                        bool apply_noise = true,
                        float downsample_voxel_size = 0.0f);
#endif // AUGMENTATION_H