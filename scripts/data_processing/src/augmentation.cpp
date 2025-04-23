#include "augmentation.h"
#include "downsample.h"

#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>
#include <Eigen/Dense>

bool augmentPointCloud2(const pcl::PCLPointCloud2::Ptr& input,
                        pcl::PCLPointCloud2::Ptr& output,
                        bool apply_rotation,
                        bool apply_translation,
                        bool apply_scale,
                        bool apply_noise,
                        float downsample_voxel_size)
{
    std::default_random_engine gen(std::random_device{}());
    std::uniform_real_distribution<float> rot_dist(0, 2 * M_PI);
    std::uniform_real_distribution<float> trans_dist(-0.5f, 0.5f);  // ±50 cm
    std::uniform_real_distribution<float> scale_dist(0.95f, 1.05f); // ±5%
    std::normal_distribution<float> noise_dist(0.0f, 0.01f);         // 1 cm std dev

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    if (apply_rotation) {
        float angle = rot_dist(gen);
        transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    }

    if (apply_translation) {
        transform.translation() << trans_dist(gen), trans_dist(gen), trans_dist(gen);
    }

    if (apply_scale) {
        float scale = scale_dist(gen);
        transform.scale(scale);
    }

    // Převod na PointXYZ pro transformaci souřadnic
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input, *xyz_cloud);

    // Aplikace transformace
    pcl::transformPointCloud(*xyz_cloud, *xyz_cloud, transform);

    if (apply_noise) {
        for (auto& pt : xyz_cloud->points) {
            pt.x += noise_dist(gen);
            pt.y += noise_dist(gen);
            pt.z += noise_dist(gen);
        }
    }

    // Převod zpět na PCLPointCloud2 pouze XYZ
    pcl::PCLPointCloud2 temp_xyz;
    pcl::toPCLPointCloud2(*xyz_cloud, temp_xyz);

    // Zkopíruj celé původní mračno (včetně všech polí)
    *output = *input;

    // Přepiš pouze x, y, z hodnoty
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : output->fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
        std::cerr << "❌ x/y/z field not found in point cloud." << std::endl;
        return false;
    }

    const size_t point_step = output->point_step;
    const size_t num_points = output->width * output->height;

    for (size_t i = 0; i < num_points; ++i) {
        std::memcpy(&output->data[i * point_step + x_offset], &xyz_cloud->points[i].x, sizeof(float));
        std::memcpy(&output->data[i * point_step + y_offset], &xyz_cloud->points[i].y, sizeof(float));
        std::memcpy(&output->data[i * point_step + z_offset], &xyz_cloud->points[i].z, sizeof(float));
    }

    // Downsampling
    if (downsample_voxel_size > 0.0f) {
        pcl::PCLPointCloud2::Ptr downsampled(new pcl::PCLPointCloud2);
        downsamplePointCloud2(output, downsampled, downsample_voxel_size);
        *output = *downsampled;
    }

    return true;
}
