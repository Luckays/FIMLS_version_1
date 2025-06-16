#include "tile.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/conversions.h>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

bool tilePointCloud2(const pcl::PCLPointCloud2::Ptr& cloud,
                     float tile_size_x,
                     float tile_size_y,
                     float overlap,
                     const std::string& output_dir,
                     const std::string& base_filename)
{
    if (!cloud || cloud->width * cloud->height == 0) {
        std::cerr << "❌ Empty or null point cloud." << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *xyz_cloud);

    // Zjisti rozsah bodů
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);

    float min_x = min_pt.x, max_x = max_pt.x;
    float min_y = min_pt.y, max_y = max_pt.y;

    float step_x = tile_size_x - overlap;
    float step_y = tile_size_y - overlap;

    int tile_id = 0;
    fs::create_directories(output_dir);
    pcl::PCDWriter writer;

    std::ostringstream index_filename;
    index_filename << base_filename << "_tile_index.dat";
    std::ofstream index_file(output_dir + "/" + index_filename.str());
    if (!index_file.is_open()) {
        std::cerr << "❌ Could not create tile index file." << std::endl;
        return false;
    }

    for (float x = min_x; x < max_x; x += step_x) {
        for (float y = min_y; y < max_y; y += step_y) {
            Eigen::Vector4f min_crop(x, y, min_pt.z, 1.0f);
            Eigen::Vector4f max_crop(x + tile_size_x, y + tile_size_y, max_pt.z, 1.0f);

            pcl::CropBox<pcl::PCLPointCloud2> crop_filter;
            crop_filter.setInputCloud(cloud);
            crop_filter.setMin(min_crop);
            crop_filter.setMax(max_crop);

            pcl::PCLPointCloud2::Ptr tile(new pcl::PCLPointCloud2);
            crop_filter.filter(*tile);

            if (tile->width * tile->height == 0) continue;

            std::ostringstream filename;
            filename << base_filename << "_tile_" << tile_id;
            fs::path filepath = fs::path(output_dir) / (filename.str() + ".pcd");

            writer.writeBinaryCompressed(filepath.string(), *tile);
            std::cout << "✅ Saved tile: " << filepath << " (" << tile->width * tile->height << " points)" << std::endl;

            float center_x = x + tile_size_x / 2.0f;
            float center_y = y + tile_size_y / 2.0f;
            index_file << filename.str() << "   " << center_x << "   " << center_y << "\n";

            ++tile_id;
        }
    }

    return true;
}
