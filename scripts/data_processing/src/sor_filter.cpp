#include "sor_filter.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

bool applySORFilter(const std::string& input_file,
                    const std::string& output_file,
                    int meanK,
                    double stddev,
                    pcl::PCLPointCloud2::Ptr& output_filtered)
{
    std::cerr << "📥 Loading file: " << input_file << std::endl;

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

    if (pcl::io::loadPCDFile(input_file, *cloud) == -1) {
        std::cerr << "❌ Couldn't read file " << input_file << std::endl;
        return false;
    }

    std::cout << "📦 Loaded " << cloud->width * cloud->height
              << " number of points " << input_file << std::endl;

    std::cout << "📋 Fields: ";
    for (const auto& field : cloud->fields) {
        std::cout << field.name << " ";
    }
    std::cout << std::endl;

    std::cerr << "🔄 Convert PointXYZ..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *xyz_cloud);
    std::cerr << "✅ Conversion done: " << xyz_cloud->size() << std::endl;

    std::cerr << "📊 Start of SOR filter: meanK=" << meanK << ", stddev=" << stddev << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(xyz_cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddev);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    sor.filter(inliers->indices);
    std::cerr << "✅ Filtred: " << inliers->indices.size() << " inliners" << std::endl;

    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    pcl::PCDWriter writer;
    if (writer.writeBinaryCompressed(output_file, *cloud_filtered) == -1) {
        std::cerr << "❌ Failed to save file " << output_file << std::endl;
        return false;
    }

    std::cout << "✅ Saved " << cloud_filtered->width * cloud_filtered->height
              << " Number of points " << output_file << " (compressed binary)" << std::endl;

    output_filtered = cloud_filtered;
    return true;
}
