#include "src/tile.h"

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>
#include <filesystem>

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " input.pcd tile_size_x tile_size_y overlap output_dir" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    float tile_size_x = std::stof(argv[2]);
    float tile_size_y = std::stof(argv[3]);
    float overlap = std::stof(argv[4]);
    std::string output_dir = argv[5];

    // Zjistit název souboru bez přípony (např. trees -> trees)
    std::string base_filename = std::filesystem::path(input_file).stem().string();

    // Načíst mračno
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    if (pcl::io::loadPCDFile(input_file, *cloud) < 0) {
        std::cerr << "❌ Failed to load " << input_file << std::endl;
        return -1;
    }

    std::cout << "📥 Loaded " << cloud->width * cloud->height << " points from " << input_file << std::endl;

    if (!tilePointCloud2(cloud, tile_size_x, tile_size_y, overlap, output_dir, base_filename)) {
        std::cerr << "❌ Failed to tile point cloud." << std::endl;
        return -1;
    }

    std::cout << "✅ Tiling completed successfully." << std::endl;
    return 0;
}