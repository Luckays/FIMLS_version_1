#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <random>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include "src/sor_filter.h"
#include "src/augmentation.h"


namespace fs = std::filesystem;

// Vrací název bez přípony, např. "trees_filtered"
std::string getBaseName(const std::string& path) {
    fs::path p(path);
    return p.stem().string();
}

// Náhodná voxel velikost v rozsahu <min, max>
float randomVoxelSize(float min, float max) {
    static std::default_random_engine gen(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}

int main(int argc, char** argv)
{
    // Zkontroluj argumenty¨
    std::cerr << "Start" << std::endl;
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " input.pcd output_filtered.pcd meanK stddev" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_parent_folder = argv[2];
    int meanK = std::stoi(argv[3]);
    double stddev = std::stod(argv[4]);

    std::string processed_folder = fs::path(output_parent_folder) / "processed";
    std::string augmented_folder = fs::path(output_parent_folder) / "augmented";

    fs::create_directories(processed_folder);
    fs::create_directories(augmented_folder);

    std::string base_name = getBaseName(input_file);
    std::string output_filtered_file = fs::path(processed_folder) / (base_name + "_SOR.pcd");



    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2);

    // ✅ SOR filtr + uložení + výstup do paměti
    bool success = applySORFilter(input_file, output_filtered_file, meanK, stddev, filtered_cloud);
    if (!success) {
        return 1;
    }

    // ✅ Augmentace
    pcl::PCDWriter writer;

    for (int i = 1; i <= 5; ++i) {
        pcl::PCLPointCloud2::Ptr augmented(new pcl::PCLPointCloud2);
        float voxel_size = randomVoxelSize(0.005f, 0.02f);  // 5–20 mm

        augmentPointCloud2(filtered_cloud, augmented,
                           true,   // rotation
                           true,   // translation
                           true,   // scale
                           true,   // noise
                           voxel_size);

        std::ostringstream filename;
        filename << base_name << "_" << std::setw(2) << std::setfill('0') << i << ".pcd";
        std::string output_augmented_file = fs::path(augmented_folder) / filename.str();

        if (writer.writeBinaryCompressed(output_augmented_file, *augmented) != 0) {
            std::cerr << "❌ Failed to save " << output_augmented_file << std::endl;
        } else {
            std::cout << "✅ Saved " << output_augmented_file
                      << " (voxel size: " << voxel_size * 1000 << " mm)" << std::endl;
        }
    }

    return 0;
}
