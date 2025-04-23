#ifndef TILE_H
#define TILE_H

#include <pcl/PCLPointCloud2.h>
#include <string>

// Rozdělí vstupní mračno na tily podle velikosti X, Y a overlapu a uloží je do výstupní složky
bool tilePointCloud2(const pcl::PCLPointCloud2::Ptr& cloud,
                     float tile_size_x,
                     float tile_size_y,
                     float overlap,
                     const std::string& output_dir,
                     const std::string& base_filename);

#endif // TILE_H
