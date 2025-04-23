#include <pcl/PCLPointCloud2.h>
#ifndef SOR_FILTER_H
#define SOR_FILTER_H

#include <string>

// Používá PCL typy interně, ale na vstupu bere jen std::string a čísla
bool applySORFilter(const std::string& input_file,
                    const std::string& output_file,
                    int meanK,
                    double stddev,
                    pcl::PCLPointCloud2::Ptr& output_filtered);


#endif // SOR_FILTER_H
