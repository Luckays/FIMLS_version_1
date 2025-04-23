cmake_minimum_required(VERSION 3.10)
project(PointCloudAugmentor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Soubory sdílené oběma nástroji
set(SHARED_SRC
    src/sor_filter.cpp
    src/augmentation.cpp
    src/downsample.cpp
    src/tile.cpp
)

# clear_and_filter nástroj
add_executable(clear_and_filter clear_and_filter.cpp ${SHARED_SRC})
target_link_libraries(clear_and_filter ${PCL_LIBRARIES})

# tile_pointcloud nástroj
add_executable(tile_pointcloud tile_pointcloud.cpp ${SHARED_SRC})
target_link_libraries(tile_pointcloud ${PCL_LIBRARIES})

add_executable(compress_pcd_files compress_pcd_files.cpp)
target_link_libraries(compress_pcd_files ${PCL_LIBRARIES})
