import os
import json
import yaml
import argparse
import subprocess
import open3d as o3d
import numpy as np

def get_unprocessed_pcd_files(pcd_dir, metadata_dir):
    unprocessed_files = []
    for filename in os.listdir(pcd_dir):
        if filename.endswith(".pcd"):
            basename = os.path.splitext(filename)[0]
            metadata_path = os.path.join(metadata_dir, f"{basename}_metadata.json")
            if not os.path.exists(metadata_path):
                unprocessed_files.append(filename)
    return unprocessed_files

def extract_basic_params(input_file: str, output_metadata: str) -> bool:
    print(f"📊 Extracting metadata from: {input_file}")
    try:
        pcd = o3d.io.read_point_cloud(input_file)
        points = np.asarray(pcd.points)

        metadata = {
            "file_format": "PCD",
            "num_points": len(points),
            "min_bounds": points.min(axis=0).tolist(),
            "max_bounds": points.max(axis=0).tolist(),
            "fields": ["x", "y", "z"]
        }

        with open(output_metadata, "w") as f:
            json.dump(metadata, f, indent=4)

        print(f"✅ Metadata saved to {output_metadata}")
        return True
    except Exception as e:
        print(f"❌ Failed to extract metadata from {input_file}: {e}")
        return False

def run_clear_and_filter(pcd_file, output_root, meanK, stddev):
    cmd = [
        "./bin/clear_and_filter",
        pcd_file,
        output_root,
        str(meanK),
        str(stddev)
    ]
    try:
        subprocess.run(cmd, check=True)
        print(f"🧹 Filtered and augmented: {pcd_file}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ clear_and_filter failed for {pcd_file}: {e}")
        return False

def run_tile_pointcloud(input_file, tile_size_x, tile_size_y, overlap, output_dir):
    cmd = [
        "./bin/tile_pointcloud",
        input_file,
        str(tile_size_x),
        str(tile_size_y),
        str(overlap),
        output_dir
    ]
    try:
        subprocess.run(cmd, check=True)
        print(f"🧩 Tiled: {input_file}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ tile_pointcloud failed for {input_file}: {e}")
        return False

def main(config_path: str):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    pcd_dir = config["data_paths"]["pcd_dir"]
    metadata_dir = config["data_paths"]["metadata_dir"]
    augmented_dir = config["data_paths"]["augmented_dir"]
    tiles_dir = config["data_paths"]["tiles_dir"]
    output_root = os.path.dirname(pcd_dir)

    meanK = config["pipeline_params"].get("sor_neighbors", 20)
    stddev = config["pipeline_params"].get("sor_std_multiplier", 2.0)
    tile_size = config["pipeline_params"].get("tile_size", 10.0)
    overlap = config["pipeline_params"].get("tile_overlap", 1.0)

    os.makedirs(metadata_dir, exist_ok=True)
    os.makedirs(pcd_dir, exist_ok=True)
    os.makedirs(tiles_dir, exist_ok=True)

    unprocessed_files = get_unprocessed_pcd_files(pcd_dir, metadata_dir)
    print(f"🔍 Found {len(unprocessed_files)} unprocessed PCD files.")

    for pcd_file in unprocessed_files:
        basename = os.path.splitext(pcd_file)[0]
        local_pcd_path = os.path.join(pcd_dir, pcd_file)
        metadata_path = os.path.join(metadata_dir, f"{basename}_metadata.json")

        if not os.path.exists(local_pcd_path):
            print(f"❌ Missing PCD file: {local_pcd_path}. Skipping.")
            continue

        extract_basic_params(local_pcd_path, metadata_path)
        run_clear_and_filter(local_pcd_path, output_root, meanK, stddev)

    if os.path.exists(augmented_dir):
        for filename in os.listdir(augmented_dir):
            if filename.endswith(".pcd"):
                input_path = os.path.join(augmented_dir, filename)
                run_tile_pointcloud(input_path, tile_size, tile_size, overlap, tiles_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Path to the YAML config file.")
    args = parser.parse_args()

    main(args.config)