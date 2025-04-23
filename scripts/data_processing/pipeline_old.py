#!/usr/bin/env python3

import os
import argparse
import yaml
import logging
from typing import List, Optional
import ctypes
# # ------------------------------------------------------------------
# # 1. Load Custom C++ PDAL Library
# # ------------------------------------------------------------------
# # Assuming you compiled your C++ PDAL code into a shared library:
# #   - Linux:   libmypdal.so
# #   - Windows: mypdal.dll
# #   - Mac:     libmypdal.dylib
# #
# # For demonstration, we'll call it "libmypdal.so"
# # Adjust the path as needed.
# pdal_lib = ctypes.cdll.LoadLibrary("libmypdal.so")
#
# # ------------------------------------------------------------------
# # 2. Define Python Prototypes for your C++ PDAL functions
# # ------------------------------------------------------------------
# # Example: a function to extract basic parameters from a LAS/LAZ file
# # We'll assume it returns success/failure as int, and populates
# # some "out" parameters via pointers or writes to a small text file.
#
# # int extract_basic_params(const char* input_file, const char* output_metadata)
# pdal_lib.extract_basic_params.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
# pdal_lib.extract_basic_params.restype = ctypes.c_int
#
# # Example: a function to filter outliers using SOR
# # int run_sor(const char* input_file, const char* output_file, int neighbors, float stddevmult)
# pdal_lib.run_sor.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_float]
# pdal_lib.run_sor.restype = ctypes.c_int
#
# # Example: a function to do tiling with overlap
# # int tile_with_overlap(const char* input_file, const char* output_dir,
# #                       float tile_size, float overlap)
# pdal_lib.tile_with_overlap.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_float, ctypes.c_float]
# pdal_lib.tile_with_overlap.restype = ctypes.c_int
#
#
# # If you have more functions (augmentation, etc.), define them similarly.
#
# ------------------------------------------------------------------
# 3. Pipeline Functions in Python
# ------------------------------------------------------------------

def extract_basic_params(input_file: str, output_metadata: str) -> bool:
    """Call the PDAL-based C++ function to get basic params."""
    res = pdal_lib.extract_basic_params(
        input_file.encode('utf-8'),
        output_metadata.encode('utf-8')
    )
    return (res == 0)
#
#
# def run_sor(input_file: str, output_file: str, neighbors: int, std_mult: float) -> bool:
#     """Call the PDAL-based C++ function to perform SOR filtering."""
#     res = pdal_lib.run_sor(
#         input_file.encode('utf-8'),
#         output_file.encode('utf-8'),
#         neighbors,
#         std_mult
#     )
#     return (res == 0)
#
#
# def tile_with_overlap(input_file: str, output_dir: str, tile_size: float, overlap: float) -> bool:
#     """Call the PDAL-based C++ function to tile the point cloud with overlap."""
#     res = pdal_lib.tile_with_overlap(
#         input_file.encode('utf-8'),
#         output_dir.encode('utf-8'),
#         tile_size,
#         overlap
#     )
#     return (res == 0)
#
#
# # Optionally: define a function for augmentations in C++ as well,
# # or implement them in Python. For example:
#
# def run_augmentation(input_file: str, output_file: str,
#                      do_rotation=True, do_translation=True, do_noise=True) -> bool:
#     """
#     Example placeholder for a C++ PDAL-based augmentation function.
#     We'll assume you wrote one named run_augmentation in C++.
#     """
#     # If so, define argtypes and restype above, then call here.
#     pass
#     return True


# ------------------------------------------------------------------
# 4. Main Pipeline Orchestration
# ------------------------------------------------------------------

def main(config_path: str):
    # -----------------------------
    # 4.1 Load Configuration & Setup Logging
    # -----------------------------
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    logging.basicConfig(
        filename=cfg['logging']['log_file'],
        filemode='a',
        level=getattr(logging, cfg['logging']['log_level'].upper()),
        format='%(asctime)s [%(levelname)s] %(message)s'
    )
    logging.getLogger().addHandler(logging.StreamHandler())
    logging.info("Starting pipeline with config: %s", config_path)

    # -----------------------------
    # 4.2 Parse Paths
    # -----------------------------
    raw_dir = cfg['data_paths']['raw_dir']
    processed_dir = cfg['data_paths']['processed_dir']
    augmented_dir = cfg['data_paths']['augmented_dir']
    tiles_dir = cfg['data_paths']['tiles_dir']
    metadata_dir = cfg['data_paths']['metadata_dir']



    # Ensure directories exist
    for d in [raw_dir, processed_dir, augmented_dir, tiles_dir, metadata_dir]:
        os.makedirs(d, exist_ok=True)

    # -----------------------------
    # 4.3 Pipeline Params
    # -----------------------------
    do_filtration = cfg['pipeline_params']['do_filtration']
    sor_neighbors = cfg['pipeline_params']['sor_neighbors']
    sor_std_mult = cfg['pipeline_params']['sor_std_multiplier']

    do_augment = any(cfg['pipeline_params']['augmentations'].values())
    tile_size = cfg['pipeline_params']['tile_size']
    tile_overlap = cfg['pipeline_params']['tile_overlap']

    # Gather .LAS/.LAZ input files
    raw_files = [f for f in os.listdir(raw_dir) if f.lower().endswith(('.las', '.laz'))]

    # # -----------------------------
    # 4.4 Basic Params Extraction
    # -----------------------------
    logging.info("Extracting basic params for %d files...", len(raw_files))
    basic_params_path = os.path.join(metadata_dir, "basic_params_raw.csv")
    with open(basic_params_path, 'w') as bp:
        bp.write("filename,result\n")
        for fname in raw_files:
            in_file = os.path.join(raw_dir, fname)
            metadata_file = os.path.join(metadata_dir, f"{os.path.splitext(fname)[0]}_params.txt")
            success = extract_basic_params(in_file, metadata_file)
            bp.write(f"{fname},{'OK' if success else 'FAIL'}\n")
    #
    # # -----------------------------
    # # 4.5 Filtration (SOR)
    # # -----------------------------
    # logging.info("Filtration step (SOR): %s", "Enabled" if do_filtration else "Disabled")
    # if do_filtration:
    #     for fname in raw_files:
    #         base, ext = os.path.splitext(fname)
    #         in_file = os.path.join(raw_dir, fname)
    #         out_file = os.path.join(processed_dir, base + "_proc" + ext)
    #
    #         success = run_sor(in_file, out_file, sor_neighbors, float(sor_std_mult))
    #         if not success:
    #             logging.warning("SOR failed for file: %s", fname)
    # else:
    #     # Copy raw to processed if skipping filtration
    #     for fname in raw_files:
    #         in_file = os.path.join(raw_dir, fname)
    #         out_file = os.path.join(processed_dir, fname)
    #         if not os.path.exists(out_file):
    #             os.system(f"cp '{in_file}' '{out_file}'")
    #
    # # Now gather processed files
    # processed_files = [
    #     f for f in os.listdir(processed_dir)
    #     if f.lower().endswith(('.las', '.laz'))
    # ]
    #
    # # -----------------------------
    # # 4.6 Data Augmentation
    # # -----------------------------
    # logging.info("Data Augmentation: %s", "Enabled" if do_augment else "Disabled")
    # if do_augment:
    #     for fname in processed_files:
    #         base, ext = os.path.splitext(fname)
    #         in_file = os.path.join(processed_dir, fname)
    #         out_file = os.path.join(augmented_dir, base + "_aug" + ext)
    #         # (Pseudo) call the augmentation function
    #         success = run_augmentation(
    #             in_file,
    #             out_file,
    #             do_rotation=cfg['pipeline_params']['augmentations']['random_rotation'],
    #             do_translation=cfg['pipeline_params']['augmentations']['random_translation'],
    #             do_noise=cfg['pipeline_params']['augmentations']['noise_injection']
    #         )
    #         if not success:
    #             logging.warning("Augmentation failed for file: %s", fname)
    # else:
    #     # If no augmentation, just copy processed to augmented folder
    #     for fname in processed_files:
    #         in_file = os.path.join(processed_dir, fname)
    #         out_file = os.path.join(augmented_dir, fname)
    #         if not os.path.exists(out_file):
    #             os.system(f"cp '{in_file}' '{out_file}'")
    #
    # # Now gather augmented files
    # augmented_files = [
    #     f for f in os.listdir(augmented_dir)
    #     if f.lower().endswith(('.las', '.laz'))
    # ]
    #
    # # -----------------------------
    # # 4.7 Tile Creation w/ Overlap
    # # -----------------------------
    # logging.info("Tiling with overlap: tile_size=%.2f, overlap=%.2f", tile_size, tile_overlap)
    # for fname in augmented_files:
    #     in_file = os.path.join(augmented_dir, fname)
    #     # We’ll store the tiles in a subfolder named after the base file
    #     base, ext = os.path.splitext(fname)
    #     out_dir = os.path.join(tiles_dir, base)
    #     os.makedirs(out_dir, exist_ok=True)
    #
    #     success = tile_with_overlap(
    #         in_file,
    #         out_dir,
    #         float(tile_size),
    #         float(tile_overlap)
    #     )
    #     if not success:
    #         logging.warning("Tiling failed for file: %s", fname)
    #
    # logging.info("Pipeline complete. Final tiles are in: %s", tiles_dir)
    #

# ------------------------------------------------------------------
# 5. Entry Point
# ------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True, help="Path to the YAML config file.")
    args = parser.parse_args()

    main(args.config)
