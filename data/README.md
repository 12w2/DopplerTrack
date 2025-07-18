# Dataset Directory Structure

The dataset directory contains the following folders:

```bash
data/
├── dynamic_pcd/ # Dynamic point clouds extracted from raw data (.pcd format)
├── original_bin/ # Raw LiDAR point cloud data (.bin format)
├── original_annotation/ # Original annotations (.json format)
└── README.md # Dataset description file
```
- `original_bin/`: Contains the original LiDAR point cloud files in `.bin` format.
- `dynamic_pcd/`: Contains dynamic point clouds separated from the raw data, in `.pcd` format.
- `original_annotation/`: Contains the original annotation data generated using the [SUSTechPOINTS](https://github.com/naurril/SUSTechPOINTS) labeling tool.

You can visualize the point clouds and their corresponding annotations using SUSTechPOINTS. Simply load the .pcd files from dynamic_pcd/ and the .json annotation files from original_annotation/ into the SUSTechPOINTS platform.

# Data License

This repository contains four datasets:

- **Straight**
- **Intersection**
- **Aq-car**
- **Aeva-car**

All datasets are licensed under the  
**[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0)](https://creativecommons.org/licenses/by-nc-sa/4.0/)**.

You must attribute the work in the manner specified by the author.  
You may not use the work for commercial purposes, and you may only distribute the resulting work under the same license if you alter, transform, or create the work.

The **Aeva-car** dataset is derived from a dataset published by **SNU RPM Labs**.  
Original dataset source: [HeLiPR](https://sites.google.com/view/heliprdataset/home)
