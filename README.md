# Point Cloud Hole Filling Using Implicit Functions

**Abstract**
3D point cloud generation often suffers from occlusion and scanning limitations, resulting in visual inconsistencies and geometric holes. This project introduces a deep learning framework to address 3D hole filling using implicit functions, specifically leveraging Occupancy Networks. By learning a continuous representation of the point cloud, our network extracts both surface representations and underlying geometry to seamlessly fill voids. This method ensures high visual consistency with surrounding geometry and has direct applications in digital heritage preservation, AR/VR, and robotics.

## 🚀 Key Features
* **Continuous Representation:** Utilizes Implicit Autoencoders and Occupancy Networks to model 3D space continuously, avoiding the memory constraints of voxel grids.
* **Synthetic Hole Generation:** Implements a K-Nearest Neighbors (KNN) algorithm to simulate realistic occlusion by removing 512–4096 points from the original point cloud.
* **High-Resolution Reconstruction:** Capable of reconstructing point clouds at 256, 1024, and 2048 resolutions.

## 🛠 Installation

First, you have to make sure that you have all dependencies in place. The simplest way to do so is to use [Anaconda](https://www.anaconda.com/). 

You can create an anaconda environment called `hole_filling` using:

```bash
conda env create -f environment.yaml
conda activate hole_filling
```
