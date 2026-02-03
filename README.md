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
Next, compile the extension modules. You can do this via:

```bash
python setup.py build_ext --inplace
```

> **Note:** To compile the extensions, you must have a CUDA-enabled device set up. If you experience any errors, you can simply comment out the specific CUDA dependencies in `setup.py`.

## 📊 Dataset Preparation: Synthetic Hole Generation

We evaluate our method on the [ShapeNet dataset](https://shapenet.org/), which contains over 55 categories and 57,748 unique 3D shapes. To simulate real-world scanning artifacts and occlusion, we generate synthetic holes using a K-Nearest Neighbors (KNN) algorithm.

### 1. Download the Dataset
Download the ShapeNet dataset and place it in the `data/` directory.

### 2. Generate Synthetic Holes
Run the preprocessing script to apply the KNN-based hole generation pipeline. This script randomly selects a seed point and removes between **512 and 4096 neighboring points** to create realistic voids.

```bash
python generate_synthetic_holes.py --dataset_dir data/shapenet --k_points 1024 --removal_range 512 4096
```
This will create a new directory `data/shapenet_with_holes/` containing the corrupted input data alongside the original ground truth.

## 🚀 How to Run

### 1. Training
To train the Implicit Autoencoder/Occupancy Network from scratch, run the training script. The network utilizes PointNet/DGCNN for the encoder and ResNet with Conditional Batch Normalization (CBN) for the decoder.
By default, the model validates every 10,000 iterations.

```bash
python train.py --config configs/shapenet_hole_filling.yaml --resolution 2048
```
You can monitor the training process using TensorBoard:
```bash
tensorboard --logdir out/shapenet_hole_filling/logs
```

### 2. Evaluation and Hole Filling (Generation)
To evaluate the model and generate the filled 3D point clouds on the test set using a pre-trained model:
```bash
python generate.py --config configs/shapenet_hole_filling.yaml --checkpoint out/pretrained/model_best.pt
```
The reconstructed, hole-filled point clouds will be saved in the `out/shapenet_hole_filling/generation/` directory as `.obj` or `.ply` files.

## 🙏 Acknowledgments
This project was carried out as a Research Experience for Undergraduates (REU) project at the **Centre of Excellence in Visual Intelligence (CEVI)**, KLE Technological University, under the guidance of Dr. Uma Mudenagudi and Ramesh Ashok Tabib. 

*Codebase inspired by the original [Occupancy Networks](https://github.com/autonomousvision/occupancy_networks).*
