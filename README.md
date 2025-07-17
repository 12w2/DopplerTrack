# DopplerTrack

## Simpler Is Better: Revisiting Doppler Velocity for Enhanced Moving Object Tracking with FMCW LiDAR

## 0. Abstract
This paper proposes DopplerTrack, a simple yet effective learning-free tracking method tailored for FMCW LiDAR. DopplerTrack harnesses Doppler velocity for efficient point cloud preprocessing and object detection with O(N) complexity. Furthermore, by exploring the potential motion directions of objects, it reconstructs the full velocity vector, enabling more direct and precise motion prediction. Extensive experiments on four datasets demonstrate that DopplerTrack outperforms existing learning-free and learning-based methods, achieving state-of-the-art tracking performance with strong generalization across diverse scenarios. Moreover, DopplerTrack runs efficiently at 120 Hz on a mobile CPU, making it highly practical for real-world deployment. 

## 1. News
- [2025-7-16] The code has been released.
- [2025-6-16] Our work is accepted for IROS2025.🎉

## 2. Installation

### 2.1 Environment & Dependencies

The following is the tested environment for this package:

- Ubuntu 20.04
- ROS Noetic
- PCL 1.10 (from ros-noetic)
- OpenCV 4.2 (from ros-noetic)

### 2.2 Data 
#### Download
Please fill out the form below to request access to the dataset. We will send you the download link after receiving your request.
[Click here to fill out the data request form (Google Form)](https://docs.google.com/forms/d/e/1FAIpQLSc2T57lVPA2QQ2BXVziZfxVbae0rwpnx1lm-ydJEMg_J5TYSQ/viewform?usp=dialog)

#### Usage
After downloading the dataset, replace the existing `data/` folder in your project with the downloaded one.

#### Directory Structure
The dataset is organized as follows:

```bash
data/
├── dynamic_pcd
│ ├── Aeva-car
│ ├── Aq-car
│ ├── Intersection
│ └── Straight
├── original_bin
│ ├── Aeva-car
│ ├── Aq-car
│ ├── Intersection
│ └── Straight
└── README.md
```

### 2.3 Install and Build

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/12w2/DopplerTrack.git
cd ..
catkin_make
source devel/setup.bash
```

## 3. Run
To run the module, choose one of the four predefined datasets by selecting the corresponding launch file. Use the following command format:


```bash
roslaunch dopplertrack dopplertarck_<dataset>.launch 
```

Parameter description:
- dataset: Name of the dataset (Straight, Intersection, Aq-car, or Aeva-car)


Example:
```bash
roslaunch dopplertrack dopplertarck_aq_car.launch output_dir:=~/results/label_straight data_path:=~/dataset/Straight
```


## 4. Evaluation

Evaluation using a point-based IoU computation method,  where the IoU is calculated by counting the number of intersected and union points between ground truth and predicted objects.  This method is implemented by modifying the [py-motmetrics](https://github.com/cheind/py-motmetrics) library.

### 4.1 Installing the Evaluation Library and Dependencies
```bash
pip install -r requirements.txt
```

### 4.1 Run Evaluation
```bash
cd eval
python eval.py  <dataset>
```
Parameter description:
- dataset: Name of the dataset (Straight, Intersection, Aq-car, or Aeva-car)

Example:
```bash
python eval.py  Aq-car
```

## 5. Acknowledgement

We  refer to [CVC Cluster](https://github.com/wangx1996/Lidar-Segementation) for detection and [hungarian_optimizer](https://github.com/RocShi/hungarian_optimizer) for matching.

We sincerely thank the authors of these works for their valuable contributions.
## 7. Citation

If you find this work useful, please consider to cite our paper:

## 8. License

- **Code**: [MIT License](./LICENSE)
- **Data**: All datasets are licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/). See [DATA_LICENSE.md](./DATA_LICENSE.md) for dataset details and attribution.
