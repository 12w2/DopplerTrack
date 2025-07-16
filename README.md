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
roslaunch dopplertrack dopplertarck_<dataset>.launch output_dir:=<your_output_dir> data_path:=<your_data_path>
```

Parameter description:
- dataset: Name of the dataset (straight, intersection, aq_car, or aeva_car)
- output_dir: Path to save the processed results
- data_path: Path to the input dataset

Example:
```bash
roslaunch dopplertrack dopplertarck_aq_car.launch output_dir:=~/results/label_straight data_path:=~/dataset/Straight
```

## 4. Evaluation

## 5. Acknowledgement

We  refer to [CVC Cluster](https://github.com/wangx1996/Lidar-Segementation) for detection and [hungarian_optimizer](https://github.com/RocShi/hungarian_optimizer) for matching.

We sincerely thank the authors of these works for their valuable contributions.
## 7. Citation

If you find this work useful, please consider to cite our paper:
