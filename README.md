(The authors are still actively improving this evaluation platform, so the codes may be very fluid. )


```
@article{Yang2023Template,
  title={Template-Free Nonrevisiting Uniform Coverage Path Planning on Curved Surfaces},
  author={Yang, Tong and Miro, Jaime Valls and Nguyen, Minh and Wang, Yue and Xiong, Rong},
  journal={IEEE/ASME Transactions on Mechatronics},
  year={2023},
  volume={28},
  number={4},
  pages={1853--1861},
  publisher={IEEE}
}
```


# benchmarking_3DCPP

The aim of this package is to create a unified benchmarking platform for 

(1) various target surfaces for coverage, 

(2) various coverage path planning algorithms, and 

(3) various robot sensing (collision) models.

We are interested in providing a standard implementation of different algorithms. However, due to the fact that most algorithm cannot receive raw (maybe mesh-based or point cloud-based) surface data, and requires pre-processing such as cellular decomposition which is not yet standardized, we encourage the authors of these algorithms to implement the pre-processing steps by themselves. 


## Dependencies (Requirements)

open3d 0.18.0 (C++ version, please don't pip install)

sudo apt-get install libeigen3-dev

sudo apt install libglfw3 libglfw3-dev

## Dependencies (Recommendation)

CUDA (mine is 11.8)

spdlog

## Execution

### 1. Create a ROS2 package and download the package

```
cd
mkdir -p ~/benchmark_ws/src
cd ~/benchmark_ws/src
git clone https://github.com/ZJUTongYang/benchmarking_3D_coverage_path_planning.git
cd ~/benchmark_ws
colcon build
source install/setup.bash
```


### 2. Prepare the algorithms to be evaluated (taking NUC as an example)

```
cd ~/benchmark_ws/src
git clone 

```

### setting up all surface objects to be evaluated

copy your STL file and PCD files to the folder `benchmarking_3dcpp/scene/`. Currently, there is only one example `remeshed_saddle.stl`. 

### setting up the config files for evaluation



### 3. make sure that the ROS2 launch script will starts all 

``` 
ros2 launch benchmarking_3dcpp first.launch.py
```


### visualize


makesure that the fixed frame of rviz2 is "world"

trigger one you want

## Mention Us

If you evaluate your algorithm using our code, please consider citing our algorithm paper: 

```
@article{Yang2023Template,
  title={Template-Free Nonrevisiting Uniform Coverage Path Planning on Curved Surfaces},
  author={Yang, Tong and Miro, Jaime Valls and Nguyen, Minh and Wang, Yue and Xiong, Rong},
  journal={IEEE/ASME Transactions on Mechatronics},
  year={2023},
  volume={28},
  number={4},
  pages={1853--1861},
  publisher={IEEE}
}
```


