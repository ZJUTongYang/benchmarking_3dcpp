# benchmarking_3D_coverage_path_planning

The aim of this package is to create a unified benchmarking platform for (1) various target surfaces for coverage, (2) various coverage path planning algorithms, and (3) various robot sensing (collision) models.

We are interested in providing a standard implementation of different algorithms. However, due to the fact that most algorithm cannot receive raw surface data and requires pre-processing such as cellular decomposition which is not yet standardized, we encourage the authors of these algorithms to implement the pre-processing steps by themselves. 

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

## Dependencies


spdlog
open3d 0.18.0 (C++ version)
nlohmann-json




