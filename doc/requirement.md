## 开发环境

- Linux  ubuntu 24.04 LTS
- CMake
- C++ （尽可能用C++ 14 或者 C++ 17）

## 依赖库

- 一个解析读取点云数据的库， 需要调研， 目前考虑使用 [PCL(Point Cloud Library)](https://pointclouds.org/)。
Context7 文档： https://context7.com/websites/pointclouds

- Qt,主要是来实现显示点云的界面然后也可以展示建模效果，sudo apt install qt6-base-dev （本地已经安装）
Context7 文档： https://context7.com/websites/qt-6-docs

- OpenBLAS, 用于矩阵运算， sudo apt install libopenblas-openmp-dev (本地已经安装)
Context7 文档： https://context7.com/openmathlib/openblas https://context7.com/openmathlib/openblas

## 目标需求

根据[论文](./implict-boundary-with-rbf.md)实现隐式边界重建算法， 主要包括以下几个模块： 
1. 点云数据读取与解析
2. 隐式边界重建算法实现
3. 点云可视化界面

[研究报告](./research-report.md)中主要是一些调研，以及为什么选择隐式边界重建算法的原因。包括一些其他的参考资料链接，主要是用来做参考的。