# RBF 隐式边界重建 (RBF Implicit Boundary Reconstruction)

基于径向基函数（RBF）的隐式边界重建算法实现

## 项目概述

本项目实现了论文 [《Reconstruction and Representation of 3D Objects with Radial Basis
Functions》](https://www.cs.jhu.edu/~misha/Fall05/Papers/carr01.pdf)中的核心算法，包括：

- 符号距离函数计算
- RBF 插值
- Marching Cubes 等值面提取
- Qt5 可视化界面

## 技术栈

- **语言**: C++14
- **构建系统**: CMake
- **点云处理库**: PCL 1.14.0
- **GUI 框架**: Qt5
- **矩阵运算**: Eigen3

## 依赖库

### Ubuntu 24.04 安装命令

```bash
# PCL (点云库)
sudo apt install libpcl-dev pcl-tools

# Qt5 (GUI 框架，部分组件可能已安装)
sudo apt install qtbase5-dev

# OpenBLAS 和 Eigen (矩阵运算库)
sudo apt-get install -y libopenblas-openmp-dev libeigen3-dev
```

## 项目结构

```
implict-boundary-rbf/
├── CMakeLists.txt                    # 构建配置
├── README.md                          # 本文件
├── doc/                               # 文档目录
│   ├── research-report.md            # 研究报告
├── src/
│   ├── core/                         # 核心算法模块
│   │   ├── DistanceFunction.{h,cpp}     # 符号距离函数计算
│   │   ├── RBFInterpolator.{h,cpp}      # RBF 插值器
│   │   └── MarchingCubes.{h,cpp}        # 等值面提取
│   ├── io/                           # 点云 I/O 模块
│   │   └── PointCloudLoader.{h,cpp}     # 文件加载器
│   ├── gui/                          # Qt5 可视化界面
│   │   ├── MainWindow.{h,cpp}           # 主窗口
│   │   └── PointCloudViewer.{h,cpp}     # PCL Visualizer 封装
│   └── main.cpp                      # 程序入口
└── build/                            # 构建目录（运行 CMake 后生成）
    └── bin/
        └── rbf_implicit              # 可执行文件
```

## 编译说明

### 1. 创建构建目录并配置

```bash
cd /path/to/implict-boundary-rbf
mkdir build
cd build
cmake ..
```

### 2. 编译

```bash
make -j$(nproc)
```

### 3. 运行

```bash
./bin/rbf_implicit
```

## 算法原理

### 1. 符号距离函数

对于每个点，计算其到最近的不同域样本的欧几里得距离：

```
DF(u_i) = {
    -argmin(r)  if I(u_i) = 1 (内域)
    +argmin(r)  if I(u_i) = 0 (外域)
}

r(u_i, u_j) = sqrt((x_i-x_j)² + (y_i-y_j)² + (z_i-z_j)²)
```

### 2. RBF 插值

使用高斯径向基函数对距离函数进行插值：

```
s(u) = Σ w_i × φ(r(u, u_i))

φ(r) = exp(-ε²r²)  (高斯 RBF)
```

权重通过求解线性方程组获得：

```
A × w = f

其中 A_ij = φ(||u_i - u_j||)
```

### 3. Marching Cubes

在体素网格上评估隐式函数，提取 s(u) = 0 的等值面作为边界。

## 使用说明

### 准备测试数据

1. **点云文件** (PCD/PLY/XYZ 格式)
   - 包含点的 3D 坐标
   - 每行一个点

2. **域标签文件** (TXT 格式)
   - 每行一个标签 (0 或 1)
   - 与点云中的点一一对应

### 运行程序

1. 点击 **"加载点云"** 按钮选择点云文件
2. 点击 **"加载标签"** 按钮选择对应的域标签文件
3. 点击 **"运行重建"** 执行 RBF 隐式边界重建
4. 在独立的 PCL Visualizer 窗口中查看结果

### 界面功能

- **加载点云**: 支持 PCD、PLY、XYZ 格式
- **加载标签**: 支持文本格式，每行一个 0 或 1
- **运行重建**: 执行完整的 RBF 重建流程
- **清除**: 重置所有数据

### 可视化

- 点云和重建的网格将在独立的 **PCL Visualizer** 窗口中显示
- 使用鼠标交互：
  - 左键拖动：旋转
  - 中键拖动：平移
  - 滚轮：缩放

## 核心模块说明

### DistanceFunction (符号距离函数)

计算每个点到最近不同域样本的有符号距离，作为 RBF 插值的输入。

### RBFInterpolator (RBF 插值器)

- 构建高斯 RBF 矩阵
- 使用 LAPACK 求解线性系统
- 在任意位置评估插值函数

**参数**:
- `epsilon`: RBF 形状参数，控制影响半径 (默认: 0.1)

### MarchingCubes (等值面提取)

- 在体素网格上评估隐式函数
- 使用标准 Marching Cubes 算法提取零等值面
- 生成封闭、水密的三角网格

**参数**:
- `resolution`: 体素网格分辨率 (默认: 50)

## 算法特点

### 优势

- **拓扑鲁棒性**: 生成封闭、水密的流形网格
- **全局插值**: 对噪声和稀疏数据具有较强鲁棒性
- **数学基础**: 基于严格的数学理论，可证明收敛性

### 局限

- **计算复杂度**: RBF 矩阵为稠密矩阵，大规模数据下计算开销大
- **参数敏感**: epsilon 参数的选择影响结果
- **内存占用**: N×N 矩阵需要 O(N²) 内存

### 适用场景

- 中小规模点云数据 (< 5000 点)
- 需要拓扑保证的重建任务
- 地质建模、医学图像处理等领域

## 已知问题与注意事项

### 可视化方式

由于 VTK 9 中 `QVTKWidget` 已被弃用，本项目使用 **独立的 PCL Visualizer 窗口**显示结果，而不是嵌入到 Qt 窗口中。

### 性能优化建议

对于大规模点云数据：
1. 使用更小的 `epsilon` 参数
2. 降低 Marching Cubes 的分辨率
3. 考虑实现 Partition of Unity 等加速方法

## 文档

- [需求文档](doc/requirement.md) - 项目需求和依赖说明
- [论文翻译](doc/implict-boundary-with-rbf.md) - RBF 隐式边界建模论文
- [研究报告](doc/research-report.md) - 算法对比分析
- [实现计划](doc/implementation-plan.md) - 详细的实现步骤

## 开发环境

- **操作系统**: Linux Ubuntu 24.04 LTS
- **编译器**: GCC 13.3.0
- **CMake**: 3.16+

## 许可证

本项目仅用于学习和研究目的。

## 参考文献

1. Sanchez, S., & Deutsch, C.V. (2022). Implicit Boundary Modeling with Radial Basis Functions. GeostatisticsLessons.com
2. Cowan, J., et al. (2003). Practical Implicit Geological Modelling. 5th International Mining Geology Conference
3. Carr, J.C., et al. (2001). Reconstruction and Representation of 3D Objects with Radial Basis Functions. SIGGRAPH 2001
