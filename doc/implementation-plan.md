# RBF 隐式边界重建实现计划

## 项目概述

基于论文《利用径向基函数进行隐式边界建模》实现隐式边界重建算法。

**技术栈**: C++14/17, CMake, PCL, Qt6, OpenBLAS

**简化决策**:
- RBF 类型: 仅高斯函数 $\phi(r) = e^{-\epsilon^2r^2}$
- 矩阵计算: 使用 OpenBLAS 加速
- 界面: 基础点云和网格显示

---

## 项目结构

```
implict-boundary-rbf/
├── CMakeLists.txt           # 主构建配置
├── src/
│   ├── core/                # 核心算法模块
│   │   ├── RBFInterpolator.h/cpp      # RBF 插值器
│   │   ├── DistanceFunction.h/cpp     # 符号距离函数计算
│   │   └── MarchingCubes.h/cpp        # 等值面提取 (MC算法)
│   ├── io/                  # 点云IO模块
│   │   └── PointCloudLoader.h/cpp     # PCL点云加载
│   ├── gui/                 # Qt6 可视化界面
│   │   ├── MainWindow.h/cpp           # 主窗口
│   │   └── PointCloudViewer.h/cpp     # OpenGL点云/网格显示
│   └── main.cpp
├── tests/                   # 单元测试
└── doc/                     # 文档
```

---

## 实现步骤

### 第1步: 项目框架搭建

**文件**: `CMakeLists.txt`

- 设置 C++14 标准
- 查找并链接依赖: PCL, Qt6, OpenBLAS
- 配置 include 目录和源文件

```cmake
cmake_minimum_required(VERSION 3.16)
project(RBFImplicitBoundary)

set(CMAKE_CXX_STANDARD 14)

# 查找依赖
find_package(PCL 1.8 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
find_package(OpenBLAS REQUIRED)

# 源文件定义
add_executable(rbf_implicit ...)

# 链接库
target_link_libraries(rbf_implicit PCL_COMMON_PCL_COMMON Qt6::Widgets ...)
```

---

### 第2步: 符号距离函数计算

**文件**: `src/core/DistanceFunction.h/cpp`

**功能**:
- 为每个点分配域指示符 $I(u_i) \in \{0, 1\}$
- 计算到最近不同域样本的欧几里得距离
- 内域样本赋负值，外域样本赋正值

**关键公式**:
```
DF(u_i) = {
    -argmin(r)  if I(u_i) = 1 (内域)
    +argmin(r)  if I(u_i) = 0 (外域)
}

r(u_i, u_j) = sqrt((x_i-x_j)^2 + (y_i-y_j)^2 + (z_i-z_j)^2)
```

**实现要点**:
- 输入: PCL `PointCloud<PointXYZ>` + 域标签向量
- 输出: 距离值向量 `std::vector<double>`

---

### 第3步: RBF 插值器

**文件**: `src/core/RBFInterpolator.h/cpp`

**功能**:
- 构建并求解 RBF 线性方程组: $\mathbf{Aw} = \boldsymbol{f}$
- 使用高斯 RBF: $\phi(r) = e^{-\epsilon^2r^2}$
- 使用 OpenBLAS 求解稠密线性系统

**核心步骤**:

1. **构建基函数矩阵 A** (N×N):
   ```
   A_ij = φ(r(u_i, u_j)) = exp(-ε² * r²)
   ```

2. **使用 OpenBLAS 求解**:
   - 使用 `cblas_dgemm` 进行矩阵运算
   - 使用 LAPACK 接口 (`dgels` 或 `dgesv`) 求解线性系统

3. **插值函数**:
   ```
   s(u) = Σ w_i * φ(r(u, u_i))
   ```

**参数**:
- $\epsilon$ (epsilon): RBF 形状参数，控制影响半径

**实现要点**:
- 使用 OpenBLAS/cblas 接口进行矩阵运算
- 处理矩阵的数值稳定性问题

---

### 第4步: Marching Cubes 等值面提取

**文件**: `src/core/MarchingCubes.h/cpp`

**功能**:
- 将隐式场体素化
- 提取 s(u) = 0 的等值面作为边界网格

**实现选项**:
1. **手动实现 MC**: 参考标准 15 种立方体构型表
2. **使用 PCL 的 MC**: `pcl::MarchingCubesHoppe` 或 `pcl::MarchingCubesRBF`

**推荐**: 使用 PCL 自带的 MC 实现，简化开发

**输出**: PCL `PolygonMesh` (三角形网格)

---

### 第5步: 点云加载器

**文件**: `src/io/PointCloudLoader.h/cpp`

**功能**:
- 加载常见点云格式: PCD, PLY, XYZ
- 解析点坐标和域标签

**PCL 支持**:
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::io::loadPCDFile("input.pcd", *cloud);
```

---

### 第6步: Qt6 可视化界面

**文件**: `src/gui/MainWindow.h/cpp`, `src/gui/PointCloudViewer.h/cpp`

**功能**:
1. **点云显示**: 使用 OpenGL 渲染输入点云
2. **网格显示**: 显示重建后的三角网格
3. **基础交互**: 旋转、缩放、平移

**实现方案**:
- 使用 `QOpenGLWidget` 自定义渲染
- 或使用 PCL 的 `pcl::visualization::PCLVisualizer` 嵌入 Qt

**推荐**: 使用 PCL Visualizer，集成简单且功能完善

---

### 第7步: 主程序集成

**文件**: `src/main.cpp`

**流程**:
```
1. 加载点云数据
2. 计算符号距离函数
3. 构建 RBF 插值器并求解权重
4. 使用 Marching Cubes 提取边界网格
5. 在 Qt 窗口中显示结果
```

---

## 关键文件清单

| 文件路径 | 功能说明 |
|---------|---------|
| `CMakeLists.txt` | 构建配置，链接 PCL/Qt6/OpenBLAS |
| `src/core/DistanceFunction.h/cpp` | 符号距离函数计算 |
| `src/core/RBFInterpolator.h/cpp` | RBF 插值 + OpenBLAS 求解 |
| `src/core/MarchingCubes.h/cpp` | 等值面提取 |
| `src/io/PointCloudLoader.h/cpp` | 点云文件加载 |
| `src/gui/MainWindow.h/cpp` | Qt 主窗口 |
| `src/gui/PointCloudViewer.h/cpp` | OpenGL 可视化 |
| `src/main.cpp` | 主程序入口 |

---

## 依赖库安装参考

```bash
# PCL (Ubuntu 24.04)
sudo apt install libpcl-dev pcl-tools

# OpenBLAS
sudo apt install libopenblas-openmp-dev

# Qt6 (已安装)
sudo apt install qt6-base-dev qt6-opengl-dev
```

---

## 测试数据

建议使用合成数据验证：
- 两个简单形状（如球体 vs 立方体）的点云
- 每个点标记所属域
- 测试边界提取的正确性

---

## 算法流程图

```
输入: 点云 + 域标签
    ↓
[符号距离函数计算]
计算每个点到最近不同域样本的距离
    ↓
[RBF 插值]
构建矩阵 A, 求解权重 w
    ↓
[Marching Cubes]
在体素网格上评估隐式函数，提取 s(u)=0 等值面
    ↓
输出: 三角网格边界
```
