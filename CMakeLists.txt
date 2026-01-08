cmake_minimum_required(VERSION 3.16)

project(RBFImplicitBoundary)

# 设置 Qt 版本 - 使用 PCL 兼容的配置
if(NOT DEFINED QT_MAJOR_VERSION)
    set(QT_MAJOR_VERSION 5)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 查找依赖库
find_package(PCL 1.8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)

# 使用标准 BLAS/LAPACK (用户可替换为 OpenBLAS)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# PCL 的 C++11/14 设置
add_definitions(${PCL_DEFINITIONS})

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
)

# 链接目录
link_directories(
    ${PCL_LIBRARY_DIRS}
)

# 源文件
set(CORE_SOURCES
    src/core/DistanceFunction.cpp
    src/core/RBFInterpolator.cpp
    src/core/MarchingCubes.cpp
    src/core/LabelInference.cpp
)

set(IO_SOURCES
    src/io/PointCloudLoader.cpp
)

set(GUI_SOURCES
    src/gui/MainWindow.cpp
    src/gui/PointCloudViewer.cpp
)

set(MAIN_SOURCES
    src/main.cpp
)

set(TOOL_SOURCES
    tools/generate_test_data.cpp
)

# Qt MOC/UI 处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 创建可执行文件
add_executable(rbf_implicit
    ${CORE_SOURCES}
    ${IO_SOURCES}
    ${GUI_SOURCES}
    ${MAIN_SOURCES}
)

# 测试数据生成工具
add_executable(generate_test_data
    ${TOOL_SOURCES}
)

# 链接库
target_link_libraries(rbf_implicit
    ${PCL_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
)

# 测试数据生成工具只使用标准库，无需链接额外库

# 线程支持 (PCL 需要，以及我们的后台线程)
find_package(Threads REQUIRED)
target_link_libraries(rbf_implicit Threads::Threads)
