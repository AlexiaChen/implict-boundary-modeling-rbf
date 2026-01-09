#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QGroupBox>
#include <memory>
#include "PointCloudViewer.h"
#include "io/PointCloudLoader.h"
#include "core/DistanceFunction.h"
#include "core/RBFInterpolator.h"
#include "core/MarchingCubes.h"

namespace rbf {

/**
 * @brief 主窗口
 *
 * 提供点云加载、RBF重建（基于新论文）和Poisson重建功能
 *
 * RBF重建流程（基于论文《使用径向基函数（RBF）重建和表示三维物体》）：
 * 1. 自动估计法向量
 * 2. 自动生成离面点（沿法线投影）
 * 3. 构建多谐波RBF插值器
 * 4. Marching Cubes提取等值面
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() = default;

private slots:
    void onLoadPointCloud();
    void onRunRBFReconstruction();
    void onRunPoissonReconstruction();
    void onClear();

private:
    void setupUI();
    bool haveData() const;

private:
    // UI components
    QPushButton* btnLoadCloud_;
    QPushButton* btnRunRBF_;
    QPushButton* btnRunPoisson_;
    QPushButton* btnClear_;
    QLabel* labelStatus_;
    PointCloudViewer* viewer_;

    // Data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    bool cloudLoaded_;
};

} // namespace rbf

#endif // MAIN_WINDOW_H
