#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
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
 * 提供点云加载、RBF 重建和结果显示功能
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() = default;

private slots:
    void onLoadPointCloud();
    void onLoadLabels();
    void onRunReconstruction();
    void onClear();

private:
    void setupUI();
    bool haveData() const;

private:
    // UI 组件
    QPushButton* btnLoadCloud_;
    QPushButton* btnLoadLabels_;
    QPushButton* btnRun_;
    QPushButton* btnClear_;
    QLabel* labelStatus_;
    PointCloudViewer* viewer_;

    // 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::vector<DomainLabel> labels_;
    bool cloudLoaded_;
    bool labelsLoaded_;
};

} // namespace rbf

#endif // MAIN_WINDOW_H
