#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QCheckBox>
#include <QGroupBox>
#include <memory>
#include "PointCloudViewer.h"
#include "io/PointCloudLoader.h"
#include "core/DistanceFunction.h"
#include "core/RBFInterpolator.h"
#include "core/MarchingCubes.h"
#include "core/LabelInference.h"

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
    // UI components
    QPushButton* btnLoadCloud_;
    QPushButton* btnLoadLabels_;
    QPushButton* btnRun_;
    QPushButton* btnClear_;
    QLabel* labelStatus_;
    QCheckBox* checkAutoInfer_;
    QGroupBox* optionsGroup_;
    PointCloudViewer* viewer_;

    // Data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::vector<DomainLabel> labels_;
    bool cloudLoaded_;
    bool labelsLoaded_;
    bool labelsInferred_;  // Track if labels were auto-inferred
};

} // namespace rbf

#endif // MAIN_WINDOW_H
