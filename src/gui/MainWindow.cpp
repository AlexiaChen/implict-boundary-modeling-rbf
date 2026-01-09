#include "MainWindow.h"
#include <QProgressDialog>
#include <QApplication>
#include <algorithm>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace rbf {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloudLoaded_(false)
    , labelsLoaded_(false)
    , labelsInferred_(false)
{
    setupUI();
}

void MainWindow::setupUI() {
    setWindowTitle("RBF Implicit Boundary Reconstruction");
    resize(1024, 768);

    // Create central widget
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    auto* mainLayout = new QHBoxLayout(centralWidget);

    // Left control panel
    auto* controlPanel = new QWidget;
    controlPanel->setFixedWidth(250);
    auto* controlLayout = new QVBoxLayout(controlPanel);

    // Title
    auto* titleLabel = new QLabel("<h2>RBF Implicit Boundary</h2>");
    titleLabel->setWordWrap(true);
    controlLayout->addWidget(titleLabel);

    controlLayout->addSpacing(20);

    // Buttons
    btnLoadCloud_ = new QPushButton("Load Point Cloud");
    btnLoadLabels_ = new QPushButton("Load Labels");
    btnRunRBF_ = new QPushButton("Reconstruct with RBF");
    btnRunPoisson_ = new QPushButton("Reconstruct with Poisson");
    btnClear_ = new QPushButton("Clear");

    btnLoadCloud_->setEnabled(true);
    btnLoadLabels_->setEnabled(false);
    btnRunRBF_->setEnabled(false);
    btnRunPoisson_->setEnabled(false);
    btnClear_->setEnabled(true);

    controlLayout->addWidget(btnLoadCloud_);
    controlLayout->addWidget(btnLoadLabels_);
    controlLayout->addSpacing(10);
    controlLayout->addWidget(btnRunRBF_);
    controlLayout->addWidget(btnRunPoisson_);
    controlLayout->addWidget(btnClear_);

    controlLayout->addSpacing(20);

    // Options group
    optionsGroup_ = new QGroupBox("Options");
    auto* optionsLayout = new QVBoxLayout(optionsGroup_);

    checkAutoInfer_ = new QCheckBox("Auto-infer domain labels");
    checkAutoInfer_->setChecked(true);  // Enable by default
    checkAutoInfer_->setToolTip(
        "Automatically infer domain labels from point cloud geometry.\n"
        "Uses convex hull method: hull points = outer (0), inner points = inner (1).\n\n"
        "Disable this if you have a separate labels file with ground truth labels."
    );
    optionsLayout->addWidget(checkAutoInfer_);

    controlLayout->addWidget(optionsGroup_);
    controlLayout->addSpacing(10);

    // Status label
    labelStatus_ = new QLabel("Status: Please load point cloud data");
    labelStatus_->setWordWrap(true);
    controlLayout->addWidget(labelStatus_);

    controlLayout->addStretch();

    // Instructions
    auto* infoLabel = new QLabel(
        "<b>Instructions:</b><br>"
        "1. Load point cloud file (PCD/PLY/XYZ)<br>"
        "2. Optionally load labels file, or enable auto-infer<br>"
        "3. Click 'Run Reconstruction'<br>"
        "4. View results in PCL Visualizer window<br><br>"
        "<i>Note: Auto-infer works best for closed objects.</i>"
    );
    infoLabel->setWordWrap(true);
    controlLayout->addWidget(infoLabel);

    // Right viewer
    viewer_ = new PointCloudViewer;

    mainLayout->addWidget(controlPanel);
    mainLayout->addWidget(viewer_, 1);

    // Connect signals
    connect(btnLoadCloud_, &QPushButton::clicked, this, &MainWindow::onLoadPointCloud);
    connect(btnLoadLabels_, &QPushButton::clicked, this, &MainWindow::onLoadLabels);
    connect(btnRunRBF_, &QPushButton::clicked, this, &MainWindow::onRunRBFReconstruction);
    connect(btnRunPoisson_, &QPushButton::clicked, this, &MainWindow::onRunPoissonReconstruction);
    connect(btnClear_, &QPushButton::clicked, this, &MainWindow::onClear);
}

void MainWindow::onLoadPointCloud() {
    QString filename = QFileDialog::getOpenFileName(
        this,
        "Select Point Cloud File",
        "",
        "Point Cloud Files (*.pcd *.ply *.xyz);;All Files (*.*)"
    );

    if (filename.isEmpty()) {
        return;
    }

    try {
        cloud_ = PointCloudLoader::load(filename.toStdString());
        cloudLoaded_ = true;

        // Reset label state
        labelsLoaded_ = false;
        labelsInferred_ = false;
        labels_.clear();

        // Show point cloud
        viewer_->showPointCloud(cloud_, "input_cloud");

        btnLoadLabels_->setEnabled(true);

        // Poisson only needs point cloud (with normals estimated internally)
        btnRunPoisson_->setEnabled(true);

        // If auto-infer is enabled, enable RBF button
        if (checkAutoInfer_->isChecked()) {
            btnRunRBF_->setEnabled(true);
            labelStatus_->setText(
                QString("Status: Loaded %1 points\nReady for reconstruction (auto-infer enabled)")
                    .arg(cloud_->size())
            );
        } else {
            btnRunRBF_->setEnabled(false);
            labelStatus_->setText(
                QString("Status: Loaded %1 points\nPlease load labels file").arg(cloud_->size())
            );
        }

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load point cloud:\n%1").arg(e.what()));
    }
}

void MainWindow::onLoadLabels() {
    QString filename = QFileDialog::getOpenFileName(
        this,
        "Select Labels File",
        "",
        "Label Files (*.txt *.labels);;All Files (*.*)"
    );

    if (filename.isEmpty()) {
        return;
    }

    try {
        labels_ = PointCloudLoader::loadLabels(filename.toStdString());

        if (labels_.size() != cloud_->size()) {
            QMessageBox::warning(
                this,
                "Warning",
                QString("Label count (%1) does not match point cloud size (%2)")
                    .arg(labels_.size())
                    .arg(cloud_->size())
            );
            return;
        }

        labelsLoaded_ = true;
        labelStatus_->setText(
            QString("Status: Loaded %1 points and labels\nReady for reconstruction").arg(cloud_->size())
        );

        btnRunRBF_->setEnabled(true);

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load labels:\n%1").arg(e.what()));
    }
}

void MainWindow::onRunRBFReconstruction() {
    if (!cloudLoaded_ || cloud_->empty()) {
        QMessageBox::warning(this, "Warning", "Please load point cloud data first");
        return;
    }

    // Auto-infer labels if not loaded
    if (!labelsLoaded_ && checkAutoInfer_->isChecked()) {
        try {
            labelStatus_->setText("Status: Auto-inferring domain labels...");
            QApplication::processEvents();

            labels_ = LabelInference::infer(cloud_, LabelInference::Method::ConvexHull);
            labelsLoaded_ = true;
            labelsInferred_ = true;

            // Count inner and outer points
            int innerCount = std::count(labels_.begin(), labels_.end(), 1);
            int outerCount = std::count(labels_.begin(), labels_.end(), 0);

            labelStatus_->setText(
                QString("Status: Auto-inferred labels\nInner: %1, Outer: %2\nReady for reconstruction")
                    .arg(innerCount)
                    .arg(outerCount)
            );
        } catch (const std::exception& e) {
            QMessageBox::critical(
                this,
                "Error",
                QString("Failed to auto-infer labels:\n%1\n\nPlease load labels manually or disable auto-infer.")
                    .arg(e.what())
            );
            return;
        }
    }

    if (!labelsLoaded_) {
        QMessageBox::warning(this, "Warning", "Please load labels file or enable auto-infer");
        return;
    }

    // Show progress dialog
    QProgressDialog progress("Running RBF Implicit Boundary Reconstruction...", "Cancel", 0, 4, this);
    progress.setWindowTitle("Reconstructing");
    progress.setWindowModality(Qt::WindowModal);
    progress.show();

    try {
        // Step 1: Compute signed distance function
        progress.setValue(1);
        progress.setLabelText("Computing signed distance function...");
        QApplication::processEvents();

        std::vector<double> distances = DistanceFunction::compute(cloud_, labels_);

        // Step 2: Build RBF interpolator
        progress.setValue(2);
        progress.setLabelText("Building RBF interpolator...");
        QApplication::processEvents();

        auto interpolator = std::make_shared<RBFInterpolator>(cloud_, distances, 0.1);

        if (!interpolator->solve()) {
            QMessageBox::critical(this, "Error", "RBF linear system solve failed");
            return;
        }

        // Step 3: Marching Cubes extract boundary
        progress.setValue(3);
        progress.setLabelText("Extracting boundary mesh...");
        QApplication::processEvents();

        MarchingCubes mc(interpolator, 80);
        pcl::PolygonMesh mesh = mc.extract();

        // Step 4: Show results
        progress.setValue(4);
        progress.setLabelText("Done!");

        viewer_->clearAll();
        viewer_->showPointCloud(cloud_, "cloud");
        viewer_->showMesh(std::make_shared<pcl::PolygonMesh>(mesh), "mesh");

        labelStatus_->setText(
            QString("Status: Reconstruction complete!\nPoints: %1\nTriangles: %2")
                .arg(cloud_->size())
                .arg(mesh.polygons.size())
        );

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Reconstruction failed:\n%1").arg(e.what()));
        labelStatus_->setText("Status: Reconstruction failed");
    }
}

void MainWindow::onClear() {
    viewer_->clearAll();
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    labels_.clear();
    cloudLoaded_ = false;
    labelsLoaded_ = false;

    btnLoadLabels_->setEnabled(false);
    btnRunRBF_->setEnabled(false);
    btnRunPoisson_->setEnabled(false);
    labelStatus_->setText("Status: Cleared. Please reload data.");
}

bool MainWindow::haveData() const {
    return cloudLoaded_ && labelsLoaded_ && !cloud_->empty();
}

void MainWindow::onRunPoissonReconstruction() {
    if (!cloudLoaded_ || cloud_->empty()) {
        QMessageBox::warning(this, "Warning", "Please load point cloud data first");
        return;
    }

    // Show progress dialog
    QProgressDialog progress("Running Poisson Surface Reconstruction...", "Cancel", 0, 3, this);
    progress.setWindowTitle("Poisson Reconstruction");
    progress.setWindowModality(Qt::WindowModal);
    progress.show();

    try {
        // Step 1: Estimate normals (required by Poisson)
        progress.setValue(1);
        progress.setLabelText("Estimating point cloud normals...");
        QApplication::processEvents();

        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(
            new pcl::PointCloud<pcl::PointNormal>
        );

        // Create normal estimation object
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud_);

        // Create KdTree for normal estimation
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>()
        );
        normalEstimation.setSearchMethod(tree);

        // 增加邻居数来改善法向量估计，特别对密度不均的区域
        normalEstimation.setKSearch(50);  // 从 30 增加到 50

        // Estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute(*normals);

        // 计算点云中心
        pcl::PointXYZ centroid(0, 0, 0);
        for (const auto& point : cloud_->points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= cloud_->size();
        centroid.y /= cloud_->size();
        centroid.z /= cloud_->size();

        // 一致化法向量方向（所有法向量朝外）
        for (size_t i = 0; i < normals->size(); ++i) {
            Eigen::Vector3f normal(
                normals->points[i].normal_x,
                normals->points[i].normal_y,
                normals->points[i].normal_z
            );
            Eigen::Vector3f centroidToPoint(
                cloud_->points[i].x - centroid.x,
                cloud_->points[i].y - centroid.y,
                cloud_->points[i].z - centroid.z
            );

            // 如果法向量与"从中心指向点"方向相反，翻转法向量
            if (normal.dot(centroidToPoint) < 0) {
                normals->points[i].normal_x *= -1;
                normals->points[i].normal_y *= -1;
                normals->points[i].normal_z *= -1;
            }
        }

        // Concatenate XYZ and normals
        pcl::concatenateFields(*cloud_, *normals, *cloudWithNormals);

        // Step 2: Run Poisson reconstruction
        progress.setValue(2);
        progress.setLabelText("Running Poisson surface reconstruction...");
        QApplication::processEvents();

        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setInputCloud(cloudWithNormals);

        // 调整 Poisson 参数来桥接间隙
        poisson.setDepth(11);
        poisson.setSolverDivide(8);
        poisson.setIsoDivide(8);
        poisson.setSamplesPerNode(1.2);  // 降低采样密度，让表面更平滑
        poisson.setConfidence(false);    // 关闭置信度，让算法更激进地桥接间隙
        poisson.setManifold(false);      // 关闭流形约束，允许非流形连接
        poisson.setOutputPolygons(false); // 输出三角网格

        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);

        // Step 3: Show results
        progress.setValue(3);
        progress.setLabelText("Done!");

        viewer_->clearAll();
        viewer_->showPointCloud(cloud_, "cloud");
        viewer_->showMesh(std::make_shared<pcl::PolygonMesh>(mesh), "poisson_mesh");

        labelStatus_->setText(
            QString("Status: Poisson reconstruction complete!\nPoints: %1\nTriangles: %2")
                .arg(cloud_->size())
                .arg(mesh.polygons.size())
        );

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Poisson reconstruction failed:\n%1").arg(e.what()));
        labelStatus_->setText("Status: Poisson reconstruction failed");
    }
}

} // namespace rbf
