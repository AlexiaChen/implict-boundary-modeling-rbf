#include "MainWindow.h"
#include <QProgressDialog>
#include <QApplication>
#include <QDebug>
#include <chrono>
#include <algorithm>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace rbf {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloudLoaded_(false)
{
    setupUI();
}

void MainWindow::setupUI() {
    setWindowTitle("RBF 3D Surface Reconstruction");
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
    auto* titleLabel = new QLabel("<h2>RBF 3D Reconstruction</h2>");
    titleLabel->setWordWrap(true);
    controlLayout->addWidget(titleLabel);

    controlLayout->addSpacing(20);

    // Buttons
    btnLoadCloud_ = new QPushButton("Load Point Cloud");
    btnRunRBF_ = new QPushButton("Reconstruct with RBF");
    btnRunPoisson_ = new QPushButton("Reconstruct with Poisson");
    btnClear_ = new QPushButton("Clear");

    btnLoadCloud_->setEnabled(true);
    btnRunRBF_->setEnabled(false);
    btnRunPoisson_->setEnabled(false);
    btnClear_->setEnabled(true);

    controlLayout->addWidget(btnLoadCloud_);
    controlLayout->addSpacing(10);
    controlLayout->addWidget(btnRunRBF_);
    controlLayout->addWidget(btnRunPoisson_);
    controlLayout->addWidget(btnClear_);

    controlLayout->addSpacing(20);

    // Status label
    labelStatus_ = new QLabel("Status: Please load point cloud data");
    labelStatus_->setWordWrap(true);
    controlLayout->addWidget(labelStatus_);

    controlLayout->addStretch();

    // Instructions
    auto* infoLabel = new QLabel(
        "<b>Instructions:</b><br>"
        "1. Load point cloud file (PCD/PLY/XYZ)<br>"
        "2. Click 'Reconstruct with RBF' or 'Reconstruct with Poisson'<br>"
        "3. View results in PCL Visualizer window<br><br>"
        "<i><b>RBF Method:</b> Based on Carr et al. paper<br>"
        "Uses polyharmonic RBF with off-surface points.</i>"
    );
    infoLabel->setWordWrap(true);
    controlLayout->addWidget(infoLabel);

    // Right viewer
    viewer_ = new PointCloudViewer;

    mainLayout->addWidget(controlPanel);
    mainLayout->addWidget(viewer_, 1);

    // Connect signals
    connect(btnLoadCloud_, &QPushButton::clicked, this, &MainWindow::onLoadPointCloud);
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

        // Show point cloud
        viewer_->showPointCloud(cloud_, "input_cloud");

        btnRunRBF_->setEnabled(true);
        btnRunPoisson_->setEnabled(true);

        labelStatus_->setText(
            QString("Status: Loaded %1 points\nReady for reconstruction").arg(cloud_->size())
        );

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load point cloud:\n%1").arg(e.what()));
    }
}

void MainWindow::onRunRBFReconstruction() {
    if (!cloudLoaded_ || cloud_->empty()) {
        QMessageBox::warning(this, "Warning", "Please load point cloud data first");
        return;
    }

    // 使用更细粒度的进度条 (0-100000，支持小数点后3位)
    QProgressDialog progress("Running RBF Surface Reconstruction...", "Cancel", 0, 100000, this);
    progress.setWindowTitle("RBF Reconstruction");
    progress.setWindowModality(Qt::WindowModal);
    progress.show();

    // 定义总步骤数
    const int TOTAL_STEPS = 5;
    int currentStep = 0;

    try {
        // Step 1: Estimate normals (0% - 20%)
        currentStep = 1;
        progress.setValue(0);  // 0.000%
        progress.setLabelText(QString("[1/%1] Estimating normals...").arg(TOTAL_STEPS));
        QApplication::processEvents();

        qDebug() << "[RBF] Step 1/" << TOTAL_STEPS << ": Estimating normals for" << cloud_->size() << "points";

        int kSearch = 30;
        auto normals = DistanceFunction::estimateNormals(cloud_, kSearch);

        progress.setValue(10000);  // 10.000%
        QApplication::processEvents();

        // Orient normals (all pointing outward)
        DistanceFunction::orientNormals(cloud_, normals);

        progress.setValue(20000);  // 20.000%
        labelStatus_->setText(QString("Status: [1/%1] Normals estimated - %2 points").arg(TOTAL_STEPS).arg(cloud_->size()));
        qDebug() << "[RBF] Step 1 complete: Normals estimated and oriented";

        // Step 2: Generate off-surface points (20% - 40%)
        currentStep = 2;
        progress.setValue(20000);  // 20.000%
        progress.setLabelText(QString("[2/%1] Generating off-surface points...").arg(TOTAL_STEPS));
        QApplication::processEvents();

        qDebug() << "[RBF] Step 2/" << TOTAL_STEPS << ": Generating off-surface points";

        double bboxDiagonal = DistanceFunction::computeBoundingBoxDiagonal(cloud_);
        double offsetDistance = 0.01 * bboxDiagonal;

        auto offSurfacePoints = DistanceFunction::generateOffSurfacePoints(
            cloud_, normals, offsetDistance
        );

        progress.setValue(40000);  // 40.000%
        labelStatus_->setText(
            QString("Status: [2/%1] Generated %2 off-surface points").arg(TOTAL_STEPS).arg(offSurfacePoints.size())
        );
        qDebug() << "[RBF] Step 2 complete: Generated" << offSurfacePoints.size() << "off-surface points";

        // Step 3: Build RBF interpolator (40% - 60%)
        currentStep = 3;
        progress.setValue(40000);  // 40.000%
        progress.setLabelText(QString("[3/%1] Building RBF matrix...").arg(TOTAL_STEPS));
        QApplication::processEvents();

        qDebug() << "[RBF] Step 3/" << TOTAL_STEPS << ": Building RBF interpolator";

        // Create point cloud from off-surface points
        pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
        centers->resize(offSurfacePoints.size());
        for (size_t i = 0; i < offSurfacePoints.size(); ++i) {
            centers->points[i] = offSurfacePoints[i].position;
        }

        // Extract distance values
        std::vector<double> distanceValues;
        distanceValues.reserve(offSurfacePoints.size());
        for (const auto& osp : offSurfacePoints) {
            distanceValues.push_back(osp.distanceValue);
        }

        progress.setValue(45000);  // 45.000%
        QApplication::processEvents();

        // Create RBF interpolator with polyharmonic RBF
        auto interpolator = std::make_shared<RBFInterpolator>(
            centers,
            distanceValues,
            RBFInterpolator::RBFFunction::Linear  // φ(r) = r
        );

        progress.setValue(50000);  // 50.000%
        QApplication::processEvents();

        // Solve linear system (50% - 90% - 这是最耗时的部分)
        int matrixSize = centers->size() + 4;
        labelStatus_->setText(
            QString("Status: [3/%1] Solving %2x%2 linear system...").arg(TOTAL_STEPS).arg(matrixSize).arg(matrixSize)
        );
        progress.setLabelText(QString("[3/%1] Solving %2x%2 linear system...").arg(TOTAL_STEPS).arg(matrixSize).arg(matrixSize));
        progress.setValue(55000);  // 55.000%
        QApplication::processEvents();

        qDebug() << "[RBF] Building" << matrixSize << "x" << matrixSize << "augmented matrix...";
        qDebug() << "[RBF] This may take a while for large matrices (O(n³) complexity)";
        qDebug() << "[RBF] Using Eigen multi-threaded LU decomposition...";

        // 设置进度回调：将 solve() 内部的 0-100 进度映射到进度条的 50000-90000 范围
        interpolator->setProgressCallback([&](int current, int total, const std::string& message) {
            // 映射: 0-100 -> 50000-90000
            int progressValue = 50000 + (current * 40000 / total);
            progress.setValue(progressValue);
            progress.setLabelText(QString("[3/%1] %2").arg(TOTAL_STEPS).arg(QString::fromStdString(message)));
            labelStatus_->setText(QString("Status: [3/%1] %2").arg(TOTAL_STEPS).arg(QString::fromStdString(message)));
            qDebug() << "[RBF]  " << QString::fromStdString(message);
            QApplication::processEvents();
        });

        auto startTime = std::chrono::high_resolution_clock::now();

        if (!interpolator->solve()) {
            QMessageBox::critical(this, "Error", "RBF linear system solve failed");
            return;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        progress.setValue(90000);  // 90.000%
        labelStatus_->setText(QString("Status: [3/%1] Solved in %2ms").arg(TOTAL_STEPS).arg(duration.count()));
        qDebug() << "[RBF] Step 3 complete: Linear system solved in" << duration.count() << "ms";

        // Step 4: Marching Cubes extract boundary (90% - 95%)
        currentStep = 4;
        progress.setValue(90000);  // 90.000%
        progress.setLabelText(QString("[4/%1] Extracting isosurface...").arg(TOTAL_STEPS));
        QApplication::processEvents();

        qDebug() << "[RBF] Step 4/" << TOTAL_STEPS << ": Running Marching Cubes";

        MarchingCubes mc(interpolator, 100);
        pcl::PolygonMesh mesh = mc.extract();

        progress.setValue(95000);  // 95.000%
        qDebug() << "[RBF] Step 4 complete: Extracted" << mesh.polygons.size() << "triangles";

        // Step 5: Show results (95% - 100%)
        currentStep = 5;
        progress.setValue(95000);  // 95.000%
        progress.setLabelText(QString("[5/%1] Rendering...").arg(TOTAL_STEPS));
        QApplication::processEvents();

        viewer_->clearAll();
        viewer_->showPointCloud(cloud_, "cloud");
        viewer_->showMesh(std::make_shared<pcl::PolygonMesh>(mesh), "rbf_mesh");

        progress.setValue(100000);  // 100.000%
        progress.setLabelText("Done!");
        qDebug() << "[RBF] Step 5/5: Rendering complete";
        qDebug() << "[RBF] ===== RECONSTRUCTION COMPLETE =====";
        qDebug() << "[RBF] Input points:" << cloud_->size();
        qDebug() << "[RBF] Output triangles:" << mesh.polygons.size();

        labelStatus_->setText(
            QString("Status: [5/5] Complete! Points: %1, Triangles: %2")
                .arg(cloud_->size())
                .arg(mesh.polygons.size())
        );

    } catch (const std::exception& e) {
        qDebug() << "[RBF] ERROR:" << e.what();
        QMessageBox::critical(this, "Error", QString("RBF reconstruction failed:\n%1").arg(e.what()));
        labelStatus_->setText("Status: RBF reconstruction failed");
    }
}

void MainWindow::onClear() {
    viewer_->clearAll();
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloudLoaded_ = false;

    btnRunRBF_->setEnabled(false);
    btnRunPoisson_->setEnabled(false);
    labelStatus_->setText("Status: Cleared. Please reload data.");
}

bool MainWindow::haveData() const {
    return cloudLoaded_ && !cloud_->empty();
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

        // Increase neighbor count for better normal estimation
        normalEstimation.setKSearch(50);

        // Estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute(*normals);

        // Calculate centroid
        pcl::PointXYZ centroid(0, 0, 0);
        for (const auto& point : cloud_->points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= cloud_->size();
        centroid.y /= cloud_->size();
        centroid.z /= cloud_->size();

        // Orient normals (all pointing outward)
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

            // If normal points inward, flip it
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

        // Adjust Poisson parameters
        poisson.setDepth(11);
        poisson.setSolverDivide(8);
        poisson.setIsoDivide(8);
        poisson.setSamplesPerNode(1.2);
        poisson.setConfidence(false);
        poisson.setManifold(false);
        poisson.setOutputPolygons(false);

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
