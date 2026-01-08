#include "MainWindow.h"
#include <QProgressDialog>
#include <QApplication>

namespace rbf {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloudLoaded_(false)
    , labelsLoaded_(false)
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
    btnRun_ = new QPushButton("Run Reconstruction");
    btnClear_ = new QPushButton("Clear");

    btnLoadCloud_->setEnabled(true);
    btnLoadLabels_->setEnabled(false);
    btnRun_->setEnabled(false);
    btnClear_->setEnabled(true);

    controlLayout->addWidget(btnLoadCloud_);
    controlLayout->addWidget(btnLoadLabels_);
    controlLayout->addSpacing(10);
    controlLayout->addWidget(btnRun_);
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
        "2. Load corresponding labels file<br>"
        "3. Click 'Run Reconstruction'<br>"
        "4. View results in PCL Visualizer window"
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
    connect(btnRun_, &QPushButton::clicked, this, &MainWindow::onRunReconstruction);
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
        labelStatus_->setText(
            QString("Status: Loaded %1 points\nPlease load labels file").arg(cloud_->size())
        );

        btnLoadLabels_->setEnabled(true);
        btnRun_->setEnabled(false);

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

        btnRun_->setEnabled(true);

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to load labels:\n%1").arg(e.what()));
    }
}

void MainWindow::onRunReconstruction() {
    if (!haveData()) {
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

        MarchingCubes mc(interpolator, 30);
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
    btnRun_->setEnabled(false);
    labelStatus_->setText("Status: Cleared. Please reload data.");
}

bool MainWindow::haveData() const {
    return cloudLoaded_ && labelsLoaded_ && !cloud_->empty();
}

} // namespace rbf
