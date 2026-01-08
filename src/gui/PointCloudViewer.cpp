#include "PointCloudViewer.h"
#include <QLabel>
#include <QVBoxLayout>

namespace rbf {

PointCloudViewer::PointCloudViewer(QWidget* parent)
    : QWidget(parent)
    , viewer_(new pcl::visualization::PCLVisualizer("RBF Implicit Boundary - Visualization", true))
{
    setupUI();
}

PointCloudViewer::~PointCloudViewer() = default;

void PointCloudViewer::setupUI() {
    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // Create a label to inform user
    auto label = new QLabel("Point cloud and mesh will be displayed\n"
                           "in a separate PCL Visualizer window", this);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 20px; "
                         "border: 2px solid #ccc; border-radius: 5px; }");
    layout->addWidget(label);

    // Configure PCL Visualizer
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    // Show window (PCL Visualizer creates a separate VTK window)
    viewer_->setShowFPS(true);
    viewer_->setWindowName("RBF Implicit Boundary - Visualization");
}

void PointCloudViewer::showPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& id)
{
    // Random color
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        cloud,
        static_cast<uint8_t>(rand() % 256),
        static_cast<uint8_t>(rand() % 256),
        static_cast<uint8_t>(rand() % 256)
    );

    if (!viewer_->updatePointCloud<pcl::PointXYZ>(cloud, handler, id)) {
        viewer_->addPointCloud<pcl::PointXYZ>(cloud, handler, id);
    }

    // Reset camera to view point cloud
    viewer_->resetCamera();

    // Trigger rendering
    viewer_->spinOnce();
}

void PointCloudViewer::showMesh(
    const pcl::PolygonMesh::Ptr& mesh,
    const std::string& id)
{
    if (!viewer_->updatePolygonMesh(*mesh, id)) {
        viewer_->addPolygonMesh(*mesh, id);
    }

    // Set mesh color
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        0.8, 0.6, 0.4,  // Light brown
        id
    );

    // Set representation to surface
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
        id
    );

    // Trigger rendering
    viewer_->spinOnce();
}

void PointCloudViewer::clearAll() {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
}

void PointCloudViewer::resetCamera() {
    viewer_->resetCamera();
}

} // namespace rbf
