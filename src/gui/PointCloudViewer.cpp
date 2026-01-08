#include "PointCloudViewer.h"
#include <QVBoxLayout>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

namespace rbf {

PointCloudViewer::PointCloudViewer(QWidget* parent)
    : QWidget(parent)
{
    setupUI();
}

PointCloudViewer::~PointCloudViewer() = default;

void PointCloudViewer::setupUI() {
    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // Create VTK widget for embedding in Qt
    vtkWidget_ = new QVTKOpenGLNativeWidget(this);
    layout->addWidget(vtkWidget_);

    // Get the render window and interactor from the widget
    auto renderWindow = vtkWidget_->renderWindow();
    auto interactor = vtkWidget_->interactor();

    // Create a renderer
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);

    // Create PCL Visualizer with existing renderer and render window
    // Don't create interactor (false), we'll use the widget's existing interactor
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(
        renderer, renderWindow, "RBF Viewer", false
    );

    // Setup the interactor with PCL's interactor style
    viewer_->setupInteractor(interactor, renderWindow);

    // Configure PCL Visualizer
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);
    viewer_->initCameraParameters();
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

    // Trigger render update
    vtkWidget_->renderWindow()->Render();
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

    // Trigger render update
    vtkWidget_->renderWindow()->Render();
}

void PointCloudViewer::clearAll() {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    vtkWidget_->renderWindow()->Render();
}

void PointCloudViewer::resetCamera() {
    viewer_->resetCamera();
    vtkWidget_->renderWindow()->Render();
}

} // namespace rbf
