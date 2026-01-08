#include "PointCloudViewer.h"
#include <QLabel>
#include <QVBoxLayout>

namespace rbf {

PointCloudViewer::PointCloudViewer(QWidget* parent)
    : QWidget(parent)
    , viewer_(new pcl::visualization::PCLVisualizer("RBF 隐式边界重建 - 可视化窗口", false))
{
    setupUI();
}

PointCloudViewer::~PointCloudViewer() = default;

void PointCloudViewer::setupUI() {
    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 创建一个标签提示用户
    auto label = new QLabel("点云和网格将在独立的 PCL Visualizer 窗口中显示\n"
                           "窗口标题: 'RBF 隐式边界重建 - 可视化窗口'", this);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 20px; "
                         "border: 2px solid #ccc; border-radius: 5px; }");
    layout->addWidget(label);

    // 配置 PCL Visualizer
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    // 显示窗口（PCL Visualizer 会创建独立的 VTK 窗口）
    viewer_->setShowFPS(true);
    viewer_->setWindowName("RBF 隐式边界重建 - 可视化窗口");
}

void PointCloudViewer::showPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& id)
{
    // 随机颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        cloud,
        static_cast<uint8_t>(rand() % 256),
        static_cast<uint8_t>(rand() % 256),
        static_cast<uint8_t>(rand() % 256)
    );

    if (!viewer_->updatePointCloud<pcl::PointXYZ>(cloud, handler, id)) {
        viewer_->addPointCloud<pcl::PointXYZ>(cloud, handler, id);
    }

    // 重置相机以查看点云
    viewer_->resetCamera();
}

void PointCloudViewer::showMesh(
    const pcl::PolygonMesh::Ptr& mesh,
    const std::string& id)
{
    if (!viewer_->updatePolygonMesh(*mesh, id)) {
        viewer_->addPolygonMesh(*mesh, id);
    }

    // 设置网格颜色
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        0.8, 0.6, 0.4,  // 淡棕色
        id
    );

    // 设置网格显示方式为面显示
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
        id
    );
}

void PointCloudViewer::clearAll() {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
}

void PointCloudViewer::resetCamera() {
    viewer_->resetCamera();
}

} // namespace rbf
