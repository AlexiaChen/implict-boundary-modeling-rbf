#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QWidget>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>

namespace rbf {

/**
 * @brief 点云和网格查看器
 *
 * 使用独立的 PCL Visualizer 窗口显示点云和重建的网格
 * 注意：PCL Visualizer 在 VTK 9 中创建独立的渲染窗口
 */
class PointCloudViewer : public QWidget {
    Q_OBJECT

public:
    explicit PointCloudViewer(QWidget* parent = nullptr);
    ~PointCloudViewer();

    /**
     * @brief 显示点云
     */
    void showPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& id = "cloud"
    );

    /**
     * @brief 显示网格
     */
    void showMesh(
        const pcl::PolygonMesh::Ptr& mesh,
        const std::string& id = "mesh"
    );

    /**
     * @brief 清除所有显示对象
     */
    void clearAll();

    /**
     * @brief 重置相机视角
     */
    void resetCamera();

private:
    void setupUI();

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

} // namespace rbf

#endif // POINT_CLOUD_VIEWER_H
