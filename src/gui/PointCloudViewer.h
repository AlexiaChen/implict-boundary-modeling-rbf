#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>

namespace rbf {

/**
 * @brief Point cloud and mesh viewer embedded in Qt
 *
 * Uses QVTKOpenGLNativeWidget to embed PCL Visualizer directly
 * into the Qt window for seamless integration.
 */
class PointCloudViewer : public QWidget {
    Q_OBJECT

public:
    explicit PointCloudViewer(QWidget* parent = nullptr);
    ~PointCloudViewer();

    /**
     * @brief Display point cloud
     */
    void showPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& id = "cloud"
    );

    /**
     * @brief Display mesh
     */
    void showMesh(
        const pcl::PolygonMesh::Ptr& mesh,
        const std::string& id = "mesh"
    );

    /**
     * @brief Clear all displayed objects
     */
    void clearAll();

    /**
     * @brief Reset camera view
     */
    void resetCamera();

private:
    void setupUI();

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    QVTKOpenGLNativeWidget* vtkWidget_;
};

} // namespace rbf

#endif // POINT_CLOUD_VIEWER_H
