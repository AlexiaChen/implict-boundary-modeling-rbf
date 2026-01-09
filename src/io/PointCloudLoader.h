#ifndef POINT_CLOUD_LOADER_H
#define POINT_CLOUD_LOADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace rbf {

/**
 * @brief 点云加载器
 *
 * 支持加载常见点云格式 (PCD, PLY, XYZ)
 */
class PointCloudLoader {
public:
    /**
     * @brief 加载点云文件
     *
     * @param filename 文件路径
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 加载的点云
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr load(const std::string& filename);

    /**
     * @brief 保存点云到文件
     */
    static bool save(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& filename
    );
};

} // namespace rbf

#endif // POINT_CLOUD_LOADER_H
