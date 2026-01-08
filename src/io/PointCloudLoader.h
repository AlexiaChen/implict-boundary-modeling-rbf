#ifndef POINT_CLOUD_LOADER_H
#define POINT_CLOUD_LOADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include "core/DistanceFunction.h"

namespace rbf {

/**
 * @brief 点云加载器
 *
 * 支持加载常见点云格式 (PCD, PLY, XYZ) 及其域标签
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
     * @brief 加载点云和域标签
     *
     * 域标签文件格式: 每行一个标签 (0 或 1)，与点云中的点一一对应
     *
     * @param cloudFile 点云文件路径
     * @param labelFile 标签文件路径
     * @param cloud 输出点云
     * @param labels 输出域标签
     * @return true 加载成功
     * @return false 加载失败
     */
    static bool loadWithLabels(
        const std::string& cloudFile,
        const std::string& labelFile,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        std::vector<DomainLabel>& labels
    );

    /**
     * @brief 保存点云到文件
     */
    static bool save(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& filename
    );

    /**
     * @brief 从文本文件加载标签
     *
     * @param filename 文件路径
     * @return std::vector<DomainLabel> 标签向量
     */
    static std::vector<DomainLabel> loadLabels(const std::string& filename);
};

} // namespace rbf

#endif // POINT_CLOUD_LOADER_H
