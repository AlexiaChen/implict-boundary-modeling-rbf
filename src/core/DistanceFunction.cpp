#include "DistanceFunction.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

namespace rbf {

std::vector<OffSurfacePoint> DistanceFunction::generateOffSurfacePoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    double offsetDistance)
{
    if (!cloud || cloud->empty()) {
        throw std::invalid_argument("Cloud is null or empty");
    }

    if (!normals || normals->size() != cloud->size()) {
        throw std::invalid_argument(
            "Normals cloud is null or size does not match cloud size"
        );
    }

    std::vector<OffSurfacePoint> offSurfacePoints;
    offSurfacePoints.reserve(cloud->size() * 3);  // 面内点 + 正负离面点

    // 为每个表面点生成：
    // 1. 面内点 (f = 0)
    // 2. 正离面点 (f = +offsetDistance)
    // 3. 负离面点 (f = -offsetDistance)
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = cloud->points[i];
        const auto& normal = normals->points[i];

        // 面内点: f(p_i) = 0
        offSurfacePoints.emplace_back(point, 0.0);

        // 正离面点: p_i + d * n_i, f = +d
        pcl::PointXYZ positivePoint;
        positivePoint.x = point.x + offsetDistance * normal.normal_x;
        positivePoint.y = point.y + offsetDistance * normal.normal_y;
        positivePoint.z = point.z + offsetDistance * normal.normal_z;
        offSurfacePoints.emplace_back(positivePoint, offsetDistance);

        // 负离面点: p_i - d * n_i, f = -d
        pcl::PointXYZ negativePoint;
        negativePoint.x = point.x - offsetDistance * normal.normal_x;
        negativePoint.y = point.y - offsetDistance * normal.normal_y;
        negativePoint.z = point.z - offsetDistance * normal.normal_z;
        offSurfacePoints.emplace_back(negativePoint, -offsetDistance);
    }

    return offSurfacePoints;
}

pcl::PointCloud<pcl::Normal>::Ptr DistanceFunction::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int kSearch)
{
    if (!cloud || cloud->empty()) {
        throw std::invalid_argument("Cloud is null or empty");
    }

    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);

    // 创建 KdTree 用于法向量估计
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>()
    );
    normalEstimation.setSearchMethod(tree);

    // 设置 K 近邻搜索
    normalEstimation.setKSearch(kSearch);

    // 估计法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normalEstimation.compute(*normals);

    return normals;
}

void DistanceFunction::orientNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    if (!cloud || cloud->empty()) {
        throw std::invalid_argument("Cloud is null or empty");
    }

    if (!normals || normals->size() != cloud->size()) {
        throw std::invalid_argument(
            "Normals cloud is null or size does not match cloud size"
        );
    }

    // 计算点云质心
    pcl::PointXYZ centroid(0, 0, 0);
    for (const auto& point : cloud->points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= static_cast<float>(cloud->size());
    centroid.y /= static_cast<float>(cloud->size());
    centroid.z /= static_cast<float>(cloud->size());

    // 一致化法向量方向（全部朝外）
    // 方法：检查法向量与"从质心指向点"向量的点积
    // 如果点积 < 0，说明法向量朝向内部，需要翻转
    for (size_t i = 0; i < normals->size(); ++i) {
        Eigen::Vector3f normal(
            normals->points[i].normal_x,
            normals->points[i].normal_y,
            normals->points[i].normal_z
        );

        Eigen::Vector3f centroidToPoint(
            cloud->points[i].x - centroid.x,
            cloud->points[i].y - centroid.y,
            cloud->points[i].z - centroid.z
        );

        // 如果法向量与"从中心指向点"方向相反，翻转法向量
        if (normal.dot(centroidToPoint) < 0) {
            normals->points[i].normal_x *= -1;
            normals->points[i].normal_y *= -1;
            normals->points[i].normal_z *= -1;
        }
    }
}

double DistanceFunction::computeBoundingBoxDiagonal(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || cloud->empty()) {
        return 0.0;
    }

    // 找到包围盒的min和max
    float minX = cloud->points[0].x;
    float maxX = minX;
    float minY = cloud->points[0].y;
    float maxY = minY;
    float minZ = cloud->points[0].z;
    float maxZ = minZ;

    for (const auto& point : cloud->points) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    // 计算对角线长度
    double dx = maxX - minX;
    double dy = maxY - minY;
    double dz = maxZ - minZ;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double DistanceFunction::euclideanDistance(
    const pcl::PointXYZ& p1,
    const pcl::PointXYZ& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace rbf
