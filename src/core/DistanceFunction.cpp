#include "DistanceFunction.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace rbf {

std::vector<double> DistanceFunction::compute(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<DomainLabel>& labels)
{
    if (!cloud || cloud->empty()) {
        return {};
    }

    if (labels.size() != cloud->size()) {
        throw std::invalid_argument(
            "Labels size must match cloud size"
        );
    }

    std::vector<double> distances;
    distances.reserve(cloud->size());

    // 为每个点计算符号距离
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = cloud->points[i];
        DomainLabel label = labels[i];

        double dist = computeSignedDistance(point, label, cloud, labels);
        distances.push_back(dist);
    }

    return distances;
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

double DistanceFunction::computeSignedDistance(
    const pcl::PointXYZ& point,
    DomainLabel pointLabel,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<DomainLabel>& labels)
{
    double minDistance = std::numeric_limits<double>::max();

    // 查找到最近的不同域样本的距离
    for (size_t i = 0; i < cloud->size(); ++i) {
        // 跳过同一域的点
        if (labels[i] == pointLabel) {
            continue;
        }

        double dist = euclideanDistance(point, cloud->points[i]);
        if (dist < minDistance) {
            minDistance = dist;
        }
    }

    // 如果没有找到不同域的点，返回一个大值
    if (minDistance == std::numeric_limits<double>::max()) {
        minDistance = 1000.0; // 默认大值
    }

    // 根据域标签赋予符号
    // 论文约定：内域 (label=1) 赋负值，外域 (label=0) 赋正值
    if (pointLabel == 1) {
        return -minDistance;
    } else {
        return minDistance;
    }
}

} // namespace rbf
