#ifndef DISTANCE_FUNCTION_H
#define DISTANCE_FUNCTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace rbf {

/**
 * @brief 域指示符类型
 */
using DomainLabel = int;

/**
 * @brief 符号距离函数计算器
 *
 * 根据论文公式计算符号距离函数：
 * DF(u_i) = {
 *     -argmin(r)  if I(u_i) = 1 (内域)
 *     +argmin(r)  if I(u_i) = 0 (外域)
 * }
 *
 * 其中 r(u_i, u_j) = sqrt((x_i-x_j)^2 + (y_i-y_j)^2 + (z_i-z_j)^2)
 */
class DistanceFunction {
public:
    /**
     * @brief 计算所有点的符号距离函数
     *
     * @param cloud 输入点云
     * @param labels 每个点对应的域标签 (0 或 1)
     * @return std::vector<double> 符号距离函数值
     */
    static std::vector<double> compute(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<DomainLabel>& labels
    );

    /**
     * @brief 计算两点之间的欧几里得距离
     */
    static double euclideanDistance(
        const pcl::PointXYZ& p1,
        const pcl::PointXYZ& p2
    );

private:
    /**
     * @brief 为单个点计算到最近不同域样本的符号距离
     */
    static double computeSignedDistance(
        const pcl::PointXYZ& point,
        DomainLabel pointLabel,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<DomainLabel>& labels
    );
};

} // namespace rbf

#endif // DISTANCE_FUNCTION_H
