#ifndef DISTANCE_FUNCTION_H
#define DISTANCE_FUNCTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace rbf {

/**
 * @brief 离面点结构
 *
 * 用于存储沿法线投影生成的离面点及其符号距离值
 */
struct OffSurfacePoint {
    pcl::PointXYZ position;      // 点的位置
    double distanceValue;        // 符号距离函数值 (0 = 面内点, >0 = 外部, <0 = 内部)

    OffSurfacePoint() : distanceValue(0.0) {}

    OffSurfacePoint(const pcl::PointXYZ& pos, double dist)
        : position(pos), distanceValue(dist) {}
};

/**
 * @brief 离面点生成器
 *
 * 基于论文《使用径向基函数（RBF）重建和表示三维物体》
 *
 * 该类负责：
 * 1. 估计点云的法向量
 * 2. 一致化法向量方向（全部朝外）
 * 3. 沿法线投影生成离面点
 *
 * 关键公式：
 * - 面内点: f(p_i) = 0
 * - 正离面点: p_i^+ = p_i + d * n_i, f(p_i^+) = +d
 * - 负离面点: p_i^- = p_i - d * n_i, f(p_i^-) = -d
 */
class DistanceFunction {
public:
    /**
     * @brief 生成离面点
     *
     * 为每个表面点生成两个离面点（正负方向），加上原始面内点
     *
     * @param cloud 输入点云（表面点）
     * @param normals 法向量点云（必须与点云大小一致）
     * @param offsetDistance 投影距离，通常取包围盒对角线长度的 0.01 倍
     * @return std::vector<OffSurfacePoint> 包含面内点和离面点的列表
     */
    static std::vector<OffSurfacePoint> generateOffSurfacePoints(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        double offsetDistance
    );

    /**
     * @brief 估计点云法向量
     *
     * 使用 PCL 的 NormalEstimation 进行法向量估计
     *
     * @param cloud 输入点云
     * @param kSearch K近邻数，用于法向量估计
     * @return pcl::PointCloud<pcl::Normal>::Ptr 估计的法向量点云
     */
    static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int kSearch = 30
    );

    /**
     * @brief 一致化法向量方向
     *
     * 确保所有法向量指向点云外部（远离质心）
     * 方法：计算法向量与"从质心指向点"向量的点积，若为负则翻转
     *
     * @param cloud 输入点云
     * @param normals 法向量点云（会被修改）
     */
    static void orientNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointCloud<pcl::Normal>::Ptr& normals
    );

    /**
     * @brief 计算点云包围盒的对角线长度
     *
     * @param cloud 输入点云
     * @return double 包围盒对角线长度
     */
    static double computeBoundingBoxDiagonal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

private:
    /**
     * @brief 计算两点之间的欧几里得距离
     */
    static double euclideanDistance(
        const pcl::PointXYZ& p1,
        const pcl::PointXYZ& p2
    );
};

} // namespace rbf

#endif // DISTANCE_FUNCTION_H
