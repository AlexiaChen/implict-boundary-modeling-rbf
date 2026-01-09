#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H

#include "RBFInterpolator.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <memory>
#include <unordered_map>

namespace rbf {

/**
 * @brief Marching Cubes 等值面提取器
 *
 * 使用 RBF 插值器作为隐式函数，提取零等值面 (s(u) = 0)
 */
class MarchingCubes {
public:
    /**
     * @brief 构造函数
     *
     * @param interpolator RBF 插值器 (已求解)
     * @param resolution 体素网格分辨率 (每个维度上的体素数)
     */
    MarchingCubes(
        const std::shared_ptr<RBFInterpolator>& interpolator,
        int resolution = 100
    );

    ~MarchingCubes() = default;

    /**
     * @brief 提取零等值面网格
     *
     * @return pcl::PolygonMesh 提取的三角网格
     */
    pcl::PolygonMesh extract();

    /**
     * @brief 设置体素分辨率
     */
    void setResolution(int res) { resolution_ = res; }

    /**
     * @brief 获取体素分辨率
     */
    int getResolution() const { return resolution_; }

private:
    /**
     * @brief 计算点云的包围盒
     */
    void computeBoundingBox();

    /**
     * @brief 在体素网格上评估隐式函数
     */
    std::vector<double> evaluateOnGrid();

    /**
     * @brief 处理单个立方体，生成三角形
     */
    void processCube(
        int ix, int iy, int iz,
        const std::vector<double>& gridValues,
        pcl::PointCloud<pcl::PointXYZ>& cloud,
        std::vector<pcl::Vertices>& triangles,
        std::unordered_map<uint64_t, int>& edgeVertexCache
    );

    /**
     * @brief 在边上插值找到零点位置
     */
    pcl::PointXYZ interpolateEdge(
        const pcl::PointXYZ& p0,
        const pcl::PointXYZ& p1,
        double v0,
        double v1
    ) const;

    /**
     * @brief 获取或创建边顶点（带缓存，确保水密性）
     */
    int getOrCreateEdgeVertex(
        int ix, int iy, int iz,
        int edge,
        const std::vector<double>& gridValues,
        pcl::PointCloud<pcl::PointXYZ>& cloud,
        std::unordered_map<uint64_t, int>& edgeVertexCache
    );

    /**
     * @brief 计算边的唯一键（基于两个网格顶点索引）
     */
    uint64_t getEdgeKey(int ix, int iy, int iz, int edge) const;

private:
    std::shared_ptr<RBFInterpolator> interpolator_;
    int resolution_;

    // 包围盒
    Eigen::Vector3f minBound_;
    Eigen::Vector3f maxBound_;
    Eigen::Vector3f voxelSize_;
};

} // namespace rbf

#endif // MARCHING_CUBES_H
