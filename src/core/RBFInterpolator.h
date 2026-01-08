#ifndef RBF_INTERPOLATOR_H
#define RBF_INTERPOLATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

namespace rbf {

/**
 * @brief RBF 插值器
 *
 * 实现基于高斯径向基函数的隐式场插值：
 * s(u) = Σ w_i * φ(r(u, u_i))
 *
 * 其中 φ(r) = exp(-ε² * r²) 为高斯 RBF
 *
 * 权重通过求解线性系统获得：A * w = f
 * 其中 A_ij = φ(r(u_i, u_j))
 */
class RBFInterpolator {
public:
    /**
     * @brief 构造函数
     *
     * @param cloud 输入点云
     * @param distanceValues 符号距离函数值
     * @param epsilon RBF 形状参数 (控制影响半径)
     */
    RBFInterpolator(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<double>& distanceValues,
        double epsilon = 0.1
    );

    ~RBFInterpolator();

    /**
     * @brief 计算插值权重 (求解线性系统)
     *
     * @return true 求解成功
     * @return false 求解失败
     */
    bool solve();

    /**
     * @brief 在任意位置评估插值函数
     *
     * @param point 评估位置
     * @return double 插值结果 (符号距离)
     */
    double evaluate(const pcl::PointXYZ& point) const;

    /**
     * @brief 获取权重向量
     */
    const std::vector<double>& getWeights() const { return weights_; }

    /**
     * @brief 设置 epsilon 参数
     */
    void setEpsilon(double epsilon) { epsilon_ = epsilon; }

    /**
     * @brief 获取 epsilon 参数
     */
    double getEpsilon() const { return epsilon_; }

private:
    /**
     * @brief 计算高斯 RBF 值
     * φ(r) = exp(-ε² * r²)
     */
    double gaussianRBF(double r) const;

    /**
     * @brief 构建RBF矩阵 A
     * A_ij = φ(||u_i - u_j||)
     */
    void buildMatrix(std::vector<double>& A, int n) const;

    /**
     * @brief 使用 OpenBLAS/LAPACK 求解线性系统
     */
    bool solveLinearSystem(
        const std::vector<double>& A,
        const std::vector<double>& b,
        std::vector<double>& x,
        int n
    ) const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::vector<double> distanceValues_;  // 符号距离函数值
    std::vector<double> weights_;         // RBF 权重
    double epsilon_;                      // RBF 形状参数
    bool solved_;                         // 是否已求解
};

} // namespace rbf

#endif // RBF_INTERPOLATOR_H
