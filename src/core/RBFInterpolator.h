#ifndef RBF_INTERPOLATOR_H
#define RBF_INTERPOLATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Sparse>
#include <vector>
#include <memory>
#include <functional>

namespace rbf {

/**
 * @brief RBF 插值器（基于多谐波RBF）
 *
 * 实现基于论文《使用径向基函数（RBF）重建和表示三维物体》
 *
 * 核心思想：
 * - 使用多谐波径向基函数 φ(r) = r 或 r³
 * - 带线性多项式约束，确保解的唯一性和稳定性
 * - 求解增广线性系统：[A P; P^T 0] [λ; c] = [f; 0]
 *
 * 插值函数形式：
 * s(x) = Σ λ_i * φ(||x - c_i||) + c_0 + c_1*x + c_2*y + c_3*z
 *
 * 其中：
 * - φ(r) 是多谐波RBF（线性或三次）
 * - λ_i 是RBF权重
 * - c_0, c_1, c_2, c_3 是多项式系数
 */
class RBFInterpolator {
public:
    /**
     * @brief 进度回调函数类型
     *
     * @param current 当前进度 (0-100)
     * @param total 总进度 (100)
     * @param message 进度消息
     */
    using ProgressCallback = std::function<void(int current, int total, const std::string& message)>;

    /**
     * @brief RBF 函数类型
     */
    enum class RBFFunction {
        Linear,  // φ(r) = r (双谐波，推荐，最常用)
        Cubic    // φ(r) = r³ (三谐波，更光滑)
    };

    enum class KernelSupport {
        Global,
        Compact
    };

    /**
     * @brief 构造函数
     *
     * @param centers RBF 中心点（包含面内点和离面点）
     * @param distanceValues 对应的符号距离函数值
     * @param rbfType RBF 函数类型
     */
    RBFInterpolator(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
        const std::vector<double>& distanceValues,
        RBFFunction rbfType = RBFFunction::Linear
    );

    ~RBFInterpolator();

    /**
     * @brief 求解器配置
     *
     * useFastMultipole: 使用近似多极加速（稀疏/迭代）
     * neighborRadius: 稀疏半径（<=0 自动按包围盒对角线百分比计算）
     * maxNeighbors: 邻域上限（0 表示仅按半径搜索）
     * useCompactSupport: 使用紧凑支撑核（Wendland C2），核函数在 supportRadius 外为 0
     * supportRadius: 紧凑支撑半径（<=0 自动按包围盒对角线百分比计算）
     */
    struct SolverOptions {
        bool useFastMultipole = false;
        double neighborRadius = -1.0;
        int maxNeighbors = 64;
        bool useCompactSupport = false;
        double supportRadius = -1.0;
    };

    /**
     * @brief 设置进度回调函数
     *
     * @param callback 进度回调函数
     */
    void setProgressCallback(ProgressCallback callback) { progressCallback_ = callback; }

    /**
     * @brief 求解增广线性系统
     *
     * 求解：
     * ┌   A    P ┐ ┌ λ ┐   ┌ f ┐
     * │          │ │   │ = │   │
     * └  P^T    0 ┘ ┌ c ┐   ┌ 0 ┐
     *
     * 其中：
     * - A_ij = φ(||c_i - c_j||),  i,j = 1..N
     * - P_i1 = 1,  P_i2 = x_i,  P_i3 = y_i,  P_i4 = z_i
     * - λ: RBF权重（N个）
     * - c: 多项式系数（4个: c0, c1, c2, c3）
     *
     * @return true 求解成功
     * @return false 求解失败
     */
    bool solve();

    /**
     * @brief 在任意位置评估插值函数
     *
     * s(x) = Σ λ_i * φ(||x - c_i||) + c_0 + c_1*x + c_2*y + c_3*z
     *
     * @param point 评估位置
     * @return double 插值结果（符号距离）
     */
    double evaluate(const pcl::PointXYZ& point) const;

    /**
     * @brief 获取 RBF 权重向量
     */
    const std::vector<double>& getWeights() const { return lambda_; }

    /**
     * @brief 获取多项式系数 [c0, c1, c2, c3]
     */
    const std::vector<double>& getPolynomialCoefficients() const { return polyCoeffs_; }

    /**
     * @brief 设置 RBF 函数类型
     */
    void setRBFFunction(RBFFunction type) { rbfType_ = type; }

    /**
     * @brief 设置求解选项
     */
    void setSolverOptions(const SolverOptions& options) { solverOptions_ = options; }

    /**
     * @brief 获取求解选项
     */
    SolverOptions getSolverOptions() const { return solverOptions_; }

    /**
     * @brief 获取 RBF 函数类型
     */
    RBFFunction getRBFFunction() const { return rbfType_; }

    /**
     * @brief 获取中心点点云（用于边界框计算等）
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenters() const { return centers_; }

    /**
     * @brief 获取是否已求解
     */
    bool isSolved() const { return solved_; }

private:
    /**
     * @brief 统一的核函数入口，自动考虑紧凑支撑
     */
    double radialBasis(double r) const;

    /**
     * @brief 计算多谐波 RBF 值
     *
     * Linear (双谐波): φ(r) = r
     * Cubic (三谐波): φ(r) = r³
     */
    double polyharmonicRBF(double r) const;

    /**
     * @brief 构建增广矩阵
     *
     * 构建 (N+4) × (N+4) 的增广矩阵：
     * ┌   A    P ┐
     * │          │
     * └  P^T    0 ┘
     */
    void buildAugmentedMatrix(std::vector<double>& A, int n) const;

    /**
     * @brief 评估多项式项
     *
     * p(x) = c_0 + c_1*x + c_2*y + c_3*z
     */
    double evaluatePolynomial(const pcl::PointXYZ& point) const;

    /**
     * @brief 使用 LAPACK 求解线性系统
     */
    bool solveLinearSystem(
        const std::vector<double>& A,
        const std::vector<double>& b,
        std::vector<double>& x,
        int n
    ) const;

    bool solveDense();
    bool solveWithFastMultipole();
    double estimateNeighborRadiusFromBBox() const;
    double estimateSupportRadiusFromBBox() const;
    double calibrateSupportRadius(int kth) const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_;     // RBF 中心点
    std::vector<double> distanceValues_;              // 符号距离函数值
    std::vector<double> lambda_;                      // RBF 权重 (N个)
    std::vector<double> polyCoeffs_;                  // 多项式系数 (4个)
    RBFFunction rbfType_;                             // RBF 类型
    SolverOptions solverOptions_;                     // 求解配置
    bool solved_;                                     // 是否已求解
    double cachedSupportRadius_;                      // 缓存的支撑半径
    ProgressCallback progressCallback_;                // 进度回调函数
};

} // namespace rbf

#endif // RBF_INTERPOLATOR_H
