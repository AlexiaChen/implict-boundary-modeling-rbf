#include "RBFInterpolator.h"
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

namespace rbf {

RBFInterpolator::RBFInterpolator(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
    const std::vector<double>& distanceValues,
    RBFFunction rbfType)
    : centers_(centers)
    , distanceValues_(distanceValues)
    , lambda_()
    , polyCoeffs_(4, 0.0)
    , rbfType_(rbfType)
    , solved_(false)
{
    if (!centers || centers->empty()) {
        throw std::invalid_argument("Centers cloud is null or empty");
    }

    if (distanceValues.size() != centers->size()) {
        throw std::invalid_argument(
            "Distance values size must match centers size"
        );
    }

    lambda_.resize(centers->size());
}

RBFInterpolator::~RBFInterpolator() = default;

bool RBFInterpolator::solve() {
    int n = static_cast<int>(centers_->size());
    int augN = n + 4;  // 增广系统大小 (N+4) × (N+4)

    // 使用 Eigen 构建并求解增广线性系统
    // ┌   A    P ┐ ┌ λ ┐   ┌ f ┐
    // │          │ │   │ = │   │
    // └  P^T    0 ┘ ┌ c ┐   ┌ 0 ┘

    // 构建增广矩阵 A_aug = [A P; P^T 0]
    Eigen::MatrixXd A_aug(augN, augN);

    // 构建 A 部分 (左上角 N×N)
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const auto& pi = centers_->points[i];
            const auto& pj = centers_->points[j];

            double dx = pi.x - pj.x;
            double dy = pi.y - pj.y;
            double dz = pi.z - pj.z;
            double r = std::sqrt(dx * dx + dy * dy + dz * dz);

            A_aug(i, j) = polyharmonicRBF(r);
        }
    }

    // 构建 P 部分 (右上角 N×4) 和 P^T 部分 (左下角 4×N)
    for (int i = 0; i < n; ++i) {
        const auto& p = centers_->points[i];

        // P_i1 = 1, P_i2 = x_i, P_i3 = y_i, P_i4 = z_i
        A_aug(i, n + 0) = 1.0;
        A_aug(i, n + 1) = p.x;
        A_aug(i, n + 2) = p.y;
        A_aug(i, n + 3) = p.z;

        // P^T - 对称填充
        A_aug(n + 0, i) = 1.0;
        A_aug(n + 1, i) = p.x;
        A_aug(n + 2, i) = p.y;
        A_aug(n + 3, i) = p.z;
    }

    // 右下角 4×4 零矩阵（Eigen默认初始化为0）

    // 构建右侧向量 b_aug = [f; 0]
    Eigen::VectorXd b_aug(augN);
    for (int i = 0; i < n; ++i) {
        b_aug(i) = distanceValues_[i];
    }
    // 多项式约束右侧为 0
    b_aug(n) = 0.0;
    b_aug(n + 1) = 0.0;
    b_aug(n + 2) = 0.0;
    b_aug(n + 3) = 0.0;

    // 使用 Eigen 的 PartialPivLU 求解器（支持多线程）
    Eigen::VectorXd x_aug = A_aug.partialPivLu().solve(b_aug);

    // 提取解向量 x_aug = [λ; c]
    // 前 N 个是 RBF 权重
    for (int i = 0; i < n; ++i) {
        lambda_[i] = x_aug(i);
    }
    // 后 4 个是多项式系数
    for (int i = 0; i < 4; ++i) {
        polyCoeffs_[i] = x_aug(n + i);
    }

    solved_ = true;
    return true;
}

double RBFInterpolator::evaluate(const pcl::PointXYZ& point) const {
    if (!solved_) {
        throw std::runtime_error(
            "RBFInterpolator must be solved before evaluation"
        );
    }

    // s(x) = Σ λ_i * φ(||x - c_i||) + p(x)
    double sum = 0.0;

    // RBF 部分: Σ λ_i * φ(||x - c_i||)
    for (size_t i = 0; i < centers_->size(); ++i) {
        const auto& center = centers_->points[i];

        // 计算欧几里得距离
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        double dz = point.z - center.z;
        double r = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 多谐波 RBF
        sum += lambda_[i] * polyharmonicRBF(r);
    }

    // 多项式部分: p(x) = c_0 + c_1*x + c_2*y + c_3*z
    sum += evaluatePolynomial(point);

    return sum;
}

double RBFInterpolator::polyharmonicRBF(double r) const {
    // 多谐波 RBF:
    // Linear (双谐波): φ(r) = r
    // Cubic (三谐波): φ(r) = r³
    switch (rbfType_) {
        case RBFFunction::Linear:
            return r;
        case RBFFunction::Cubic:
            return r * r * r;
        default:
            return r;
    }
}

void RBFInterpolator::buildAugmentedMatrix(std::vector<double>& A, int n) const {
    // 此函数不再需要，使用Eigen直接构建矩阵
    // 保留接口以避免编译错误
}

double RBFInterpolator::evaluatePolynomial(const pcl::PointXYZ& point) const {
    // p(x) = c_0 + c_1*x + c_2*y + c_3*z
    return polyCoeffs_[0] +
           polyCoeffs_[1] * point.x +
           polyCoeffs_[2] * point.y +
           polyCoeffs_[3] * point.z;
}

bool RBFInterpolator::solveLinearSystem(
    const std::vector<double>& A,
    const std::vector<double>& b,
    std::vector<double>& x,
    int n) const
{
    // 此函数不再需要，使用Eigen直接求解
    // 保留接口以避免编译错误
    return true;
}

} // namespace rbf
