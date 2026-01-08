#include "RBFInterpolator.h"
#include <cmath>
#include <cstring>
#include <stdexcept>

// LAPACK 函数声明 (用于求解线性方程组)
extern "C" {
    // DGESV: 求解一般线性系统 AX = B
    void dgesv_(
        int* n,           // 矩阵阶数
        int* nrhs,        // 右侧向量数量
        double* A,        // 系数矩阵
        int* lda,         // A 的 leading dimension
        int* ipiv,        // pivot 索引
        double* B,        // 右侧向量/解向量
        int* ldb,         // B 的 leading dimension
        int* info         // 返回信息
    );
}

namespace rbf {

RBFInterpolator::RBFInterpolator(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<double>& distanceValues,
    double epsilon)
    : cloud_(cloud)
    , distanceValues_(distanceValues)
    , weights_()
    , epsilon_(epsilon)
    , solved_(false)
{
    if (!cloud || cloud->empty()) {
        throw std::invalid_argument("Cloud is null or empty");
    }

    if (distanceValues.size() != cloud->size()) {
        throw std::invalid_argument(
            "Distance values size must match cloud size"
        );
    }

    weights_.resize(cloud->size());
}

RBFInterpolator::~RBFInterpolator() = default;

bool RBFInterpolator::solve() {
    int n = static_cast<int>(cloud_->size());

    // 构建 RBF 矩阵 A (N x N, 按列优先存储)
    std::vector<double> A(n * n);
    buildMatrix(A, n);

    // 准备右侧向量 b (distanceValues 的副本)
    std::vector<double> b(distanceValues_);

    // 使用 LAPACK 求解 AX = b
    bool success = solveLinearSystem(A, b, weights_, n);

    if (success) {
        solved_ = true;
    }

    return solved_;
}

double RBFInterpolator::evaluate(const pcl::PointXYZ& point) const {
    if (!solved_) {
        throw std::runtime_error(
            "RBFInterpolator must be solved before evaluation"
        );
    }

    double sum = 0.0;

    // s(u) = Σ w_i * φ(r(u, u_i))
    for (size_t i = 0; i < cloud_->size(); ++i) {
        const auto& cloudPoint = cloud_->points[i];

        // 计算欧几里得距离
        double dx = point.x - cloudPoint.x;
        double dy = point.y - cloudPoint.y;
        double dz = point.z - cloudPoint.z;
        double r = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 高斯 RBF: φ(r) = exp(-ε² * r²)
        sum += weights_[i] * gaussianRBF(r);
    }

    return sum;
}

double RBFInterpolator::gaussianRBF(double r) const {
    // φ(r) = exp(-ε² * r²)
    double eps_r = epsilon_ * r;
    return std::exp(-eps_r * eps_r);
}

void RBFInterpolator::buildMatrix(std::vector<double>& A, int n) const {
    // 构建 RBF 矩阵: A_ij = φ(||u_i - u_j||)
    // 按列优先存储 (Fortran 风格，用于 LAPACK)
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            // 计算点 i 和点 j 之间的距离
            const auto& pi = cloud_->points[i];
            const auto& pj = cloud_->points[j];

            double dx = pi.x - pj.x;
            double dy = pi.y - pj.y;
            double dz = pi.z - pj.z;
            double r = std::sqrt(dx * dx + dy * dy + dz * dz);

            // A_ij = φ(r) = exp(-ε² * r²)
            A[j * n + i] = gaussianRBF(r);
        }
    }
}

bool RBFInterpolator::solveLinearSystem(
    const std::vector<double>& A,
    const std::vector<double>& b,
    std::vector<double>& x,
    int n) const
{
    // 准备 LAPACK 求解
    std::vector<double> A_copy = A;  // DGESV 会修改 A
    std::vector<double> b_copy = b;  // DGESV 会把解写入 b

    int nrhs = 1;        // 右侧向量数量
    int lda = n;         // A 的 leading dimension
    int ldb = n;         // B 的 leading dimension
    int info = 0;        // 返回信息

    std::vector<int> ipiv(n);  // pivot 索引数组

    // 调用 LAPACK 的 DGESV 求解 AX = b
    dgesv_(&n, &nrhs, A_copy.data(), &lda, ipiv.data(),
           b_copy.data(), &ldb, &info);

    if (info != 0) {
        // info < 0: 参数错误
        // info > 0: 矩阵奇异
        return false;
    }

    // 解在 b_copy 中
    x = b_copy;
    return true;
}

} // namespace rbf
