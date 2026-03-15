#include "RBFInterpolator.h"
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/SparseLU>
#include <QString>
#include <limits>
#include <algorithm>

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
    , solverOptions_()
    , solved_(false)
    , progressCallback_(nullptr)
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
    if (solverOptions_.useFastMultipole) {
        if (progressCallback_) {
            progressCallback_(0, 100, "Using fast multipole-style sparse solver...");
        }
        bool ok = solveWithFastMultipole();
        if (ok) {
            return true;
        }
        if (progressCallback_) {
            progressCallback_(0, 100, "Fallback to dense solver after sparse attempt failed...");
        }
    }
    return solveDense();
}

bool RBFInterpolator::solveDense() {
    int n = static_cast<int>(centers_->size());
    int augN = n + 4;  // 增广系统大小 (N+4) × (N+4)

    // 使用 Eigen 构建并求解增广线性系统
    // ┌   A    P ┐ ┌ λ ┐   ┌ f ┐
    // │          │ │   │ = │   │
    // └  P^T    0 ┘ ┌ c ┐   ┌ 0 ┘

    // 构建增广矩阵 A_aug = [A P; P^T 0]
    Eigen::MatrixXd A_aug(augN, augN);

    // 进度报告：开始构建A矩阵 (0-30%)
    if (progressCallback_) {
        progressCallback_(0, 100, "Building RBF matrix A...");
    }

    // 构建 A 部分 (左上角 N×N) - 这是最耗时的双重循环
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

        // 每10%报告一次进度
        if (progressCallback_ && (j % std::max(1, n / 10)) == 0) {
            int progress = static_cast<int>(30.0 * j / n);
            progressCallback_(
                progress,
                100,
                QString("Building matrix A: %1/%2").arg(j).arg(n).toStdString()
            );
        }
    }

    // 进度报告：A矩阵完成 (30%)
    if (progressCallback_) {
        progressCallback_(30, 100, "Matrix A complete, adding P matrix...");
    }

    // 构建 P 部分 (右上角 N×4) 和 P^T 部分 (左下角 4×N) - 30-40%
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

        // 每25%报告一次进度
        if (progressCallback_ && (i % std::max(1, n / 4)) == 0) {
            int progress = 30 + static_cast<int>(10.0 * i / n);
            progressCallback_(
                progress,
                100,
                QString("Adding P matrix: %1/%2").arg(i).arg(n).toStdString()
            );
        }
    }

    // 进度报告：矩阵构建完成 (40%)
    if (progressCallback_) {
        progressCallback_(40, 100, "Matrix complete, building RHS vector...");
    }

    // 右下角 4×4 零矩阵（Eigen默认初始化为0）

    // 构建右侧向量 b_aug = [f; 0] - 40-45%
    Eigen::VectorXd b_aug(augN);
    for (int i = 0; i < n; ++i) {
        b_aug(i) = distanceValues_[i];
    }
    // 多项式约束右侧为 0
    b_aug(n) = 0.0;
    b_aug(n + 1) = 0.0;
    b_aug(n + 2) = 0.0;
    b_aug(n + 3) = 0.0;

    // 进度报告：开始LU分解 (45-100%)
    if (progressCallback_) {
        progressCallback_(45, 100, "Starting LU decomposition (this may take a while)...");
    }

    // 使用 Eigen 的 PartialPivLU 求解器（支持多线程）
    Eigen::VectorXd x_aug = A_aug.partialPivLu().solve(b_aug);

    // 进度报告：求解完成 (100%)
    if (progressCallback_) {
        progressCallback_(100, 100, "Linear system solved!");
    }

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

bool RBFInterpolator::solveWithFastMultipole() {
    int n = static_cast<int>(centers_->size());
    int augN = n + 4;

    if (n == 0) {
        return false;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(centers_);

    double radius = solverOptions_.neighborRadius;
    if (radius <= 0.0) {
        radius = estimateNeighborRadiusFromBBox();
    }

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n) * std::max(16, solverOptions_.maxNeighbors + 4));

    if (progressCallback_) {
        progressCallback_(5, 100, QString("Building sparse RBF matrix (r=%.4f)...").arg(radius).toStdString());
    }

    std::vector<int> idx;
    std::vector<float> dist2;
    idx.reserve(static_cast<size_t>(std::max(8, solverOptions_.maxNeighbors)));
    dist2.reserve(idx.size());

    for (int i = 0; i < n; ++i) {
        const auto& pi = centers_->points[i];

        int found = 0;
        if (solverOptions_.maxNeighbors > 0) {
            found = kdtree.nearestKSearch(pi, solverOptions_.maxNeighbors, idx, dist2);
        } else {
            found = kdtree.radiusSearch(pi, radius, idx, dist2);
        }

        for (int k = 0; k < found; ++k) {
            int j = idx[k];
            if (j >= augN) {
                continue;
            }

            double r = std::sqrt(static_cast<double>(dist2[k]));
            double value = polyharmonicRBF(r);

            // 对称填充，避免重复的自环
            if (i == j) {
                triplets.emplace_back(i, j, value);
            } else {
                triplets.emplace_back(i, j, value);
                triplets.emplace_back(j, i, value);
            }
        }

        // P / P^T
        triplets.emplace_back(i, n + 0, 1.0);
        triplets.emplace_back(i, n + 1, pi.x);
        triplets.emplace_back(i, n + 2, pi.y);
        triplets.emplace_back(i, n + 3, pi.z);

        triplets.emplace_back(n + 0, i, 1.0);
        triplets.emplace_back(n + 1, i, pi.x);
        triplets.emplace_back(n + 2, i, pi.y);
        triplets.emplace_back(n + 3, i, pi.z);

        if (progressCallback_ && (i % std::max(1, n / 10)) == 0) {
            int progress = 5 + static_cast<int>(25.0 * i / n);
            progressCallback_(
                progress,
                100,
                QString("Sparse assembly %1/%2").arg(i).arg(n).toStdString()
            );
        }
    }

    Eigen::SparseMatrix<double> A_aug(augN, augN);
    A_aug.setFromTriplets(triplets.begin(), triplets.end());
    A_aug.makeCompressed();

    Eigen::VectorXd b_aug(augN);
    for (int i = 0; i < n; ++i) {
        b_aug(i) = distanceValues_[i];
    }
    b_aug.tail(4).setZero();

    if (progressCallback_) {
        progressCallback_(35, 100, "Running SparseLU factorization...");
    }

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.analyzePattern(A_aug);
    solver.factorize(A_aug);

    if (solver.info() != Eigen::Success) {
        return false;
    }

    Eigen::VectorXd x_aug = solver.solve(b_aug);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    if (progressCallback_) {
        progressCallback_(90, 100, "Sparse solve complete, extracting weights...");
    }

    for (int i = 0; i < n; ++i) {
        lambda_[i] = x_aug(i);
    }
    for (int i = 0; i < 4; ++i) {
        polyCoeffs_[i] = x_aug(n + i);
    }

    solved_ = true;
    if (progressCallback_) {
        progressCallback_(100, 100, "Fast multipole-style solve finished");
    }
    return true;
}

double RBFInterpolator::estimateNeighborRadiusFromBBox() const {
    double minX = std::numeric_limits<double>::max();
    double minY = minX;
    double minZ = minX;
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = maxX;
    double maxZ = maxX;

    for (const auto& p : centers_->points) {
        minX = std::min(minX, static_cast<double>(p.x));
        minY = std::min(minY, static_cast<double>(p.y));
        minZ = std::min(minZ, static_cast<double>(p.z));
        maxX = std::max(maxX, static_cast<double>(p.x));
        maxY = std::max(maxY, static_cast<double>(p.y));
        maxZ = std::max(maxZ, static_cast<double>(p.z));
    }

    double dx = maxX - minX;
    double dy = maxY - minY;
    double dz = maxZ - minZ;
    double diag = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 缺省使用 3% 的对角线作为邻域半径
    return 0.03 * diag;
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
