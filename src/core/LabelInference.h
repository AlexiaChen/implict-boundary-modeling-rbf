#ifndef LABEL_INFERENCE_H
#define LABEL_INFERENCE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

namespace rbf {

/**
 * @brief Domain label type
 */
using DomainLabel = int;

/**
 * @brief Automatic domain label inference from point cloud
 *
 * For point clouds without explicit domain labels, this class provides
 * methods to automatically infer domain labels based on geometric features.
 *
 * Supported methods:
 * - ConvexHull: Points on convex hull are outer domain (0), inner points are inner domain (1)
 * - Centroid: Points closer to centroid are inner domain (1), farther points are outer domain (0)
 */
class LabelInference {
public:
    enum class Method {
        ConvexHull,    // Based on convex hull (suitable for closed objects)
        Centroid       // Based on distance to centroid (simple but less accurate)
    };

    /**
     * @brief Infer domain labels using the specified method
     *
     * @param cloud Input point cloud
     * @param method Inference method
     * @return std::vector<DomainLabel> Inferred domain labels (0 or 1)
     */
    static std::vector<DomainLabel> infer(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Method method = Method::ConvexHull
    );

    /**
     * @brief Infer labels using convex hull method
     *
     * Algorithm:
     * 1. Compute convex hull of the point cloud
     * 2. Points on hull vertices -> outer domain (0)
     * 3. Points inside hull -> inner domain (1)
     *
     * @param cloud Input point cloud
     * @return std::vector<DomainLabel> Inferred labels
     */
    static std::vector<DomainLabel> inferFromConvexHull(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );

    /**
     * @brief Infer labels using centroid-based method
     *
     * Algorithm:
     * 1. Compute centroid of point cloud
     * 2. Compute median distance from centroid
     * 3. Points closer than median -> inner domain (1)
     * 4. Points farther than median -> outer domain (0)
     *
     * @param cloud Input point cloud
     * @return std::vector<DomainLabel> Inferred labels
     */
    static std::vector<DomainLabel> inferFromCentroid(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
};

} // namespace rbf

#endif // LABEL_INFERENCE_H
