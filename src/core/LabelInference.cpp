#include "LabelInference.h"
#include <pcl/surface/convex_hull.h>
#include <pcl/PointIndices.h>
#include <algorithm>
#include <numeric>
#include <set>

namespace rbf {

std::vector<DomainLabel> LabelInference::infer(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    Method method)
{
    switch (method) {
        case Method::ConvexHull:
            return inferFromConvexHull(cloud);
        case Method::Centroid:
            return inferFromCentroid(cloud);
        default:
            return inferFromConvexHull(cloud);
    }
}

std::vector<DomainLabel> LabelInference::inferFromConvexHull(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) {
        return {};
    }

    // Compute convex hull
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ> hullPoints;
    std::vector<pcl::Vertices> polygons;
    hull.reconstruct(hullPoints, polygons);

    // Get hull point indices
    pcl::PointIndices hullIndices;
    hull.getHullPointIndices(hullIndices);

    // Create a set for fast lookup
    std::set<int> hullIndexSet(hullIndices.indices.begin(), hullIndices.indices.end());

    // Assign labels: hull points = outer (0), inner points = inner (1)
    std::vector<DomainLabel> labels(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (hullIndexSet.find(static_cast<int>(i)) != hullIndexSet.end()) {
            labels[i] = 0;  // Outer domain
        } else {
            labels[i] = 1;  // Inner domain
        }
    }

    return labels;
}

std::vector<DomainLabel> LabelInference::inferFromCentroid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) {
        return {};
    }

    // Compute centroid
    pcl::PointXYZ centroid(0, 0, 0);
    for (const auto& point : cloud->points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= cloud->size();
    centroid.y /= cloud->size();
    centroid.z /= cloud->size();

    // Compute distances from centroid
    std::vector<double> distances;
    distances.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        double dx = point.x - centroid.x;
        double dy = point.y - centroid.y;
        double dz = point.z - centroid.z;
        distances.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
    }

    // Compute median distance
    std::vector<double> sortedDistances = distances;
    std::sort(sortedDistances.begin(), sortedDistances.end());
    double medianDistance = sortedDistances[sortedDistances.size() / 2];

    // Assign labels based on median distance
    std::vector<DomainLabel> labels(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        labels[i] = (distances[i] < medianDistance) ? 1 : 0;
    }

    return labels;
}

} // namespace rbf
