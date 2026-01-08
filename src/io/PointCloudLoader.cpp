#include "PointCloudLoader.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rbf {

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudLoader::load(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 根据扩展名选择加载方式
    std::string ext = filename.substr(filename.find_last_of('.') + 1);

    if (ext == "pcd" || ext == "PCD") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
            throw std::runtime_error("Failed to load PCD file: " + filename);
        }
    } else if (ext == "ply" || ext == "PLY") {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) {
            throw std::runtime_error("Failed to load PLY file: " + filename);
        }
    } else if (ext == "xyz" || ext == "XYZ") {
        // 手动解析 XYZ 格式
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open XYZ file: " + filename);
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;  // 跳过空行和注释
            }

            std::istringstream iss(line);
            pcl::PointXYZ point;
            if (iss >> point.x >> point.y >> point.z) {
                cloud->push_back(point);
            }
        }
    } else {
        throw std::runtime_error("Unsupported file format: " + ext);
    }

    if (cloud->empty()) {
        throw std::runtime_error("Loaded empty point cloud from: " + filename);
    }

    return cloud;
}

bool PointCloudLoader::loadWithLabels(
    const std::string& cloudFile,
    const std::string& labelFile,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<DomainLabel>& labels)
{
    try {
        cloud = load(cloudFile);
        labels = loadLabels(labelFile);

        if (labels.size() != cloud->size()) {
            throw std::runtime_error(
                "Label count (" + std::to_string(labels.size()) +
                ") does not match point cloud size (" +
                std::to_string(cloud->size()) + ")"
            );
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading point cloud with labels: " << e.what() << std::endl;
        return false;
    }
}

bool PointCloudLoader::save(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& filename)
{
    std::string ext = filename.substr(filename.find_last_of('.') + 1);

    if (ext == "pcd" || ext == "PCD") {
        return pcl::io::savePCDFileBinary(filename, *cloud) == 0;
    } else if (ext == "ply" || ext == "PLY") {
        return pcl::io::savePLYFileBinary(filename, *cloud) == 0;
    } else if (ext == "xyz" || ext == "XYZ") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        for (const auto& point : cloud->points) {
            file << point.x << " " << point.y << " " << point.z << "\n";
        }

        return true;
    }

    return false;
}

std::vector<DomainLabel> PointCloudLoader::loadLabels(const std::string& filename) {
    std::vector<DomainLabel> labels;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open label file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;  // 跳过空行和注释
        }

        std::istringstream iss(line);
        int label;
        if (iss >> label) {
            if (label == 0 || label == 1) {
                labels.push_back(label);
            } else {
                throw std::runtime_error(
                    "Invalid label value: " + std::to_string(label) +
                    " (expected 0 or 1)"
                );
            }
        }
    }

    return labels;
}

} // namespace rbf
