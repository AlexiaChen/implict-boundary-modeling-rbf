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

} // namespace rbf
