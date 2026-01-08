/**
 * Test Data Generator for RBF Implicit Boundary Reconstruction
 *
 * Scenario: Two concentric spheres
 * - Inner domain (label=1): points inside sphere of radius 1.0
 * - Outer domain (label=0): points outside sphere of radius 2.0
 * - Expected boundary: sphere at radius ~1.5
 */

#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <vector>

struct Point {
    double x, y, z;
};

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-3.0, 3.0);

    const int NUM_INNER_POINTS = 500;
    const int NUM_OUTER_POINTS = 500;
    const double INNER_RADIUS = 1.0;
    const double OUTER_RADIUS = 2.0;

    std::vector<Point> points;
    std::vector<int> labels;

    // Generate inner domain points (label=1)
    int innerCount = 0;
    while (innerCount < NUM_INNER_POINTS) {
        Point p;
        p.x = dis(gen);
        p.y = dis(gen);
        p.z = dis(gen);

        double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

        if (r < INNER_RADIUS) {
            points.push_back(p);
            labels.push_back(1);
            innerCount++;
        }
    }

    // Generate outer domain points (label=0)
    int outerCount = 0;
    while (outerCount < NUM_OUTER_POINTS) {
        Point p;
        p.x = dis(gen);
        p.y = dis(gen);
        p.z = dis(gen);

        double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

        if (r > OUTER_RADIUS) {
            points.push_back(p);
            labels.push_back(0);
            outerCount++;
        }
    }

    // Write PCD file
    std::ofstream pcdFile("two_spheres.pcd");
    pcdFile << "# .PCD v0.7 - Point Cloud Data file format\n";
    pcdFile << "VERSION 0.7\n";
    pcdFile << "FIELDS x y z\n";
    pcdFile << "SIZE 4 4 4\n";
    pcdFile << "TYPE F F F\n";
    pcdFile << "COUNT 1 1 1\n";
    pcdFile << "WIDTH " << points.size() << "\n";
    pcdFile << "HEIGHT 1\n";
    pcdFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
    pcdFile << "POINTS " << points.size() << "\n";
    pcdFile << "DATA ascii\n";

    for (const auto& p : points) {
        pcdFile << p.x << " " << p.y << " " << p.z << "\n";
    }
    pcdFile.close();

    // Write labels file
    std::ofstream labelFile("two_spheres.labels");
    for (int label : labels) {
        labelFile << label << "\n";
    }
    labelFile.close();

    std::cout << "Generated test data:\n";
    std::cout << "  - Point cloud: two_spheres.pcd (" << points.size() << " points)\n";
    std::cout << "  - Labels: two_spheres.labels (" << labels.size() << " labels)\n";
    std::cout << "  - Inner domain (label=1): " << NUM_INNER_POINTS << " points\n";
    std::cout << "  - Outer domain (label=0): " << NUM_OUTER_POINTS << " points\n";
    std::cout << "\nExpected boundary: sphere at radius ~1.5\n";

    return 0;
}
