#ifndef VOXELIZER_CUH
#define VOXELIZER_CUH

// I made this because I am too lazy to build

#include <fstream>
#include <sstream>
#include <unordered_set>

namespace Voxelizer {

// Simple Triangle struct for CPU processing
struct Tri {
    vec3 v0, v1, v2;
};

// 1. Minimal OBJ Loader (extracts triangles only)
inline std::vector<Tri> loadTrianglesFromOBJ(const std::string &path) {
    std::vector<vec3> positions;
    std::vector<Tri> triangles;
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "[Voxelizer] Could not open " << path << std::endl;
        return triangles;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string prefix;
        ss >> prefix;

        if (prefix == "v") {
            vec3 v;
            ss >> v.x >> v.y >> v.z;
            positions.push_back(v);
        } else if (prefix == "f") {
            std::string s1, s2, s3, s4;
            ss >> s1 >> s2 >> s3;

            auto parseIdx = [&](std::string &s) {
                return std::stoi(s.substr(0, s.find('/'))) - 1;
            };

            // Triangulate on the fly
            vec3 p0 = positions[parseIdx(s1)];
            vec3 p1 = positions[parseIdx(s2)];
            vec3 p2 = positions[parseIdx(s3)];
            triangles.push_back({p0, p1, p2});

            if (ss >> s4) { // Handle quads
                vec3 p3 = positions[parseIdx(s4)];
                triangles.push_back({p0, p2, p3});
            }
        }
    }
    return triangles;
}

// 2. AABB-Triangle Overlap Test
inline bool checkOverlap(const vec3 &boxCenter, float boxHalfSize,
                         const Tri &tri) {
    // Translate triangle to be relative to box center
    vec3 v0 = tri.v0 - boxCenter;
    vec3 v1 = tri.v1 - boxCenter;
    vec3 v2 = tri.v2 - boxCenter;

    // 1. Test AABB against Triangle AABB
    float minX = fminf(v0.x, fminf(v1.x, v2.x));
    float maxX = fmaxf(v0.x, fmaxf(v1.x, v2.x));
    if (minX > boxHalfSize || maxX < -boxHalfSize)
        return false;

    float minY = fminf(v0.y, fminf(v1.y, v2.y));
    float maxY = fmaxf(v0.y, fmaxf(v1.y, v2.y));
    if (minY > boxHalfSize || maxY < -boxHalfSize)
        return false;

    float minZ = fminf(v0.z, fminf(v1.z, v2.z));
    float maxZ = fmaxf(v0.z, fmaxf(v1.z, v2.z));
    if (minZ > boxHalfSize || maxZ < -boxHalfSize)
        return false;

    return true;
}
} // namespace Voxelizer

#endif // VOXELIZER_CUH