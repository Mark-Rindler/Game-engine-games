// game.cuh
// Single Model Viewer: Andrew Jackson
// Loads only the Jackson statue and locks the camera to the side profile view.

#ifndef GAME_CUH
#define GAME_CUH

#include "common/PTRTtransfer.cuh"
#include "common/vec3.cuh"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace Game {

// Configuration
struct Config {
    static constexpr int WINDOW_WIDTH = 1920;
    static constexpr int WINDOW_HEIGHT = 1080;
    static constexpr const char *WINDOW_TITLE = "Andrew Jackson Viewer";

    static constexpr const char *PT_PRESET = "ultra";

    static constexpr float HDRI_INTENSITY = 0.05f;
    static constexpr float CAMERA_CLEARANCE = 6.0f;

    // Put model on this floor level
    static constexpr float FLOOR_Y = -1.0f;

    // Camera ray constants
    static constexpr float FAR_RAY_DIST = 100000.0f;
    static constexpr float OUTSIDE_EPS = 0.01f;
};

// Input Structure (Kept for compatibility with main.cu)
struct Input {
    bool w, a, s, d;
    bool e, r;
    bool shift, escape, space;
    bool jumpPressed, ePressed, rPressed;
    float mouseDX, mouseDY;
};

// AABB Helper Struct
struct AABB3 {
    vec3 mn{std::numeric_limits<float>::infinity()};
    vec3 mx{-std::numeric_limits<float>::infinity()};

    void expand(const vec3 &p) {
        mn.x = std::min(mn.x, p.x);
        mn.y = std::min(mn.y, p.y);
        mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x);
        mx.y = std::max(mx.y, p.y);
        mx.z = std::max(mx.z, p.z);
    }

    bool valid() const {
        return std::isfinite(mn.x) && std::isfinite(mn.y) &&
               std::isfinite(mn.z) && std::isfinite(mx.x) &&
               std::isfinite(mx.y) && std::isfinite(mx.z) && (mn.x <= mx.x) &&
               (mn.y <= mx.y) && (mn.z <= mx.z);
    }

    vec3 center() const { return (mn + mx) * 0.5f; }

    bool contains(const vec3 &p) const {
        return (p.x >= mn.x && p.x <= mx.x) && (p.y >= mn.y && p.y <= mx.y) &&
               (p.z >= mn.z && p.z <= mx.z);
    }
};

// Math Helpers for Camera & Placement

// Rotations (Euler XYZ)
inline vec3 rotateX(const vec3 &v, float a) {
    float c = cosf(a), s = sinf(a);
    return vec3(v.x, c * v.y - s * v.z, s * v.y + c * v.z);
}
inline vec3 rotateY(const vec3 &v, float a) {
    float c = cosf(a), s = sinf(a);
    return vec3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z);
}
inline vec3 rotateZ(const vec3 &v, float a) {
    float c = cosf(a), s = sinf(a);
    return vec3(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
}
inline vec3 rotateEulerXYZ(const vec3 &v, const vec3 &r) {
    vec3 out = v;
    out = rotateX(out, r.x);
    out = rotateY(out, r.y);
    out = rotateZ(out, r.z);
    return out;
}

inline AABB3 transformAABB_EulerXYZ(const AABB3 &b, const UnifiedTransform &t) {
    AABB3 out;
    vec3 corners[8] = {
        vec3(b.mn.x, b.mn.y, b.mn.z), vec3(b.mx.x, b.mn.y, b.mn.z),
        vec3(b.mn.x, b.mx.y, b.mn.z), vec3(b.mx.x, b.mx.y, b.mn.z),
        vec3(b.mn.x, b.mn.y, b.mx.z), vec3(b.mx.x, b.mn.y, b.mx.z),
        vec3(b.mn.x, b.mx.y, b.mx.z), vec3(b.mx.x, b.mx.y, b.mx.z),
    };

    for (int i = 0; i < 8; ++i) {
        vec3 p = corners[i];
        p = vec3(p.x * t.scale.x, p.y * t.scale.y, p.z * t.scale.z);
        p = rotateEulerXYZ(p, t.rotation);
        p = p + t.position;
        out.expand(p);
    }
    return out;
}

// Ray vs AABB (slab); return first forward hit t
inline bool rayAABB(const vec3 &ro, const vec3 &rd, const AABB3 &b,
                    float &tNearOut) {
    const float eps = 1e-8f;
    float tmin = -std::numeric_limits<float>::infinity();
    float tmax = std::numeric_limits<float>::infinity();

    auto slab = [&](float roC, float rdC, float mnC, float mxC) -> bool {
        if (fabsf(rdC) < eps)
            return (roC >= mnC && roC <= mxC);
        float inv = 1.0f / rdC;
        float t0 = (mnC - roC) * inv;
        float t1 = (mxC - roC) * inv;
        if (t0 > t1)
            std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        return tmax >= tmin;
    };

    if (!slab(ro.x, rd.x, b.mn.x, b.mx.x))
        return false;
    if (!slab(ro.y, rd.y, b.mn.y, b.mx.y))
        return false;
    if (!slab(ro.z, rd.z, b.mn.z, b.mx.z))
        return false;

    float tHit = (tmin >= 0.0f) ? tmin : tmax;
    if (tHit < 0.0f)
        return false;

    tNearOut = tHit;
    return true;
}

inline vec3 clampOutsideAABB(const vec3 &p, const vec3 &pushDir,
                             const AABB3 &box) {
    if (!box.contains(p))
        return p;

    vec3 d = pushDir.normalized();
    float best = std::numeric_limits<float>::infinity();

    auto axisExit = [&](float pc, float dc, float mn, float mx) {
        const float eps = 1e-8f;
        if (fabsf(dc) < eps)
            return;
        float boundary = (dc > 0.0f) ? mx : mn;
        float t = (boundary - pc) / dc;
        if (t >= 0.0f)
            best = std::min(best, t);
    };

    axisExit(p.x, d.x, box.mn.x, box.mx.x);
    axisExit(p.y, d.y, box.mn.y, box.mx.y);
    axisExit(p.z, d.z, box.mn.z, box.mx.z);

    if (!std::isfinite(best))
        return p + d * Config::OUTSIDE_EPS;
    return p + d * (best + Config::OUTSIDE_EPS);
}

// Compute OBJ AABB by scanning "v x y z" lines
inline AABB3 computeOBJLocalAABB(const std::string &path) {
    AABB3 b;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Failed to open OBJ for AABB: " << path << "\n";
        return b;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 2)
            continue;
        if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream iss(line);
            char vchar;
            float x, y, z;
            iss >> vchar >> x >> y >> z;
            if (iss)
                b.expand(vec3(x, y, z));
        }
    }
    return b;
}

// Compute stage position so model rests on floor (AABB bottom -> FLOOR_Y)
inline vec3 computeStagePosOnFloorY(const AABB3 &localBox, const vec3 &rotation,
                                    float uniformScale, float floorY) {
    UnifiedTransform t;
    t.position = vec3(0.0f, 0.0f, 0.0f);
    t.rotation = rotation;
    t.scale = vec3(uniformScale, uniformScale, uniformScale);

    AABB3 worldAtOrigin = transformAABB_EulerXYZ(localBox, t);
    float posY = floorY - worldAtOrigin.mn.y;

    return vec3(0.0f, posY, 0.0f);
}

// Place camera exactly 6 units from closest point on the AABB along view ray
inline void placeCamera(UnifiedScene &scene, const AABB3 &worldBox,
                        const vec3 &viewFromDirRaw) {
    vec3 c = worldBox.center();

    vec3 viewFromDir = viewFromDirRaw.normalized();
    vec3 rd = (viewFromDir * -1.0f).normalized();

    vec3 ro = c + viewFromDir * Config::FAR_RAY_DIST;

    float tNear = 0.0f;
    if (!rayAABB(ro, rd, worldBox, tNear)) {
        // Fallback if ray misses (shouldn't happen with center target)
        vec3 ext = (worldBox.mx - worldBox.mn) * 0.5f;
        float r = std::max(ext.x, std::max(ext.y, ext.z));
        vec3 camPos = c + viewFromDir * (r + Config::CAMERA_CLEARANCE +
                                         Config::OUTSIDE_EPS);
        camPos = clampOutsideAABB(camPos, viewFromDir, worldBox);
        scene.camera.setPosition(camPos);
        scene.camera.setTarget(c);
        return;
    }

    vec3 hitPoint = ro + rd * tNear; // closest point on AABB along view ray
    vec3 camPos = hitPoint + viewFromDir * Config::CAMERA_CLEARANCE;

    // Guarantee camera not inside AABB
    camPos = clampOutsideAABB(camPos, viewFromDir, worldBox);

    scene.camera.setPosition(camPos);
    scene.camera.setTarget(c);
}

// Lighting Setup
inline void setupLighting(UnifiedScene &scene) {
    scene.setHDRI("sky.hdr", Config::HDRI_INTENSITY);
    scene.setAmbientLight(vec3(0.00f));

    scene.addAreaLight(vec3(0.0f, 4.5f, 3.0f), vec3(0.0f, -1.0f, -0.4f),
                       vec3(1.0f, 0.98f, 0.9f), 2.5f, 2.0f, 120.0f);

    scene.addAreaLight(vec3(-3.0f, 2.5f, 2.0f), vec3(0.6f, -0.6f, -0.2f),
                       vec3(0.9f, 0.95f, 1.0f), 2.0f, 1.5f, 40.0f);
}

// Minimal State (Required by main.cu)
struct State {
    // Empty, we don't need dynamic state for a static scene
};

// GAME FUNCTIONS

inline void start(State &state, UnifiedScene &scene) {
    // 1. Setup Lights
    setupLighting(scene);

    // 2. Define Andrew Jackson Parameters (extracted from original
    // buildModelList)
    std::string filename = "models/andrew-jackson-zinc-sculpture-150k.obj";
    UnifiedMaterial material = UnifiedMaterial::Glass();
    float scale = 0.014f;
    vec3 rotation = vec3(0, 0.3f, 0); // Radians
    std::string name = "Jackson";

    // 3. Compute AABB for auto-placement
    AABB3 localAABB = computeOBJLocalAABB(filename);

    // 4. Calculate position to sit on floor
    vec3 position = vec3(0.0f, Config::FLOOR_Y, 0.0f);
    if (localAABB.valid()) {
        position = computeStagePosOnFloorY(localAABB, rotation, scale,
                                           Config::FLOOR_Y);
    }

    // 5. Add Mesh to Scene
    auto obj = scene.addMeshFromOBJ(filename, material);
    obj.setPosition(position).setScale(scale).setRotation(rotation).setName(
        name);

    // 6. Camera Placement
    // Andrew Jackson (index 1 in original) used view direction (-1, 0, 0)
    // This looks from Left (-X) toward Right (+X)
    vec3 viewFromDir = vec3(-1.0f, 0.0f, 0.0f);

    if (localAABB.valid()) {
        UnifiedTransform t;
        t.position = position;
        t.rotation = rotation;
        t.scale = vec3(scale, scale, scale);

        AABB3 worldAABB = transformAABB_EulerXYZ(localAABB, t);
        placeCamera(scene, worldAABB, viewFromDir);
    } else {
        // Fallback if OBJ load failed
        scene.camera.setPosition(vec3(-10.0f, 2.0f, 0.0f));
        scene.camera.setTarget(vec3(0.0f, 0.0f, 0.0f));
    }
}

inline void update(State &, UnifiedScene &, const Input &, float) {
    // Static scene, no updates required
}

inline std::string getWindowTitle(const State &, float fps) {
    return std::string(Config::WINDOW_TITLE) +
           " | FPS: " + std::to_string((int)fps) + " | Model: Jackson";
}

inline void onGameOver(const State &) {
    std::cout << "Exiting Viewer..." << std::endl;
}

} // namespace Game

#endif // GAME_CUH