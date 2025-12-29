// game.cuh
// MINIATURES: Game Logic & Scene Setup

#ifndef GAME_CUH
#define GAME_CUH

#include "common/PTRTtransfer.cuh"
#include "common/vec3.cuh"
#include "voxelizer.cuh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace Game {

// CONFIGURATION

struct Config {
    static constexpr int WINDOW_WIDTH = 1920;
    static constexpr int WINDOW_HEIGHT = 1080;
    static constexpr const char *WINDOW_TITLE = "MINIATURES - OPEN AIR";

    // Physics & Movement
    static constexpr float WALK_SPEED = 3.5f;
    static constexpr float RUN_SPEED = 7.0f;
    static constexpr float FLY_SPEED = 15.0f;
    static constexpr float FLY_FAST_MULT = 2.5f;
    static constexpr float MOUSE_SENSITIVITY = 0.002f;
    static constexpr float PLAYER_HEIGHT = 1.7f;

    // Head Bob
    static constexpr float BOB_FREQ_WALK = 10.0f;
    static constexpr float BOB_AMP_WALK = 0.05f;
    static constexpr float BOB_FREQ_RUN = 16.0f;
    static constexpr float BOB_AMP_RUN = 0.12f;

    // Architecture
    static constexpr float WORLD_SCALE = 2.0f;

    // Interaction
    static constexpr float INTERACTION_DIST = 15.0f;
    static constexpr float VOID_INTERACTION_DIST = 4.0f;

    // Blueprint Settings
    static constexpr float BP_CELL_SIZE = 0.05f;
    static constexpr float BP_SPACING = 0.06f;

    // Bench Settings
    static constexpr float MIN_BENCH_DIST = 2.5f;
    static constexpr float MINIATURE_SCALE = 0.1f;

    // Void Settings
    static constexpr float VOID_Y_LEVEL = 1000.0f;
    static constexpr float VOID_PLATFORM_SIZE = 26.0f;

    // ECONOMY CONFIGURATION (HARD MODE)
    static constexpr float STARTING_MONEY = 2000.0f;
    static constexpr float COST_BENCH = 1500.0f;
    static constexpr float COST_LIGHT = 3000.0f;
    static constexpr float COST_EXPANSION_BASE = 25000.0f;

    static constexpr float APPRAISAL_MULTIPLIER = 0.1f;

    // rendering setting
    static constexpr const char *PT_PRESET = "performance";
};

// DATA STRUCTURES

struct Input {
    bool w = false, a = false, s = false, d = false;
    bool e = false, f = false, b = false, r = false, l = false, i = false;
    bool shift = false, space = false, ctrl = false, escape = false;

    bool f11Pressed = false;

    bool leftClick = false;
    bool rightClick = false;

    // Event flags
    bool leftClickPressed = false;
    bool rightClickPressed = false;
    bool interactPressed = false;
    bool bPressed = false;
    bool rPressed = false;
    bool lPressed = false;
    bool iPressed = false;
    bool jPressed = false;
    bool kPressed = false;
    bool spacePressed = false;

    float mouseDX = 0.0f;
    float mouseDY = 0.0f;
};

struct BlueprintGrid {
    ObjectHandle officeFloor;
    ObjectHandle galleryFloor;
    ObjectHandle galWallN, galWallS, galWallE, galWallW;
    ObjectHandle offWallN, offWallS, offWallE, offWallW;
    ObjectHandle backboard;
    vec3 centerPos{0.0f, 1.7f, 2.3f};
};

struct VoidObject {
    ObjectHandle handle;
    std::string materialName;
    vec3 voidPosition;
    bool active = true;
};

struct BenchData {
    ObjectHandle handle;
    float valuePerSecond = 0.0f;
    std::vector<VoidObject> miniatures;

    // LIGHTING DATA
    bool hasLight = false;
    LightHandle lightSource;
    ObjectHandle lightMesh;
};

struct PaletteCube {
    ObjectHandle handle;
    std::string materialName;
};

struct State {
    // Player
    vec3 position{0.0f, Config::PLAYER_HEIGHT, 0.0f};
    vec3 velocity{0.0f, 0.0f, 0.0f};
    float yaw = 3.14f;
    float pitch = 0.0f;

    // Camera Animation
    float bobPhase = 0.0f;

    // Tycoon Data
    float money = Config::STARTING_MONEY;
    int expansionLevel = 0;
    float currentSize = 5.0f;

    // Blueprint Data
    BlueprintGrid blueprint;

    // Dynamic Architecture
    ObjectHandle hGalFloor;

    // Objects
    std::vector<BenchData> benches;
    UnifiedMeshDesc benchArchetype;
    int draggedBenchIndex = -1;
    int activeVoidBenchIndex = -1;

    // Void / Creative
    std::vector<PaletteCube> materialPalette;

    // Interaction Tools
    ObjectHandle ghostCube;
    std::string activeMaterialName;
    bool materialDirty = true;

    // Teleportation State
    bool inVoid = false;
    vec3 savedPosition{0.0f, 0.0f, 0.0f};
};

// ECONOMY & APPRAISAL LOGIC

enum class MatClass { Neutral, Organic, Industrial, Luxury, Tech };

struct MatStats {
    float cost;
    float baseValue;
    MatClass type;
    bool needsExposure;
};

inline MatStats getMatStats(const std::string &name) {
    if (name.find("Gold") != std::string::npos || name == "Diamond")
        return {200.0f, 4.0f, MatClass::Luxury, true};
    if (name == "Silver" || name == "Chrome")
        return {120.0f, 2.5f, MatClass::Luxury, true};
    if (name.find("Marble") != std::string::npos)
        return {80.0f, 1.5f, MatClass::Luxury, false};

    if (name.find("Wood") != std::string::npos)
        return {20.0f, 0.5f, MatClass::Organic, false};

    if (name == "Concrete" || name == "Iron" || name == "RubberBlack")
        return {10.0f, 0.2f, MatClass::Industrial, false};
    if (name == "BrushedAlu")
        return {40.0f, 0.8f, MatClass::Industrial, false};

    if (name.find("Neon") != std::string::npos ||
        name.find("Lamp") != std::string::npos)
        return {100.0f, 2.0f, MatClass::Tech, true};

    return {5.0f, 0.1f, MatClass::Neutral, false};
}

// OPTIMIZED APPRAISAL LOGIC
inline float calculateBenchValue(const BenchData &bench,
                                 bool printReport = false) {
    if (bench.miniatures.empty())
        return 0.0f;

    // Grid Configuration
    const int GRID_DIM = 64; // Sufficient coverage for 26x26 void area
    const int GRID_OFFSET = 32;
    const int GRID_SIZE = GRID_DIM * GRID_DIM * GRID_DIM;

    // Spatial Grid: Stores Index into a local material cache, -1 if empty
    std::vector<int> spatialGrid(GRID_SIZE, -1);

    // Local cache to map ID -> String for stats lookup
    std::vector<std::string> materialCache;
    std::unordered_map<std::string, int> matNameToId;

    auto getGridIdx = [&](int x, int y, int z) -> int {
        int ox = x + GRID_OFFSET;
        int oy = y; // Y is usually > 1000, we'll normalize this
        int oz = z + GRID_OFFSET;
        if (ox < 0 || ox >= GRID_DIM || oy < 0 || oy >= GRID_DIM || oz < 0 ||
            oz >= GRID_DIM)
            return -1;
        return ox + GRID_DIM * (oy + GRID_DIM * oz);
    };

    // 1. Populate Grid
    float minY = 99999.0f;
    int totalBlocks = 0;
    double sumX = 0, sumZ = 0;

    // Normalize Y based on void level
    int yBase = (int)Config::VOID_Y_LEVEL;

    for (const auto &obj : bench.miniatures) {
        if (!obj.active)
            continue;

        int ix = (int)roundf(obj.voidPosition.x);
        int iy = (int)roundf(obj.voidPosition.y) - yBase;
        int iz = (int)roundf(obj.voidPosition.z);

        // Register material in cache if new
        if (matNameToId.find(obj.materialName) == matNameToId.end()) {
            int newId = (int)materialCache.size();
            materialCache.push_back(obj.materialName);
            matNameToId[obj.materialName] = newId;
        }
        int matId = matNameToId[obj.materialName];

        int idx = getGridIdx(ix, iy, iz);
        if (idx != -1) {
            spatialGrid[idx] = matId;
            totalBlocks++;
            sumX += obj.voidPosition.x;
            sumZ += obj.voidPosition.z;
            if (obj.voidPosition.y < minY)
                minY = obj.voidPosition.y;
        }
    }

    if (totalBlocks == 0)
        return 0.0f;

    // 2. Stability Calculation
    float comX = (float)(sumX / totalBlocks);
    float comZ = (float)(sumZ / totalBlocks);
    double baseSumX = 0, baseSumZ = 0;
    int baseCount = 0;
    float baseThreshold = minY + 0.1f;

    for (const auto &obj : bench.miniatures) {
        if (!obj.active)
            continue;
        if (obj.voidPosition.y <= baseThreshold) {
            baseSumX += obj.voidPosition.x;
            baseSumZ += obj.voidPosition.z;
            baseCount++;
        }
    }

    // 3. Harmony & Value Calculation
    float totalMaterialValue = 0.0f;
    float harmonyScore = 0.0f;
    float clashPenalty = 0.0f;

    int dx[] = {1, -1, 0, 0, 0, 0};
    int dy[] = {0, 0, 1, -1, 0, 0};
    int dz[] = {0, 0, 0, 0, 1, -1};

    // Iterate over objects again to check neighbors via grid
    for (const auto &obj : bench.miniatures) {
        if (!obj.active)
            continue;

        int ix = (int)roundf(obj.voidPosition.x);
        int iy = (int)roundf(obj.voidPosition.y) - yBase;
        int iz = (int)roundf(obj.voidPosition.z);

        MatStats stats = getMatStats(obj.materialName);

        int openAirFaces = 0;
        int neighbors = 0;

        for (int i = 0; i < 6; ++i) {
            int nIdx = getGridIdx(ix + dx[i], iy + dy[i], iz + dz[i]);

            // Check grid at neighbor location
            if (nIdx != -1 && spatialGrid[nIdx] != -1) {
                neighbors++;
                int nMatId = spatialGrid[nIdx];
                std::string nMatName = materialCache[nMatId];
                MatStats nStats = getMatStats(nMatName);

                if (stats.type == nStats.type) {
                    harmonyScore += 0.2f;
                } else if (stats.type == MatClass::Luxury &&
                           nStats.type == MatClass::Industrial) {
                    harmonyScore += 1.0f;
                } else if (stats.type == MatClass::Organic &&
                           nStats.type == MatClass::Tech) {
                    clashPenalty += 0.5f;
                }
            } else {
                openAirFaces++;
            }
        }

        float blockVal = stats.baseValue;
        if (stats.needsExposure && openAirFaces == 0)
            blockVal *= 0.1f;
        if (stats.type == MatClass::Tech)
            blockVal += (openAirFaces * 0.5f);
        totalMaterialValue += blockVal;
    }

    float stabilityMultiplier = 1.0f;
    if (baseCount > 0) {
        float baseCX = (float)(baseSumX / baseCount);
        float baseCZ = (float)(baseSumZ / baseCount);
        float dist =
            sqrtf(powf(comX - baseCX, 2.0f) + powf(comZ - baseCZ, 2.0f));

        if (dist > 1.5f) {
            stabilityMultiplier = 0.7f;
            if (printReport)
                std::cout << "WARNING: Unstable!\n";
        }
    } else {
        stabilityMultiplier = 0.5f;
    }

    float finalVal = totalMaterialValue;
    finalVal += harmonyScore;
    finalVal -= clashPenalty;
    finalVal *= stabilityMultiplier;
    finalVal *= Config::APPRAISAL_MULTIPLIER;

    if (bench.hasLight)
        finalVal *= 1.5f;

    return (finalVal > 0.0f) ? finalVal : 0.0f;
}

inline void importVoxelModel(State &state, UnifiedScene &scene,
                             const std::string &objPath,
                             const std::string &materialName) {
    if (state.activeVoidBenchIndex == -1) {
        std::cout << "Error: Must be inside the Void to import." << std::endl;
        return;
    }

    std::cout << "Importing " << objPath << "..." << std::endl;
    auto tris = Voxelizer::loadTrianglesFromOBJ(objPath);
    if (tris.empty())
        return;

    // Helpers
    auto min3 = [](const vec3 &a, const vec3 &b) {
        return vec3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
    };
    auto max3 = [](const vec3 &a, const vec3 &b) {
        return vec3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
    };

    // 1. Calculate Model Bounds
    vec3 modelMin(1e9), modelMax(-1e9);
    for (const auto &t : tris) {
        modelMin = min3(modelMin, min3(t.v0, min3(t.v1, t.v2)));
        modelMax = max3(modelMax, max3(t.v0, max3(t.v1, t.v2)));
    }

    // 2. Auto-Calculate Scale to Fit Platform
    // Platform range is roughly -13 to +13 (Size 26)
    // We fit the largest horizontal dimension of the model to this size.
    float modelWidth = modelMax.x - modelMin.x;
    float modelDepth = modelMax.z - modelMin.z;
    float maxModelDim = fmaxf(modelWidth, modelDepth);

    if (maxModelDim < 0.001f)
        maxModelDim = 1.0f;

    // We use a slightly smaller size (Size - 1) to ensure it fits comfortably
    // within the integer bounds
    float targetSize = Config::VOID_PLATFORM_SIZE - 1.0f;
    float scale = targetSize / maxModelDim;

    // 3. Transform Triangles to World Space
    // We align the model's center to (0,0) horizontally
    vec3 modelCenter = (modelMin + modelMax) * 0.5f;

    // Calculate offset to place model ON the floor (Y) and CENTERED (X/Z)
    // Floor level is VOID_Y_LEVEL. The first block should be at VOID_Y_LEVEL
    // + 1.0f
    float yOffset = Config::VOID_Y_LEVEL + 1.0f - (modelMin.y * scale);
    vec3 offset(-modelCenter.x * scale, yOffset, -modelCenter.z * scale);

    // Apply Transform
    for (auto &t : tris) {
        t.v0 = (t.v0 * scale) + offset;
        t.v1 = (t.v1 * scale) + offset;
        t.v2 = (t.v2 * scale) + offset;
    }

    std::cout << "Auto-scaling: " << scale << "x. Locking to Integer Grid."
              << std::endl;

    // 4. Voxelize (Grid Locking)
    std::vector<vec3> voxelPositions;

    // Grid bounds (Integer coordinates)
    int halfSize = (int)(Config::VOID_PLATFORM_SIZE / 2.0f);
    int yMax = (int)((modelMax.y - modelMin.y) * scale) + 2;

    // Scan the platform volume
    for (int x = -halfSize; x <= halfSize; ++x) {
        for (int z = -halfSize; z <= halfSize; ++z) {
            for (int y = 0; y < yMax; ++y) {

                // EXACT INTEGER COORDINATES
                // Manual building uses roundf(), so we use exact integers here.
                float gridY = Config::VOID_Y_LEVEL + 1.0f + y;
                vec3 voxelPos((float)x, gridY, (float)z);

                // Check overlap with strict box size (0.5 radius = 1.0 size)
                // We use 0.51f to catch triangles that are exactly on the face
                bool hit = false;
                for (const auto &tri : tris) {
                    if (Voxelizer::checkOverlap(voxelPos, 0.51f, tri)) {
                        hit = true;
                        break;
                    }
                }

                if (hit) {
                    voxelPositions.push_back(voxelPos);
                }
            }
        }
    }

    // 5. Spawn Logic
    BenchData &bench = state.benches[state.activeVoidBenchIndex];
    MatStats stats = getMatStats(materialName);
    float totalCost = voxelPositions.size() * stats.cost;

    if (state.money < totalCost) {
        std::cout << "Import Failed: Cost $" << totalCost << " exceeds funds."
                  << std::endl;
        return;
    }
    state.money -= totalCost;

    UnifiedMeshDesc cubeDesc = UnifiedMeshDesc::Cube();
    cubeDesc.setDynamic(false);
    cubeDesc.setScale(1.0f);

    for (const auto &pos : voxelPositions) {
        std::string name = "Imp_" + std::to_string(bench.miniatures.size());

        ObjectHandle handle = scene.instantiateObject(cubeDesc, name);
        handle.setPosition(pos);
        handle.useLibraryMaterial(materialName);

        VoidObject vo;
        vo.handle = handle;
        vo.materialName = materialName;
        vo.voidPosition = pos;
        vo.active = true;
        bench.miniatures.push_back(vo);
    }

    std::cout << "Success! Spawned " << voxelPositions.size() << " voxels."
              << std::endl;
    bench.valuePerSecond = calculateBenchValue(bench, false);
}

// Visual Updates
inline void updateGalleryGeometry(State &state) {
    float sizePixels = state.currentSize;
    float sizeWorld = sizePixels * Config::WORLD_SCALE;
    float zCenter = -2.5f - (sizeWorld / 2.0f);
    state.hGalFloor.setPosition(vec3(0, -0.5f, zCenter))
        .setScale(vec3(sizeWorld, 1.0f, sizeWorld));
}

inline void updateBlueprintVisuals(State &state) {
    float sp = Config::BP_SPACING;
    float sz = Config::BP_CELL_SIZE;
    vec3 basePos = state.blueprint.centerPos + vec3(0, -0.3f, -0.05f);

    float officeDim = 5.0f;
    float officeSize = officeDim * sp - (sp - sz);
    state.blueprint.officeFloor.setPosition(basePos + vec3(0, 2.0f * sp, 0))
        .setScale(vec3(officeSize, officeSize, sz));

    float galDim = state.currentSize;
    float galSizeW = galDim * sp - (sp - sz);
    float galSizeH = galDim * sp - (sp - sz);

    float galBaseY = (officeDim + 1) * sp;
    float galCenterY = galBaseY + (galDim * sp * 0.5f) - (sp * 0.5f);

    state.blueprint.galleryFloor.setPosition(basePos + vec3(0, galCenterY, 0))
        .setScale(vec3(galSizeW, galSizeH, sz));

    float mapWidth = (galDim > 5 ? galDim : 5) * sp + 0.2f;
    float mapHeight = (officeDim + galDim + 2) * sp + 0.2f;
    state.blueprint.backboard
        .setPosition(basePos + vec3(0, (mapHeight / 2.0f) - 0.2f, 0.06f))
        .setScale(vec3(mapWidth, mapHeight, 0.05f));

    float wallThick = sz;
    float fullW = galDim * sp;
    float halfW = fullW / 2.0f;

    state.blueprint.galWallN.setPosition(basePos + vec3(0, galBaseY + fullW, 0))
        .setScale(vec3(fullW, wallThick, wallThick));
    state.blueprint.galWallS.setPosition(basePos + vec3(0, galBaseY - sp, 0))
        .setScale(vec3(fullW, wallThick, wallThick));
    state.blueprint.galWallE
        .setPosition(basePos + vec3(halfW, galBaseY + halfW - (sp * 0.5f), 0))
        .setScale(vec3(wallThick, fullW + sp, wallThick));
    state.blueprint.galWallW
        .setPosition(basePos + vec3(-halfW, galBaseY + halfW - (sp * 0.5f), 0))
        .setScale(vec3(wallThick, fullW + sp, wallThick));
}

// Transition Helpers

inline void miniaturizeBenchContent(State &state, bool printReport = false) {
    if (state.activeVoidBenchIndex == -1)
        return;

    BenchData &bench = state.benches[state.activeVoidBenchIndex];
    bench.valuePerSecond = calculateBenchValue(bench, printReport);

    vec3 benchPos = bench.handle.getPosition();
    vec3 benchTopCenter = benchPos + vec3(0.0f, 0.55f, 0.0f);
    vec3 voidOrigin(0.0f, Config::VOID_Y_LEVEL, 0.0f);

    for (auto &obj : bench.miniatures) {
        if (!obj.active)
            continue;

        vec3 relativePos = obj.voidPosition - voidOrigin;
        vec3 miniOffset = relativePos * Config::MINIATURE_SCALE;

        obj.handle.setPosition(benchTopCenter + miniOffset);
        obj.handle.setScale(Config::MINIATURE_SCALE);
    }
}

inline void maximizeBenchContent(State &state) {
    if (state.activeVoidBenchIndex == -1)
        return;

    BenchData &bench = state.benches[state.activeVoidBenchIndex];

    for (auto &obj : bench.miniatures) {
        if (!obj.active)
            continue;

        obj.handle.setPosition(obj.voidPosition);
        obj.handle.setScale(1.0f);
    }
}

// Gameplay Logic
inline void attemptExpansion(State &state) {
    if (state.currentSize >= 23.0f)
        return;

    float cost = Config::COST_EXPANSION_BASE * (state.expansionLevel + 1);

    if (state.money >= cost) {
        state.money -= cost;
        state.expansionLevel++;
        state.currentSize += 2.0f;
        updateGalleryGeometry(state);
        updateBlueprintVisuals(state);
    }
}

inline bool intersectRayPlane(vec3 rayOrigin, vec3 rayDir, float planeY,
                              vec3 &hitPos) {
    if (fabs(rayDir.y) < 1e-6)
        return false;
    float t = (planeY - rayOrigin.y) / rayDir.y;
    if (t < 0)
        return false;
    hitPos = rayOrigin + rayDir * t;
    return true;
}

inline bool intersectRayAABB(vec3 rayOrigin, vec3 rayDir, vec3 boxMin,
                             vec3 boxMax, float &t) {
    vec3 dirInv(1.0f / rayDir.x, 1.0f / rayDir.y, 1.0f / rayDir.z);
    float t1 = (boxMin.x - rayOrigin.x) * dirInv.x;
    float t2 = (boxMax.x - rayOrigin.x) * dirInv.x;
    float t3 = (boxMin.y - rayOrigin.y) * dirInv.y;
    float t4 = (boxMax.y - rayOrigin.y) * dirInv.y;
    float t5 = (boxMin.z - rayOrigin.z) * dirInv.z;
    float t6 = (boxMax.z - rayOrigin.z) * dirInv.z;
    float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
    float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
    if (tmax < 0 || tmin > tmax)
        return false;
    t = tmin;
    return true;
}

// Spawning
inline void spawnBench(State &state, UnifiedScene &scene) {
    if (state.money < Config::COST_BENCH)
        return;

    state.money -= Config::COST_BENCH;

    ObjectHandle newHandle = scene.instantiateObject(
        state.benchArchetype, "Bench_" + std::to_string(state.benches.size()));
    newHandle.setPosition(vec3(0, 0.5f, 0)).setScale(vec3(2.0f, 1.0f, 2.0f));
    newHandle.useLibraryMaterial("WoodCherry");

    BenchData data;
    data.handle = newHandle;
    data.valuePerSecond = 0.0f;
    data.hasLight = false;

    state.benches.push_back(data);
    state.draggedBenchIndex = (int)state.benches.size() - 1;
}

inline void attemptSpawnLight(State &state, UnifiedScene &scene,
                              int benchIndex) {
    if (benchIndex < 0 || benchIndex >= state.benches.size())
        return;
    BenchData &bench = state.benches[benchIndex];

    if (bench.hasLight)
        return;
    if (state.money < Config::COST_LIGHT)
        return;

    state.money -= Config::COST_LIGHT;
    bench.hasLight = true;

    UnifiedMeshDesc lampDesc = UnifiedMeshDesc::Cube();
    lampDesc.setDynamic(true);
    lampDesc.setScale(vec3(0.5f, 0.1f, 0.5f));

    vec3 benchPos = bench.handle.getPosition();
    vec3 benchTopCenter = benchPos + vec3(0.0f, 0.55f, 0.0f);
    vec3 lightPos = benchTopCenter + vec3(0.0f, 3.0f, 0.0f);

    bench.lightMesh = scene.instantiateObject(
        lampDesc, "LightMesh_" + std::to_string(benchIndex));
    bench.lightMesh.setPosition(lightPos);
    bench.lightMesh.useLibraryMaterial("LampWood");

    bench.lightSource =
        scene.addSpotLight(lightPos - vec3(0, 0.1f, 0), vec3(0, -1, 0),
                           vec3(1.0f, 0.95f, 0.8f), 150.0f, 0.6f, 0.9f);
    bench.lightSource.setName("BenchLight_" + std::to_string(benchIndex));

    bench.valuePerSecond = calculateBenchValue(bench, false);
}

inline void spawnVoidObjectFromGhost(State &state, UnifiedScene &scene) {
    if (state.activeVoidBenchIndex == -1)
        return;

    MatStats stats = getMatStats(state.activeMaterialName);
    if (state.money < stats.cost)
        return;

    state.money -= stats.cost;

    vec3 spawnPos = state.ghostCube.getPosition();

    UnifiedMeshDesc desc = UnifiedMeshDesc::Cube();

    desc.setDynamic(false);

    desc.setScale(1.0f);

    BenchData &activeBench = state.benches[state.activeVoidBenchIndex];
    std::string name = "VoidObj_B" +
                       std::to_string(state.activeVoidBenchIndex) + "_" +
                       std::to_string(activeBench.miniatures.size());

    ObjectHandle newHandle = scene.instantiateObject(desc, name);
    newHandle.setPosition(spawnPos);

    if (!state.activeMaterialName.empty()) {
        newHandle.useLibraryMaterial(state.activeMaterialName);
    }

    VoidObject obj;
    obj.handle = newHandle;
    obj.materialName = state.activeMaterialName;
    obj.voidPosition = spawnPos;
    obj.active = true;

    activeBench.miniatures.push_back(obj);

    activeBench.valuePerSecond = calculateBenchValue(activeBench, false);
}

// Initialization
inline void start(State &state, UnifiedScene &scene) {
    scene.setHDRI("kloppenheim_07_puresky_4k.hdr", 1.0f);

    // Materials
    scene.addLibraryMaterial("Concrete", UnifiedMaterial::Concrete());
    scene.addLibraryMaterial("WoodOak", UnifiedMaterial::WoodOak());
    scene.addLibraryMaterial("WoodCherry", UnifiedMaterial::WoodCherry());
    scene.addLibraryMaterial("PlasticRed", UnifiedMaterial::PlasticRed());
    scene.addLibraryMaterial("MarbleWhite",
                             UnifiedMaterial::MarbleCarrara(false));
    scene.addLibraryMaterial("LampWood", UnifiedMaterial::EmissiveLamp(
                                             vec3(1.0f, 0.8f, 0.6f), 0.005f));
    scene.addLibraryMaterial("RubberBlack", UnifiedMaterial::RubberBlack());
    scene.addLibraryMaterial("GridWhite",
                             UnifiedMaterial(vec3(1.0f), 0.8f, 0.0f));

    scene.addLibraryMaterial("WoodWalnut", UnifiedMaterial::WoodWalnut());
    scene.addLibraryMaterial("Gold", UnifiedMaterial::Gold());
    scene.addLibraryMaterial("Silver", UnifiedMaterial::Silver());
    scene.addLibraryMaterial("Copper", UnifiedMaterial::Copper());
    scene.addLibraryMaterial("Chrome", UnifiedMaterial::Chrome());
    scene.addLibraryMaterial("Iron", UnifiedMaterial::Iron());
    scene.addLibraryMaterial("BrushedAlu", UnifiedMaterial::BrushedAluminum());
    scene.addLibraryMaterial("MarblePolished",
                             UnifiedMaterial::MarbleCarrara(true));
    scene.addLibraryMaterial("MarbleNero", UnifiedMaterial::MarbleNero(true));
    scene.addLibraryMaterial("MarbleVerde", UnifiedMaterial::MarbleVerde(true));
    scene.addLibraryMaterial("Glass", UnifiedMaterial::Glass());
    scene.addLibraryMaterial("FrostedGlass", UnifiedMaterial::FrostedGlass());
    scene.addLibraryMaterial("Diamond", UnifiedMaterial::Diamond());
    scene.addLibraryMaterial("PlasticBlue", UnifiedMaterial::PlasticBlue());
    scene.addLibraryMaterial("PlasticGreen", UnifiedMaterial::PlasticGreen());
    scene.addLibraryMaterial("CarPaintRed",
                             UnifiedMaterial::CarPaint(vec3(0.8f, 0.1f, 0.1f)));
    scene.addLibraryMaterial(
        "NeonBlue", UnifiedMaterial::NeonLight(vec3(0.1f, 0.5f, 1.0f)));

    // Base Environment
    scene.addCube()
        .useLibraryMaterial("Concrete")
        .setPosition(vec3(0, -0.5f, 0))
        .setScale(vec3(5, 1, 5));

    // Blueprint Optimization
    state.blueprint.backboard = scene.addCube();
    state.blueprint.backboard.useLibraryMaterial("LampWood");
    state.blueprint.officeFloor = scene.addCube();
    state.blueprint.officeFloor.useLibraryMaterial("PlasticRed");
    state.blueprint.galleryFloor = scene.addCube();
    state.blueprint.galleryFloor.useLibraryMaterial("MarbleWhite");

    // Perimeters
    state.blueprint.galWallN = scene.addCube();
    state.blueprint.galWallN.useLibraryMaterial("RubberBlack");
    state.blueprint.galWallS = scene.addCube();
    state.blueprint.galWallS.useLibraryMaterial("RubberBlack");
    state.blueprint.galWallE = scene.addCube();
    state.blueprint.galWallE.useLibraryMaterial("RubberBlack");
    state.blueprint.galWallW = scene.addCube();
    state.blueprint.galWallW.useLibraryMaterial("RubberBlack");

    // Dynamic Floor
    state.hGalFloor = scene.addCube();
    state.hGalFloor.useLibraryMaterial("WoodOak");
    state.hGalFloor.setDynamic(true);

    state.benchArchetype = UnifiedMeshDesc::Cube();
    state.benchArchetype.setDynamic(true);
    state.benchArchetype.setScale(vec3(2.0f, 1.0f, 2.0f));

    // Void Platform
    auto voidPlat = scene.addCube();
    voidPlat.useLibraryMaterial("RubberBlack");
    voidPlat.setPosition(vec3(0, Config::VOID_Y_LEVEL, 0))
        .setScale(
            vec3(Config::VOID_PLATFORM_SIZE, 1.0f, Config::VOID_PLATFORM_SIZE));

    // Void Grid
    int gridSize = (int)Config::VOID_PLATFORM_SIZE;
    float lineThickness = 0.05f;
    float gridY = Config::VOID_Y_LEVEL + 0.51f;
    float offset = 0.5f;

    for (int i = 0; i <= gridSize; ++i) {
        float x = -Config::VOID_PLATFORM_SIZE / 2.0f + i + offset;
        auto line = scene.addCube();
        line.useLibraryMaterial("GridWhite");
        line.setPosition(vec3(x, gridY, 0))
            .setScale(
                vec3(lineThickness, lineThickness, Config::VOID_PLATFORM_SIZE));
    }
    for (int i = 0; i <= gridSize; ++i) {
        float z = -Config::VOID_PLATFORM_SIZE / 2.0f + i + offset;
        auto line = scene.addCube();
        line.useLibraryMaterial("GridWhite");
        line.setPosition(vec3(0, gridY, z))
            .setScale(
                vec3(Config::VOID_PLATFORM_SIZE, lineThickness, lineThickness));
    }

    // Material Palette
    std::vector<std::string> paletteNames = {
        "WoodCherry",  "WoodOak",        "WoodWalnut",  "Gold",
        "Silver",      "Copper",         "Chrome",      "Iron",
        "BrushedAlu",  "MarblePolished", "MarbleNero",  "MarbleVerde",
        "Glass",       "FrostedGlass",   "Diamond",     "PlasticRed",
        "PlasticBlue", "PlasticGreen",   "CarPaintRed", "NeonBlue"};

    float wallX = -(Config::VOID_PLATFORM_SIZE / 2.0f) - 5.0f;
    float startY = Config::VOID_Y_LEVEL + 2.0f;
    float startZ = -5.0f;
    int cols = 4;
    float spacing = 2.0f;

    for (size_t i = 0; i < paletteNames.size(); ++i) {
        int r = i / cols;
        int c = i % cols;
        auto h = scene.addCube();
        h.useLibraryMaterial(paletteNames[i]);
        h.setPosition(vec3(wallX, startY + r * spacing, startZ + c * spacing));
        state.materialPalette.push_back({h, paletteNames[i]});
    }

    state.activeMaterialName = paletteNames[0];

    state.ghostCube = scene.addCube();
    state.ghostCube.useLibraryMaterial(state.activeMaterialName);
    state.ghostCube.setDynamic(true);
    state.ghostCube.setScale(1.0f);

    scene.setAmbientLight(vec3(0.8f));
    updateGalleryGeometry(state);
    updateBlueprintVisuals(state);
    std::cout << "MINIATURES INITIALIZED (OPTIMIZED)." << std::endl;
}

// Update Loop
inline void update(State &state, UnifiedScene &scene, const Input &input,
                   float dt) {
    state.yaw += input.mouseDX * Config::MOUSE_SENSITIVITY;
    state.pitch -= input.mouseDY * Config::MOUSE_SENSITIVITY;
    state.pitch = fmaxf(-1.5f, fminf(1.5f, state.pitch));

    if (state.inVoid && input.rPressed) {
        miniaturizeBenchContent(state, true);
        state.activeVoidBenchIndex = -1;
        state.position = state.savedPosition;
        state.inVoid = false;
        state.ghostCube.setScale(0.0f);
        return;
    }

    if (input.iPressed && state.inVoid) {
        Game::importVoxelModel(state, scene, "models/wash.obj", "Gold");
    }

    if (input.jPressed && state.inVoid) {
        Game::importVoxelModel(state, scene, "models/vse.obj",
                               "MarblePolished");
    }

    if (input.kPressed && state.inVoid) {
        Game::importVoxelModel(state, scene, "models/zinger.obj", "Copper");
    }

    vec3 forward, right;
    if (!state.inVoid) {
        forward = vec3(sin(state.yaw), 0.0f, -cos(state.yaw));
        right = cross(forward, vec3(0, 1, 0)).normalized();
    } else {
        forward = vec3(sin(state.yaw) * cos(state.pitch), sin(state.pitch),
                       -cos(state.yaw) * cos(state.pitch));
        right = cross(forward, vec3(0, 1, 0)).normalized();
    }

    vec3 wishDir(0.0f);
    if (input.w)
        wishDir = wishDir + forward;
    if (input.s)
        wishDir = wishDir - forward;
    if (input.d)
        wishDir = wishDir + right;
    if (input.a)
        wishDir = wishDir - right;

    if (wishDir.length_squared() > 0.001f)
        wishDir = wishDir.normalized();
    bool isMoving = (wishDir.length_squared() > 0.001f);

    if (!state.inVoid) {
        float speed = input.shift ? Config::RUN_SPEED : Config::WALK_SPEED;
        state.position = state.position + wishDir * speed * dt;

        if (isMoving) {
            float freq =
                input.shift ? Config::BOB_FREQ_RUN : Config::BOB_FREQ_WALK;
            float amp =
                input.shift ? Config::BOB_AMP_RUN : Config::BOB_AMP_WALK;
            state.bobPhase += dt * freq;
            float bobOffset = sin(state.bobPhase) * amp;
            scene.camera.setPosition(state.position + vec3(0, bobOffset, 0));
        } else {
            state.bobPhase = 0.0f;
            scene.camera.setPosition(state.position);
        }
    } else {
        float speed = input.shift ? (Config::FLY_SPEED * Config::FLY_FAST_MULT)
                                  : Config::FLY_SPEED;
        state.position = state.position + wishDir * speed * dt;

        if (input.space)
            state.position.y += speed * dt;
        if (input.ctrl)
            state.position.y -= speed * dt;

        scene.camera.setPosition(state.position);
    }

    vec3 lookDir;
    lookDir.x = sin(state.yaw) * cos(state.pitch);
    lookDir.y = sin(state.pitch);
    lookDir.z = -cos(state.yaw) * cos(state.pitch);
    scene.camera.setTarget(scene.camera.lookfrom + lookDir);

    for (const auto &b : state.benches)
        state.money += b.valuePerSecond * dt;

    if (input.interactPressed && !state.inVoid) {
        float dist = (state.position - state.blueprint.centerPos).length();
        vec3 toBP = (state.blueprint.centerPos - state.position).normalized();
        if (dist < Config::INTERACTION_DIST &&
            dot(toBP, vec3(sin(state.yaw), 0, -cos(state.yaw))) > 0.5f) {
            attemptExpansion(state);
        }
    }

    if (state.inVoid && state.activeVoidBenchIndex != -1) {
        vec3 ghostPos =
            scene.camera.lookfrom + lookDir * Config::VOID_INTERACTION_DIST;
        ghostPos.x = roundf(ghostPos.x);
        ghostPos.y = roundf(ghostPos.y);
        ghostPos.z = roundf(ghostPos.z);

        float halfSize = Config::VOID_PLATFORM_SIZE / 2.0f - 0.5f;
        ghostPos.x = fmaxf(-halfSize, fminf(halfSize, ghostPos.x));
        ghostPos.z = fmaxf(-halfSize, fminf(halfSize, ghostPos.z));

        if (ghostPos.y < Config::VOID_Y_LEVEL + 0.5f) {
            ghostPos.y = Config::VOID_Y_LEVEL + 0.5f;
        }

        state.ghostCube.setPosition(ghostPos);
        state.ghostCube.setScale(1.0f);

        if (state.materialDirty) {
            state.ghostCube.useLibraryMaterial(state.activeMaterialName);
            state.materialDirty = false;
        }

        if (input.leftClickPressed) {
            float closestDist = 100.0f;
            int matIdx = -1;

            for (int i = 0; i < state.materialPalette.size(); ++i) {
                vec3 pos = state.materialPalette[i].handle.getPosition();
                vec3 minB = pos - vec3(0.5f);
                vec3 maxB = pos + vec3(0.5f);
                float t = 0;
                if (intersectRayAABB(scene.camera.lookfrom, lookDir, minB, maxB,
                                     t)) {
                    if (t < closestDist) {
                        closestDist = t;
                        matIdx = i;
                    }
                }
            }

            if (matIdx != -1) {
                state.activeMaterialName =
                    state.materialPalette[matIdx].materialName;
                state.materialDirty = true;
            } else {
                spawnVoidObjectFromGhost(state, scene);
            }
        }

        if (input.rightClickPressed) {
            float closestDist = 100.0f;
            int hitIdx = -1;
            BenchData &activeBench = state.benches[state.activeVoidBenchIndex];

            for (int i = 0; i < activeBench.miniatures.size(); ++i) {
                if (!activeBench.miniatures[i].active)
                    continue;
                vec3 pos = activeBench.miniatures[i].handle.getPosition();
                vec3 minB = pos - vec3(0.5f);
                vec3 maxB = pos + vec3(0.5f);
                float t = 0;
                if (intersectRayAABB(scene.camera.lookfrom, lookDir, minB, maxB,
                                     t)) {
                    if (t < closestDist) {
                        closestDist = t;
                        hitIdx = i;
                    }
                }
            }

            if (hitIdx != -1) {
                activeBench.miniatures[hitIdx].handle.setPosition(
                    vec3(0, -9999.0f, 0));
                activeBench.miniatures[hitIdx].handle.setScale(0.0f);
                activeBench.miniatures[hitIdx].active = false;
                activeBench.valuePerSecond =
                    calculateBenchValue(activeBench, false);
            }
        }
    }

    if (!state.inVoid) {
        if (input.bPressed)
            spawnBench(state, scene);

        if (state.draggedBenchIndex != -1) {
            vec3 hitPos;
            if (intersectRayPlane(scene.camera.lookfrom, lookDir, 0.0f,
                                  hitPos)) {
                float snap = 1.0f;
                float targetX = roundf(hitPos.x / snap) * snap;
                float targetZ = roundf(hitPos.z / snap) * snap;

                float galSizeWorld = state.currentSize * Config::WORLD_SCALE;
                float galCenterZ = -2.5f - (galSizeWorld / 2.0f);
                float galHalfSize = galSizeWorld / 2.0f;

                float margin = 1.0f;
                float minX = -galHalfSize + margin;
                float maxX = galHalfSize - margin;
                float minZ = galCenterZ - galHalfSize + margin;
                float maxZ = galCenterZ + galHalfSize - margin;

                bool insideBounds = (targetX >= minX && targetX <= maxX &&
                                     targetZ >= minZ && targetZ <= maxZ);

                if (insideBounds) {
                    bool validDist = true;
                    for (int i = 0; i < state.benches.size(); ++i) {
                        if (i == state.draggedBenchIndex)
                            continue;
                        vec3 otherPos = state.benches[i].handle.getPosition();
                        float dist = sqrtf(powf(otherPos.x - targetX, 2) +
                                           powf(otherPos.z - targetZ, 2));
                        if (dist < Config::MIN_BENCH_DIST) {
                            validDist = false;
                            break;
                        }
                    }

                    if (validDist) {
                        BenchData &b = state.benches[state.draggedBenchIndex];
                        b.handle.setPosition(vec3(targetX, 0.5f, targetZ));
                        if (b.hasLight) {
                            vec3 lightPos = vec3(targetX, 7.0f, targetZ);
                            b.lightMesh.setPosition(lightPos);
                            b.lightSource.setPosition(lightPos -
                                                      vec3(0, 0.1f, 0));
                        }
                        state.activeVoidBenchIndex = state.draggedBenchIndex;
                        miniaturizeBenchContent(state, false);
                        state.activeVoidBenchIndex = -1;
                    }
                }
            }
            if (input.leftClickPressed)
                state.draggedBenchIndex = -1;
        } else {
            float closestDist = 100.0f;
            int hitIdx = -1;
            for (int i = 0; i < state.benches.size(); ++i) {
                vec3 pos = state.benches[i].handle.getPosition();
                vec3 minB = pos - vec3(1.0f, 0.5f, 1.0f);
                vec3 maxB = pos + vec3(1.0f, 0.5f, 1.0f);
                float t = 0;
                if (intersectRayAABB(scene.camera.lookfrom, lookDir, minB, maxB,
                                     t)) {
                    if (t < closestDist) {
                        closestDist = t;
                        hitIdx = i;
                    }
                }
            }

            if (hitIdx != -1) {
                if (input.leftClickPressed) {
                    state.draggedBenchIndex = hitIdx;
                } else if (input.rightClickPressed) {
                    state.activeVoidBenchIndex = hitIdx;
                    maximizeBenchContent(state);
                    state.savedPosition = state.position;
                    state.position =
                        vec3(0.0f, Config::VOID_Y_LEVEL + 5.0f, 0.0f);
                    state.inVoid = true;
                    state.yaw = 1.57f;
                    state.pitch = 0.0f;
                    state.ghostCube.setScale(1.0f);
                    state.materialDirty = true;
                } else if (input.lPressed) {
                    attemptSpawnLight(state, scene, hitIdx);
                }
            }
        }
    }
}

inline std::string getWindowTitle(const State &state, float fps) {
    if (state.inVoid) {
        MatStats stats = getMatStats(state.activeMaterialName);
        return "CREATIVE | Funds: $" + std::to_string((int)state.money) +
               " | Material: " + state.activeMaterialName + " | Cost: $" +
               std::to_string((int)stats.cost) + " | R:Return";
    }
    return std::string(Config::WINDOW_TITLE) + " | Funds: $" +
           std::to_string((int)state.money) +
           " | FPS: " + std::to_string((int)fps);
}

inline void onGameOver(const State &state) {
    std::cout << "Closed." << std::endl;
}

} // namespace Game

#endif // GAME_CUH