// game.cuh
// Game Logic & Scene Setup

#ifndef GAME_CUH
#define GAME_CUH

#include "common/PTRTtransfer.cuh"
#include "common/vec3.cuh"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace Game {

// Configuration
struct Config {
    static constexpr int WINDOW_WIDTH = 1920;
    static constexpr int WINDOW_HEIGHT = 1080;
    static constexpr const char *WINDOW_TITLE = "Unified Gallery Walker";

    // Physics
    static constexpr float GRAVITY = -18.0f;
    static constexpr float WALK_SPEED = 5.0f;
    static constexpr float SPRINT_MULT = 2.0f;
    static constexpr float JUMP_FORCE = 8.0f;
    static constexpr float PLAYER_HEIGHT = 1.7f;
    static constexpr float MOUSE_SENSITIVITY = 0.002f;

    // Gameplay
    static constexpr float INTERACTION_DIST = 8.0f;
    static constexpr float SPIN_DURATION = 1.0f;
    static constexpr float INTERACTION_RADIUS = 1.5f;

    // rendering setting
    static constexpr const char *PT_PRESET = "quality";
};

// Input Structure
struct Input {
    bool w, a, s, d;
    bool e, r;
    bool shift, escape, space;
    bool jumpPressed, ePressed, rPressed;
    float mouseDX, mouseDY;
};

// Animation State
struct ObjectState {
    size_t meshIndex;
    vec3 basePosition;
    float baseRotationY;
    bool isSpinning = false;
    float spinTimer = 0.0f;
};

// Game State
struct State {
    vec3 position{0.0f, Config::PLAYER_HEIGHT, 10.0f};
    vec3 velocity{0.0f, 0.0f, 0.0f};
    float yaw = 3.14159f;
    float pitch = 0.0f;
    float bobTimer = 0.0f;
    bool isGrounded = false;
    std::vector<ObjectState> galleryObjects;
};

// Model Definition
struct ModelDef {
    std::string filename;
    UnifiedMaterial material;
    float scale;
    vec3 rotationOffset;
    std::string name;
};

// GAME FUNCTIONS

inline void start(State &state, UnifiedScene &scene) {
    // 1. Setup Environment (HDRI)
    scene.setHDRI("sky.hdr", 0.05f);

    // Low ambient light (HDRI provides most background light now)
    scene.setAmbientLight(vec3(0.00f));

    // 2. Define Gallery Models
    std::vector<ModelDef> gallery = {
        {"models/abraham-lincoln-mills-life-mask-150k.obj",
         UnifiedMaterial::Copper(), 0.016f, vec3(0, 0.05f, 0), "Lincoln"},
        {"models/andrew-jackson-zinc-sculpture-150k.obj",
         UnifiedMaterial::Silver(), 0.014f, vec3(0, 0.3f, 0), "Jackson"},
        {"models/george-washington-greenough-statue-(1840)-150k.obj",
         UnifiedMaterial::Diamond(), 0.0012f, vec3(0, -0.5f, 0), "Washington"},
        {"models/anne_sullivan_macy-150k.obj", UnifiedMaterial::Iron(), 0.16f,
         vec3(0, 0.5f, 0), "Sullivan"},
        {"models/cosmic-buddha-laser-scan-150k.obj", UnifiedMaterial::Gold(),
         0.007f, vec3(0, -0.5f, 0), "Buddha"},
        {"models/vase.obj", UnifiedMaterial::Chrome(), 0.007f, vec3(0, 0.3f, 0),
         "Vase"},
        {"models/x3d-cm-exterior-shell-90k-uvs.obj",
         UnifiedMaterial::PearlescentPaint(vec3(0.1f, 0.5f, 0.9f)), 0.01f,
         vec3(0, 0.3f, 0), "Shell"},
        {"models/full.obj", UnifiedMaterial::Glass(), 15.0f, vec3(0, 0.5f, 0),
         "Full"},
        {"models/halfway.obj", UnifiedMaterial::FrostedGlass(), 20.0f,
         vec3(0, -0.2f, 0), "Halfway"},
        {"models/ugly.obj", UnifiedMaterial::RubberBlack(), 20.0f,
         vec3(0, 0.5f, 0), "Ugly"}};

    // 3. Place Models
    float spacingZ = 8.0f;
    float startZ = 0.0f;
    float offsetX = 4.0f;

    for (size_t i = 0; i < gallery.size(); ++i) {
        const auto &item = gallery[i];

        float x = (i % 2 == 0) ? -offsetX : offsetX;
        float z = startZ - (i / 2) * spacingZ;
        float baseHeight = -1.0f;

        auto obj = scene.addMeshFromOBJ(item.filename, item.material);
        obj.setPosition(vec3(x, baseHeight, z))
            .setScale(item.scale)
            .setRotation(item.rotationOffset)
            .setName(item.name);

        scene.meshes[obj.index].isDynamic = true;

        float orientationY = (i % 2 == 0) ? 1.5708f : -1.5708f;
        obj.rotate(vec3(0, orientationY, 0));

        ObjectState objState;
        objState.meshIndex = obj.index;
        objState.basePosition = vec3(x, baseHeight, z);
        objState.baseRotationY = item.rotationOffset.y + orientationY;
        state.galleryObjects.push_back(objState);

        // OVERHEAD LIGHTING
        vec3 lightPos = vec3(x, baseHeight + 4.0f, z);
        vec3 lightDir = vec3(0.0f, -1.0f, 0.0f);

        // Intensity 150.0f
        scene.addSpotLight(lightPos, lightDir, vec3(1.0f, 0.98f, 0.9f), 150.0f,
                           0.8f, 1.2f, 45.0f, 0.1f);
    }
    scene.camera.setPosition(state.position);
}

inline void update(State &state, UnifiedScene &scene, const Input &input,
                   float dt) {
    // 1. Mouse Look
    state.yaw += input.mouseDX * Config::MOUSE_SENSITIVITY;
    state.pitch -= input.mouseDY * Config::MOUSE_SENSITIVITY;
    state.pitch = fmaxf(-1.5f, fminf(1.5f, state.pitch));

    // 2. Movement
    vec3 camDir;
    camDir.x = sin(state.yaw) * cos(state.pitch);
    camDir.y = sin(state.pitch);
    camDir.z = -cos(state.yaw) * cos(state.pitch);

    vec3 moveForward = vec3(sin(state.yaw), 0.0f, -cos(state.yaw)).normalized();
    vec3 moveRight = cross(moveForward, vec3(0, 1, 0)).normalized();

    vec3 inputDir(0.0f);
    if (input.w)
        inputDir = inputDir + moveForward;
    if (input.s)
        inputDir = inputDir - moveForward;
    if (input.d)
        inputDir = inputDir + moveRight;
    if (input.a)
        inputDir = inputDir - moveRight;

    if (inputDir.length_squared() > 0.001f)
        inputDir = inputDir.normalized();

    float speed = input.shift ? (Config::WALK_SPEED * Config::SPRINT_MULT)
                              : Config::WALK_SPEED;
    state.velocity.x = inputDir.x * speed;
    state.velocity.z = inputDir.z * speed;

    // Head Bob
    float bobOffsetX = 0.0f, bobOffsetY = 0.0f;
    if (state.isGrounded && inputDir.length_squared() > 0.001f) {
        float bobFreq = input.shift ? 15.0f : 10.0f;
        state.bobTimer += dt * bobFreq;
        bobOffsetY = sin(state.bobTimer) * 0.1f;
        bobOffsetX = cos(state.bobTimer / 2.0f) * 0.05f;
    } else {
        state.bobTimer = 0.0f;
    }

    // 3. Gravity
    if (state.isGrounded && input.jumpPressed) {
        state.velocity.y = Config::JUMP_FORCE;
        state.isGrounded = false;
    }
    state.velocity.y += Config::GRAVITY * dt;
    state.position = state.position + state.velocity * dt;

    if (state.position.y < Config::PLAYER_HEIGHT) {
        state.position.y = Config::PLAYER_HEIGHT;
        state.velocity.y = 0.0f;
        state.isGrounded = true;
    }

    // 4. Object Logic
    static float globalTime = 0.0f;
    globalTime += dt;

    vec3 camPos = state.position + vec3(bobOffsetX, bobOffsetY, 0.0f);

    for (auto &objState : state.galleryObjects) {
        auto *mesh = scene.getMesh(objState.meshIndex);
        if (!mesh)
            continue;

        // Oscillation
        float oscY = sin(globalTime) * 0.5f;
        vec3 newPos = objState.basePosition;
        newPos.y += oscY;
        mesh->transform.setPosition(newPos);

        if (objState.isSpinning) {
            objState.spinTimer += dt;
            float t = objState.spinTimer / Config::SPIN_DURATION;
            if (t >= 1.0f) {
                t = 1.0f;
                objState.isSpinning = false;
                objState.spinTimer = 0.0f;
            }
            float ease =
                (t < 0.5f) ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
            vec3 currentRot = mesh->transform.rotation;
            currentRot.y = objState.baseRotationY + (ease * 6.28318f);
            mesh->transform.setRotation(currentRot);

        } else if (input.ePressed) {
            vec3 interactTarget = newPos;
            vec3 toTarget = interactTarget - camPos;

            if (toTarget.length() < Config::INTERACTION_DIST) {
                float t = dot(toTarget, camDir);
                if (t > 0) {
                    vec3 closestPointOnRay = camPos + camDir * t;
                    if ((interactTarget - closestPointOnRay).length() <
                        Config::INTERACTION_RADIUS) {
                        objState.isSpinning = true;
                        objState.spinTimer = 0.0f;
                    }
                }
            }
        }
        scene.markMeshDirty(objState.meshIndex);
    }

    scene.camera.setPosition(camPos);
    scene.camera.setTarget(camPos + camDir);
}

inline std::string getWindowTitle(const State &state, float fps) {
    return std::string(Config::WINDOW_TITLE) +
           " | FPS: " + std::to_string((int)fps) + " | [E] Spin";
}

inline void onGameOver(const State &state) {
    std::cout << "Exiting..." << std::endl;
}

} // namespace Game

#endif // GAME_CUH