
// GAME.CUH  Cube Runner Game Logic

#pragma once

#include "common/PTRTtransfer.cuh"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace Game {

struct Input {
    bool leftPressed = false;
    bool rightPressed = false;
    bool jumpPressed = false;
    bool boostHeld = false;
    bool quitPressed = false;

    bool leftHeld = false;
    bool rightHeld = false;
    bool jumpHeld = false;
};

// GAME CONFIGURATION

namespace Config {
// window
constexpr int WINDOW_WIDTH = 1920;
constexpr int WINDOW_HEIGHT = 1080;
inline const char *WINDOW_TITLE = "Cube Runner";

// lane + player
constexpr int NUM_LANES = 5;
constexpr float LANE_WIDTH = 2.5f;
constexpr float CUBE_SIZE = LANE_WIDTH;
constexpr float PLAYER_START_Y = 0.0f;
constexpr float PLAYER_Z = 0.0f;
constexpr float MOVE_DURATION = 0.08f;
constexpr float GRAVITY = 55.0f;
constexpr float JUMP_FORCE = 18.0f;

// platform
constexpr float PLATFORM_Y = -1.25f;
constexpr float PLATFORM_WIDTH = 18.0f;
constexpr float PLATFORM_LENGTH = 150.0f;
constexpr float PLATFORM_Z_OFFSET = 40.0f;

// camera
inline const vec3 CAMERA_BASE_POS = vec3(0, 9, -19);
inline const vec3 CAMERA_BASE_TARGET = vec3(0, 0, 25);
constexpr float CAMERA_FOV = 60.0f;
constexpr float CAMERA_FOLLOW_STRENGTH = 0.05f;

// camera tilt config
constexpr float CAMERA_MAX_ROLL = 3.0f;
constexpr float CAMERA_ROLL_SPEED = 5.0f;

// Enemies
constexpr float ENEMY_SPAWN_Z = 150.0f;
constexpr float ENEMY_DESPAWN_Z = -40.0f;
constexpr float ENEMY_WAITING_Z = 200.0f;
constexpr int MAX_ENEMIES = 64;
constexpr float ENEMY_START_SPEED = 15.0f;
constexpr float ENEMY_SPEED_INC = 0.8f;
constexpr float WAVE_START_DIST = 40.0f;
constexpr float WAVE_DIST_INC = 1.0f;

// Collision
constexpr float COLLISION_SIZE = CUBE_SIZE * 0.9f;

// boss
constexpr int BOSS_WAVE_INTERVAL = 20;
constexpr float BOSS_OSC_SPEED = 3.0f;

// rendering setting
static constexpr const char *PT_PRESET = "quality";

} // namespace Config

// MATERIALS AND ENEMIES

namespace Materials {
inline UnifiedMaterial generateRandom(std::mt19937 &rng) {
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);
    UnifiedMaterial m;
    m.albedo = vec3(dist01(rng), dist01(rng), dist01(rng));
    m.roughness = 0.05f + (dist01(rng) * 0.9f);
    float metRoll = dist01(rng);
    m.metallic =
        (metRoll > 0.7f) ? 1.0f : ((metRoll < 0.5f) ? 0.0f : dist01(rng));
    m.specular = ::lerp(vec3(0.04f), m.albedo, m.metallic);
    m.emission = (dist01(rng) > 0.85f) ? m.albedo * (0.5f + dist01(rng) * 1.5f)
                                       : vec3(0.0f);
    if (m.metallic < 0.1f && dist01(rng) > 0.90f) {
        m.transmission = 0.8f + (dist01(rng) * 0.2f);
        m.ior = 1.1f + dist01(rng) * 1.0f;
        m.roughness = dist01(rng) * 0.3f;
    }
    return m;
}

inline UnifiedMaterial getBoss() {
    UnifiedMaterial m;
    m.albedo = vec3(1.0f, 0.0f, 0.0f);
    m.emission = vec3(2.0f, 0.1f, 0.1f);
    m.roughness = 0.1f;
    m.metallic = 1.0f;
    return m;
}

inline UnifiedMaterial getNeonFloor(float time) {
    UnifiedMaterial m;
    m.albedo = vec3(0.02f);
    m.roughness = 0.2f;
    m.metallic = 0.8f;

    float pulse = 0.5f + 0.5f * sinf(time * 3.0f);
    vec3 colorA = vec3(0.0f, 0.2f, 0.4f);
    vec3 colorB = vec3(0.4f, 0.0f, 0.4f);

    m.emission = ::lerp(colorA, colorB, pulse) * (1.0f + pulse);
    return m;
}
} // namespace Materials

enum class EnemyType { NORMAL, BOSS };

struct Enemy {
    int lane = 0;
    float x = 0.0f;
    float z = Config::ENEMY_WAITING_Z;
    EnemyType type = EnemyType::NORMAL;
    bool isTall = false;
    float oscCenter = 0.0f;
    float oscTimeOffset = 0.0f;
    ObjectHandle handle;
    bool active = false;
    bool scored = false;
};

enum class SpawnPattern {
    RANDOM,
    WALL_WITH_GAP,
    STAIRS,
    ZIGZAG,
    CHECKERS,
    V_FORMATION,
    DOUBLE_GAP
};

// GAME STATE

class State {
  public:
    int currentLaneIndex = 2;
    int targetLaneIndex = 2;
    float playerX = 0.0f;

    float playerY = Config::PLAYER_START_Y;
    float verticalVel = 0.0f;
    bool isGrounded = true;

    bool isMoving = false;
    float moveTime = 0.0f;
    float startX = 0.0f;
    float targetX = 0.0f;
    int queuedLaneChange = 0;

    float currentCameraRoll = 0.0f;

    // Death Physics state
    vec3 deathVelocity{0.0f, 0.0f, 0.0f};
    vec3 deathRotation{0.0f, 0.0f, 0.0f};

    ObjectHandle playerHandle;
    ObjectHandle platformHandle;

    std::vector<Enemy> enemies;

    float spawnTimer = 0.0f;
    float gameTime = 0.0f;

    int score = 0;
    int highScore = 0;
    int enemiesSpawnedSinceBoss = 0;

    bool gameOver = false;
    bool started = false;

    std::mt19937 rng;

    State() {
        auto seed = std::chrono::high_resolution_clock::now()
                        .time_since_epoch()
                        .count();
        rng.seed(static_cast<unsigned int>(seed));
        enemies.resize(Config::MAX_ENEMIES);
        playerX = getLaneX(2);
    }

    float getLaneX(int lane) const {
        return (lane - (Config::NUM_LANES / 2)) * Config::LANE_WIDTH;
    }

    float getEnemySpeed() const {
        return Config::ENEMY_START_SPEED + (score * Config::ENEMY_SPEED_INC);
    }

    float getSpawnInterval() const {
        float speed = getEnemySpeed();
        float targetDistance =
            Config::WAVE_START_DIST + (score * Config::WAVE_DIST_INC);
        return targetDistance / speed;
    }

    float easeInOut(float t) const {
        if (t < 0.0f)
            return 0.0f;
        if (t > 1.0f)
            return 1.0f;
        return t * t * (3.0f - 2.0f * t);
    }

    void reset() {
        currentLaneIndex = 2;
        targetLaneIndex = 2;
        isMoving = false;
        queuedLaneChange = 0;
        moveTime = 0.0f;
        playerX = getLaneX(2);
        playerY = Config::PLAYER_START_Y;
        verticalVel = 0.0f;
        isGrounded = true;
        currentCameraRoll = 0.0f;
        spawnTimer = 0.0f;
        gameTime = 0.0f;
        enemiesSpawnedSinceBoss = 0;

        // Reset player visuals
        playerHandle.setRotation(vec3(0.0f));
        playerHandle.setScale(vec3(Config::CUBE_SIZE));

        if (score > highScore)
            highScore = score;
        score = 0;
        gameOver = false;
        for (auto &enemy : enemies) {
            enemy.active = false;
            enemy.z = Config::ENEMY_WAITING_Z;
            enemy.scored = false;
        }
    }

    void initiateMove(int direction) {
        int newIndex = currentLaneIndex + direction;
        if (newIndex >= 0 && newIndex < Config::NUM_LANES) {
            targetLaneIndex = newIndex;
            isMoving = true;
            moveTime = 0.0f;
            startX = playerX;
            targetX = getLaneX(targetLaneIndex);
        }
    }

    void jump() {
        if (isGrounded) {
            verticalVel = Config::JUMP_FORCE;
            isGrounded = false;
        }
    }

    Enemy *activateEnemy(int lane, float xPos, EnemyType type) {
        for (auto &enemy : enemies) {
            if (!enemy.active) {
                enemy.active = true;
                enemy.lane = lane;
                enemy.x = xPos;
                enemy.z = Config::ENEMY_SPAWN_Z;
                enemy.type = type;
                enemy.scored = false;
                if (type == EnemyType::NORMAL) {
                    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
                    enemy.isTall = (dist(rng) < 0.3f);
                } else {
                    enemy.isTall = false;
                }
                if (type == EnemyType::BOSS) {
                    enemy.oscCenter = getLaneX(2);
                    enemy.oscTimeOffset = gameTime;
                }
                return &enemy;
            }
        }
        return nullptr;
    }

    // Expanded Wave Logic
    bool spawnEnemyGroup() {
        bool isBossWave = (score > 0 && (score / Config::BOSS_WAVE_INTERVAL) >
                                            enemiesSpawnedSinceBoss);
        if (isBossWave) {
            enemiesSpawnedSinceBoss++;
            activateEnemy(2, getLaneX(2), EnemyType::BOSS);
            return true;
        }

        SpawnPattern pat = SpawnPattern::RANDOM;

        // Difficulty curve for patterns
        std::vector<SpawnPattern> pool;
        pool.push_back(SpawnPattern::RANDOM);
        pool.push_back(SpawnPattern::RANDOM); // Weight random higher initially

        if (score > 5)
            pool.push_back(SpawnPattern::WALL_WITH_GAP);
        if (score > 10)
            pool.push_back(SpawnPattern::DOUBLE_GAP);
        if (score > 15)
            pool.push_back(SpawnPattern::CHECKERS);
        if (score > 20)
            pool.push_back(SpawnPattern::STAIRS);
        if (score > 25)
            pool.push_back(SpawnPattern::ZIGZAG);
        if (score > 30)
            pool.push_back(SpawnPattern::V_FORMATION);

        std::uniform_int_distribution<size_t> poolDist(0, pool.size() - 1);
        pat = pool[poolDist(rng)];

        switch (pat) {
        case SpawnPattern::WALL_WITH_GAP: {
            int gap = std::uniform_int_distribution<int>(0, Config::NUM_LANES -
                                                                1)(rng);
            for (int i = 0; i < Config::NUM_LANES; ++i) {
                if (i == gap)
                    continue;
                activateEnemy(i, getLaneX(i), EnemyType::NORMAL);
            }
            break;
        }
        case SpawnPattern::DOUBLE_GAP: {
            // Two random holes
            std::vector<int> lanes(Config::NUM_LANES);
            std::iota(lanes.begin(), lanes.end(), 0);
            std::shuffle(lanes.begin(), lanes.end(), rng);
            for (int i = 0; i < 3; ++i) {
                activateEnemy(lanes[i], getLaneX(lanes[i]), EnemyType::NORMAL);
            }
            break;
        }
        case SpawnPattern::CHECKERS: {
            for (int i = 0; i < Config::NUM_LANES; i += 2) {
                activateEnemy(i, getLaneX(i), EnemyType::NORMAL);
            }
            break;
        }
        case SpawnPattern::V_FORMATION: {
            Enemy *e;
            e = activateEnemy(2, getLaneX(2), EnemyType::NORMAL);
            if (e)
                e->z -= 10.0f;

            e = activateEnemy(1, getLaneX(1), EnemyType::NORMAL);

            e = activateEnemy(3, getLaneX(3), EnemyType::NORMAL);

            e = activateEnemy(0, getLaneX(0), EnemyType::NORMAL);
            if (e)
                e->z += 10.0f;

            e = activateEnemy(4, getLaneX(4), EnemyType::NORMAL);
            if (e)
                e->z += 10.0f;
            break;
        }
        case SpawnPattern::STAIRS: {
            bool leftToRight = (rng() % 2 == 0);
            for (int i = 0; i < Config::NUM_LANES; ++i) {
                int lane = leftToRight ? i : (Config::NUM_LANES - 1 - i);
                Enemy *e =
                    activateEnemy(lane, getLaneX(lane), EnemyType::NORMAL);
                if (e) {
                    e->z += i * 12.0f;
                    e->isTall = false;
                }
            }
            break;
        }
        case SpawnPattern::ZIGZAG: {
            int center = Config::NUM_LANES / 2;
            Enemy *e1 =
                activateEnemy(center, getLaneX(center), EnemyType::NORMAL);
            Enemy *e2 = activateEnemy(center - 1, getLaneX(center - 1),
                                      EnemyType::NORMAL);
            Enemy *e3 = activateEnemy(center + 1, getLaneX(center + 1),
                                      EnemyType::NORMAL);
            if (e1)
                e1->z += (rng() % 2 == 0) ? 15.0f : -15.0f;
            break;
        }
        default: {
            std::vector<int> lanes(Config::NUM_LANES);
            std::iota(lanes.begin(), lanes.end(), 0);
            std::shuffle(lanes.begin(), lanes.end(), rng);
            int numCubes = 1 + (score / 15);
            if (numCubes >= Config::NUM_LANES)
                numCubes = Config::NUM_LANES - 1;
            for (int i = 0; i < numCubes; ++i) {
                activateEnemy(lanes[i], getLaneX(lanes[i]), EnemyType::NORMAL);
            }
            break;
        }
        }
        return true;
    }
};

// LIFECYCLE

inline void start(State &state, UnifiedScene &scene) {
    scene.setCamera(Config::CAMERA_BASE_POS, Config::CAMERA_BASE_TARGET,
                    vec3(0, 1, 0), Config::CAMERA_FOV);
    scene.setBVHParams(2, 4);

    state.platformHandle =
        scene.addCube(UnifiedMaterial::MarbleNero())
            .setPosition(vec3(0, Config::PLATFORM_Y, Config::PLATFORM_Z_OFFSET))
            .setScale(
                vec3(Config::PLATFORM_WIDTH, 0.5f, Config::PLATFORM_LENGTH))
            .setName("platform")
            .setDynamic(false);

    vec3 initialPos = vec3(state.getLaneX(state.currentLaneIndex),
                           Config::PLAYER_START_Y, Config::PLAYER_Z);
    UnifiedMaterial playerMat = UnifiedMaterial::Gold();
    playerMat.emission = vec3(0.0f);
    state.playerHandle = scene.addCube(playerMat)
                             .setPosition(initialPos)
                             .setScale(vec3(Config::CUBE_SIZE))
                             .setName("player")
                             .setDynamic(true);

    for (int i = 0; i < Config::MAX_ENEMIES; ++i) {
        UnifiedMaterial mat = (i >= Config::MAX_ENEMIES - 5)
                                  ? Materials::getBoss()
                                  : Materials::generateRandom(state.rng);
        state.enemies[i].handle =
            scene.addCube(mat)
                .setPosition(vec3(0, 0, Config::ENEMY_WAITING_Z))
                .setScale(vec3(Config::CUBE_SIZE))
                .setName("enemy_" + std::to_string(i))
                .setDynamic(true);
    }

    scene.addDirectionalLight(vec3(-0.5f, -1.0f, 0.2f),
                              vec3(1.0f, 0.98f, 0.95f), 3.0f);
    scene.addPointLight(vec3(0, 10, 40), vec3(1.0f, 0.9f, 0.8f), 100.0f, 60.0f,
                        1.0f);
    scene.addPointLight(vec3(0, 5, -10), vec3(0.6f, 0.7f, 1.0f), 30.0f, 40.0f,
                        0.5f);
    scene.setSkyGradient(vec3(0.1f, 0.1f, 0.2f), vec3(0.4f, 0.3f, 0.6f));
    scene.setAmbientLight(vec3(0.05f));
    scene.setPathTracerParams(1, 4);
    std::cout << "Game initialized!\n";
}

inline void update(State &state, UnifiedScene &scene, const Input &input,
                   float deltaTime) {

    // 1. Initial Start Logic
    if (!state.started) {
        if (input.jumpPressed) {
            state.started = true;
        }
        return;
    }

    float effectiveDt = deltaTime * (input.boostHeld ? 1.5f : 1.0f);

    // 2. Global Updates
    state.gameTime += deltaTime;
    if (state.platformHandle.isValid()) {
        state.platformHandle.setMaterial(
            Materials::getNeonFloor(state.gameTime));
    }

    // 3. Game Over / Death Animation Logic
    if (state.gameOver) {
        // Ragdoll physics
        state.deathVelocity.y -= Config::GRAVITY * deltaTime;
        state.playerX += state.deathVelocity.x * deltaTime;
        state.playerY += state.deathVelocity.y * deltaTime;

        if (state.playerY < Config::PLAYER_START_Y) {
            state.playerY = Config::PLAYER_START_Y;
            state.deathVelocity.y *= -0.5f;
            state.deathVelocity.x *= 0.9f;
            state.deathRotation *= 0.8f;
        }

        state.playerHandle.setPosition(
            vec3(state.playerX, state.playerY, Config::PLAYER_Z));
        state.playerHandle.rotate(state.deathRotation * deltaTime);

        if (input.jumpPressed) {
            state.reset();
            state.started = true;
        }
        return;
    }

    // Standard Gameplay Logic

    // Camera Tilt
    float targetRoll = 0.0f;
    if (state.isMoving) {
        targetRoll = (state.targetLaneIndex > state.currentLaneIndex)
                         ? -Config::CAMERA_MAX_ROLL
                         : Config::CAMERA_MAX_ROLL;
    }
    float tRoll = Config::CAMERA_ROLL_SPEED * effectiveDt;
    state.currentCameraRoll += (targetRoll - state.currentCameraRoll) * tRoll;
    float rads = state.currentCameraRoll * (3.14159f / 180.0f);
    vec3 upVec(sinf(rads), cosf(rads), 0.0f);
    float camFollowX = state.playerX * Config::CAMERA_FOLLOW_STRENGTH;
    vec3 newCamPos = Config::CAMERA_BASE_POS + vec3(camFollowX, 0, 0);
    scene.setCamera(newCamPos, Config::CAMERA_BASE_TARGET, upVec,
                    Config::CAMERA_FOV);

    // Player Movement
    if (input.jumpPressed)
        state.jump();

    if (state.isMoving) {
        if (input.leftPressed)
            state.queuedLaneChange = 1;
        if (input.rightPressed)
            state.queuedLaneChange = -1;
    } else {
        if (input.leftPressed)
            state.initiateMove(1);
        else if (input.rightPressed)
            state.initiateMove(-1);
    }

    if (state.isMoving) {
        state.moveTime += deltaTime;
        float t = state.moveTime / Config::MOVE_DURATION;
        if (t >= 1.0f) {
            state.playerX = state.targetX;
            state.currentLaneIndex = state.targetLaneIndex;
            state.isMoving = false;
            if (state.queuedLaneChange != 0) {
                state.initiateMove(state.queuedLaneChange);
                state.queuedLaneChange = 0;
            }
        } else {
            state.playerX = state.startX +
                            (state.targetX - state.startX) * state.easeInOut(t);
        }
    }

    // Enemy Collision Check
    float potentialSupportY = -999.0f;
    for (const auto &enemy : state.enemies) {
        if (!enemy.active)
            continue;
        float distX = fabsf(state.playerX - enemy.x);
        float distZ = fabsf(Config::PLAYER_Z - enemy.z);
        if (distX < Config::COLLISION_SIZE && distZ < Config::COLLISION_SIZE) {
            float enemyScaleY =
                enemy.isTall ? Config::CUBE_SIZE * 2.0f : Config::CUBE_SIZE;
            float enemyPhysicalTop = Config::PLATFORM_Y + enemyScaleY;
            float playerPhysicalBottom =
                state.playerY - (Config::CUBE_SIZE / 2.0f);

            if (playerPhysicalBottom >= enemyPhysicalTop - 0.5f) {
                float supportCenterY =
                    enemyPhysicalTop + (Config::CUBE_SIZE / 2.0f);
                if (supportCenterY > potentialSupportY)
                    potentialSupportY = supportCenterY;
            } else {
                // CRASH!
                state.gameOver = true;
                if (state.score > state.highScore)
                    state.highScore = state.score;

                std::uniform_real_distribution<float> randForce(-5.0f, 5.0f);
                std::uniform_real_distribution<float> randRot(-8.0f, 8.0f);

                state.deathVelocity = vec3(randForce(state.rng), 15.0f, 0.0f);
                state.deathRotation = vec3(
                    randRot(state.rng), randRot(state.rng), randRot(state.rng));

                std::cout << "CRASH! Score: " << state.score << "\n";
                return;
            }
        }
    }

    if (!state.isGrounded) {
        state.verticalVel -= Config::GRAVITY * deltaTime;
        state.playerY += state.verticalVel * deltaTime;
    }

    float floorY = (potentialSupportY > -900.0f) ? potentialSupportY
                                                 : Config::PLAYER_START_Y;
    if (state.playerY <= floorY) {
        state.playerY = floorY;
        state.verticalVel = 0.0f;
        state.isGrounded = true;
    } else {
        if (state.isGrounded && floorY < state.playerY - 0.1f)
            state.isGrounded = false;
    }

    state.playerHandle.setPosition(
        vec3(state.playerX, state.playerY, Config::PLAYER_Z));
    UnifiedMaterial playerMat = UnifiedMaterial::Gold();
    playerMat.emission = vec3(0.0f);
    state.playerHandle.setMaterial(playerMat);

    state.spawnTimer += effectiveDt;
    if (state.spawnTimer >= state.getSpawnInterval()) {
        if (state.spawnEnemyGroup())
            state.spawnTimer = 0.0f;
    }

    float enemySpeed = state.getEnemySpeed();
    for (auto &enemy : state.enemies) {
        if (!enemy.active) {
            enemy.handle.setPosition(vec3(0, 0, Config::ENEMY_WAITING_Z));
            continue;
        }
        enemy.z -= enemySpeed * effectiveDt;
        if (enemy.type == EnemyType::BOSS) {
            float t =
                (state.gameTime - enemy.oscTimeOffset) * Config::BOSS_OSC_SPEED;
            enemy.x = enemy.oscCenter + sinf(t) * Config::LANE_WIDTH;
        }
        vec3 scale = vec3(Config::CUBE_SIZE);
        float posY = 0.0f;
        if (enemy.isTall) {
            scale.y = Config::CUBE_SIZE * 2.0f;
            posY = Config::CUBE_SIZE / 2.0f;
        }
        enemy.handle.setScale(scale);
        enemy.handle.setPosition(vec3(enemy.x, posY, enemy.z));

        if (!enemy.scored && enemy.z < Config::PLAYER_Z - Config::CUBE_SIZE) {
            enemy.scored = true;
            state.score++;
        }
        if (enemy.z < Config::ENEMY_DESPAWN_Z) {
            enemy.active = false;
            enemy.z = Config::ENEMY_WAITING_Z;
        }
    }
}

inline std::string getWindowTitle(const State &state, float fps) {
    std::stringstream ss;
    if (!state.started)
        ss << "CUBE RUNNER - Press SPACE to Start";
    else if (state.gameOver)
        ss << "GAME OVER! Score: " << state.score
           << " | High Score: " << state.highScore
           << " | Press SPACE to Restart";
    else {
        ss << "Score: " << state.score
           << " | Lane: " << (state.targetLaneIndex + 1) << " / "
           << Config::NUM_LANES;
        if (fps > 0)
            ss << " | FPS: " << static_cast<int>(fps);
    }
    return ss.str();
}

inline void onGameOver(State &state) {
    std::cout << "\nThanks for playing!\n";
    std::cout << "Final High Score: " << state.highScore << "\n";
}

} // namespace Game