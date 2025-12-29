#define RENDER_MODE 1 // 0 = Path Tracing, 1 = Ray Tracing

// Platform Defines
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#endif

// STB Image
#define STB_IMAGE_IMPLEMENTATION

// Renderer Selection (Mutually Exclusive)
#if RENDER_MODE == 0
#define UNIFIED_SCENE_ENABLE_PT
#elif RENDER_MODE == 1
#define UNIFIED_SCENE_ENABLE_RT
#endif

// Includes

#define BLUE_NOISE_IMPLEMENTATION
#include "common/bluenoise.cuh"

#include "common/PTRTtransfer.cuh"

#if RENDER_MODE == 0
#include "pathtracer/app_utils.cuh"
#else
#include "raytracer/RTapp_utils.cuh"
#endif

// Game logic
#include "game.cuh"

#include <iostream>
#include <sstream>

// RENDERER ABSTRACTION

#if RENDER_MODE == 0
class Renderer {
  public:
    std::unique_ptr<Scene> scene;
    UnifiedScene *unifiedScene = nullptr;

    void init(UnifiedScene &uScene) {
        unifiedScene = &uScene;
        scene = UnifiedSceneBuilder::buildPTScene(uScene);

        scene->setPerformancePreset(Game::Config::PT_PRESET);
        scene->uploadToGPU();
    }

    void update() {
        if (unifiedScene) {
            UnifiedSceneBuilder::updatePTCamera(*scene, *unifiedScene);
            if (unifiedScene->hasDirtyMeshes()) {
                UnifiedSceneBuilder::updatePTScene(*scene, *unifiedScene);
            }
            scene->uploadToGPU();
        }
    }

    void render(uint8_t *devicePixels) {
        scene->render_to_device(devicePixels);
    }

    void cleanup() { scene.reset(); }
};

#else
class Renderer {
  public:
    std::unique_ptr<Scene> scene;
    UnifiedScene *unifiedScene = nullptr;

    void init(UnifiedScene &uScene) {
        unifiedScene = &uScene;
        scene = UnifiedSceneBuilder::buildRTScene(uScene);
        scene->uploadToGPU();
    }

    void update() {
        if (unifiedScene) {
            UnifiedSceneBuilder::updateRTCamera(*scene, *unifiedScene);
            if (unifiedScene->hasDirtyMeshes()) {
                UnifiedSceneBuilder::updateRTScene(*scene, *unifiedScene);
            }
            scene->uploadToGPU();
        }
    }

    void render(uint8_t *devicePixels) {
        scene->render_to_device(devicePixels);
    }

    void cleanup() { scene.reset(); }
};
#endif

// INPUT HANDLING

class InputManager {
  public:
    // Previous frame state for edge detection
    bool prevLeft = false;
    bool prevRight = false;
    bool prevJump = false;

    Game::Input poll(GLFWwindow *window) {
        Game::Input input;

        // Read current state
        bool left = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
        bool right = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
        bool jump = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
        bool boost = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        bool quit = (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS);

        // Edge-triggered (pressed this frame)
        input.leftPressed = left && !prevLeft;
        input.rightPressed = right && !prevRight;
        input.jumpPressed = jump && !prevJump;

        // Held state
        input.leftHeld = left;
        input.rightHeld = right;
        input.jumpHeld = jump;
        input.boostHeld = boost;
        input.quitPressed = quit;

        // Update previous state
        prevLeft = left;
        prevRight = right;
        prevJump = jump;

        return input;
    }
};

// MAIN ENTRY POINT

int main(int argc, char *argv[]) {
    try {
        // Print Info
        std::cout << "  A / D   - Move Left / Right\n";
        std::cout << "  SPACE   - Jump / Start / Restart\n";
        std::cout << "  W       - Turbo Speed\n";
        std::cout << "  ESC     - Quit\n";
#if RENDER_MODE == 0
        std::cout << "Renderer: Path Tracing\n";
#else
        std::cout << "Renderer: Ray Tracing\n";
#endif

        // Initialize Game State
        Game::State gameState;

        // Create Scene
        UnifiedScene scene(Game::Config::WINDOW_WIDTH,
                           Game::Config::WINDOW_HEIGHT);

        // Call Game Start
        Game::start(gameState, scene);

        // Initialize Renderer
        Renderer renderer;
        renderer.init(scene);

        // Initialize Window
        rtgl::InteropViewer viewer{};
        rtgl::init_interop_viewer(viewer, Game::Config::WINDOW_WIDTH,
                                  Game::Config::WINDOW_HEIGHT,
                                  Game::Config::WINDOW_TITLE, 0);

        // Initialize input
        InputManager inputManager;

        // Timing
        double previousTime = glfwGetTime();
        float fpsAccum = 0.0f;
        int frameCount = 0;
        float currentFps = 0.0f;

        initBlueNoise();

        // MAIN LOOP
        while (!glfwWindowShouldClose(viewer.window)) {
            // Calculate Delta Time
            double currentTime = glfwGetTime();
            float deltaTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;

            // Clamp delta time to prevent physics explosions
            if (deltaTime > 0.1f)
                deltaTime = 0.1f;

            // FPS calculation
            fpsAccum += deltaTime;
            frameCount++;
            if (fpsAccum >= 1.0f) {
                currentFps = frameCount / fpsAccum;
                fpsAccum = 0.0f;
                frameCount = 0;
            }

            // Poll Input
            Game::Input input = inputManager.poll(viewer.window);

            // Handle quit
            if (input.quitPressed) {
                glfwSetWindowShouldClose(viewer.window, 1);
                continue;
            }

            // Update Game Logic
            Game::update(gameState, scene, input, deltaTime);

            // Update Renderer
            renderer.update();

            // Render Frame
            size_t bufferSize = 0;
            uint8_t *devicePtr = rtgl::map_pbo_device_ptr(viewer, &bufferSize);

            if (bufferSize >=
                static_cast<size_t>(Game::Config::WINDOW_WIDTH *
                                    Game::Config::WINDOW_HEIGHT * 3)) {
                renderer.render(devicePtr);
            }

            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());

            rtgl::unmap_pbo(viewer);
            rtgl::blit_pbo_to_texture(viewer);
            rtgl::draw_interop(viewer);

            // Update Window Title
            std::string title = Game::getWindowTitle(gameState, currentFps);
            glfwSetWindowTitle(viewer.window, title.c_str());
        }

        // CLEANUP

        Game::onGameOver(gameState);
        renderer.cleanup();
        rtgl::destroy_interop_viewer(viewer);

        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }
}