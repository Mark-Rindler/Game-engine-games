// MAIN.CU
// Modified to render exactly ONE frame and then hold that image

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

// Renderer Selection
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

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <limits>

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
    bool prevJump = false;
    bool prevInteract = false;
    bool prevScan = false;
    bool firstMouse = true;
    double lastX = 0, lastY = 0;

    void init(GLFWwindow *window) {
        // Hide cursor and capture it
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Initialize last position to center
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        lastX = w / 2.0;
        lastY = h / 2.0;
    }

    Game::Input poll(GLFWwindow *window) {
        Game::Input input{};

        // Keyboard (raw state)
        input.w = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        input.a = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
        input.s = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        input.d = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
        input.e = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS);
        input.r = (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS);
        input.shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
        input.escape = (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS);

        // Jump edge
        bool jump = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
        input.space = jump;
        input.jumpPressed = jump && !prevJump;
        prevJump = jump;

        // E edge
        input.ePressed = input.e && !prevInteract;
        prevInteract = input.e;

        // R edge
        input.rPressed = input.r && !prevScan;
        prevScan = input.r;

        // Mouse
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        input.mouseDX = static_cast<float>(xpos - lastX);
        input.mouseDY = static_cast<float>(ypos - lastY);

        lastX = xpos;
        lastY = ypos;

        return input;
    }
};

// MAIN ENTRY POINT
int main(int argc, char *argv[]) {
    try {
        // Print Info
        std::cout << "  MOUSE   - Look Around\n";
        std::cout << "  W/A/S/D - Move\n";
        std::cout << "  SHIFT   - Sprint\n";
        std::cout << "  SPACE   - Jump\n";
        std::cout << "  ESC     - Quit\n";
        std::cout << "  [INFO]  Rendering ONE frame then holding state.\n";

#if RENDER_MODE == 0
        std::cout << "Renderer: Path Tracing\n";
#else
        std::cout << "Renderer: Ray Tracing\n";
#endif

        // Initialize Window
        rtgl::InteropViewer viewer{};
        rtgl::init_interop_viewer(viewer, Game::Config::WINDOW_WIDTH,
                                  Game::Config::WINDOW_HEIGHT,
                                  Game::Config::WINDOW_TITLE, 0);

        // Initialize Game State & Scene
        Game::State gameState;
        UnifiedScene scene(Game::Config::WINDOW_WIDTH,
                           Game::Config::WINDOW_HEIGHT);

        Game::start(gameState, scene);

        // Initialize Renderer
        Renderer renderer;
        renderer.init(scene);

        // Initialize Input
        InputManager inputManager;
        inputManager.init(viewer.window);

        // Timing
        double previousTime = glfwGetTime();

        // For display FPS
        double fpsWindowAccum = 0.0;
        int fpsWindowFrames = 0;
        float currentFps = 0.0f;

        initBlueNoise();

        // ** Flag to ensure we only render once **
        bool hasRendered = false;

        // MAIN LOOP
        while (!glfwWindowShouldClose(viewer.window)) {
            // Calculate Delta Time
            double currentTime = glfwGetTime();
            float deltaTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;

            if (deltaTime > 0.1f)
                deltaTime = 0.1f;

            // FPS stats
            fpsWindowAccum += deltaTime;
            fpsWindowFrames++;

            if (fpsWindowAccum >= 1.0) {
                currentFps =
                    static_cast<float>(fpsWindowFrames / fpsWindowAccum);
                fpsWindowAccum = 0.0;
                fpsWindowFrames = 0;
            }

            // Poll Input
            Game::Input input = inputManager.poll(viewer.window);

            if (input.escape) {
                glfwSetWindowShouldClose(viewer.window, 1);
                continue;
            }

            // Update Game Logic
            Game::update(gameState, scene, input, deltaTime);

            // Update Renderer
            renderer.update();

            // ** SINGLE FRAME RENDER LOGIC **
            if (!hasRendered) {
                size_t bufferSize = 0;
                // Map the PBO so CUDA can write to it
                uint8_t *devicePtr =
                    rtgl::map_pbo_device_ptr(viewer, &bufferSize);

                if (bufferSize >=
                    static_cast<size_t>(Game::Config::WINDOW_WIDTH *
                                        Game::Config::WINDOW_HEIGHT * 3)) {
                    renderer.render(devicePtr);

                    // IMPORTANT: Ensure GPU finishes writing before unmapping
                    CUDA_CHECK(cudaGetLastError());
                    CUDA_CHECK(cudaDeviceSynchronize());
                }

                // Unmap PBO
                rtgl::unmap_pbo(viewer);

                // Mark as done so we don't map/render/unmap again
                hasRendered = true;
                std::cout << "[INFO] Frame 1 rendered. Freezing output.\n";
            }

            rtgl::blit_pbo_to_texture(viewer);
            rtgl::draw_interop(viewer);

            // Update Window Title
            std::string title = Game::getWindowTitle(gameState, currentFps);
            glfwSetWindowTitle(viewer.window, title.c_str());
        }

        // Cleanup
        Game::onGameOver(gameState);
        renderer.cleanup();
        rtgl::destroy_interop_viewer(viewer);

        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }
}