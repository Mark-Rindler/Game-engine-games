// MAIN.CU - Application Entry Point & Window Management
#define RENDER_MODE 0 // 0 = Path Tracing, 1 = Ray Tracing

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

#include "game.cuh"
#include <iostream>

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

class InputManager;
static InputManager *g_InputManager = nullptr;

// INPUT HANDLING
class InputManager {
  public:
    Game::Input accumulatedInput{};
    double lastX = 0, lastY = 0;
    bool firstMouse = true;

    // Static callback using Global Pointer
    static void key_callback(GLFWwindow *window, int key, int scancode,
                             int action, int mods) {
        // Use global pointer instead of glfwGetWindowUserPointer
        if (!g_InputManager)
            return;
        InputManager *mgr = g_InputManager;

        if (action == GLFW_PRESS) {
            if (key == GLFW_KEY_ESCAPE)
                mgr->accumulatedInput.escape = true;

            // F11 TOGGLE
            if (key == GLFW_KEY_F11) {
                mgr->accumulatedInput.f11Pressed = true;
            }

            if (key == GLFW_KEY_E)
                mgr->accumulatedInput.interactPressed = true;
            if (key == GLFW_KEY_B)
                mgr->accumulatedInput.bPressed = true;
            if (key == GLFW_KEY_R)
                mgr->accumulatedInput.rPressed = true;
            if (key == GLFW_KEY_L)
                mgr->accumulatedInput.lPressed = true;
            if (key == GLFW_KEY_SPACE)
                mgr->accumulatedInput.spacePressed = true;
            if (key == GLFW_KEY_I)
                mgr->accumulatedInput.iPressed = true;
            if (key == GLFW_KEY_J)
                mgr->accumulatedInput.jPressed = true;
            if (key == GLFW_KEY_K)
                mgr->accumulatedInput.kPressed = true;
        }
    }

    static void mouse_button_callback(GLFWwindow *window, int button,
                                      int action, int mods) {
        if (!g_InputManager)
            return;
        InputManager *mgr = g_InputManager;

        if (action == GLFW_PRESS) {
            if (button == GLFW_MOUSE_BUTTON_LEFT)
                mgr->accumulatedInput.leftClickPressed = true;
            if (button == GLFW_MOUSE_BUTTON_RIGHT)
                mgr->accumulatedInput.rightClickPressed = true;
        }
    }

    static void cursor_position_callback(GLFWwindow *window, double xpos,
                                         double ypos) {
        if (!g_InputManager)
            return;
        InputManager *mgr = g_InputManager;

        if (mgr->firstMouse) {
            mgr->lastX = xpos;
            mgr->lastY = ypos;
            mgr->firstMouse = false;
        }

        mgr->accumulatedInput.mouseDX += static_cast<float>(xpos - mgr->lastX);
        mgr->accumulatedInput.mouseDY += static_cast<float>(ypos - mgr->lastY);
        mgr->lastX = xpos;
        mgr->lastY = ypos;
    }

    void init(GLFWwindow *window) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        g_InputManager = this;

        glfwSetKeyCallback(window, key_callback);
        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetCursorPosCallback(window, cursor_position_callback);

        int w, h;
        glfwGetWindowSize(window, &w, &h);
        lastX = w / 2.0;
        lastY = h / 2.0;
    }

    // Polling logic
    Game::Input poll(GLFWwindow *window) {
        Game::Input current = accumulatedInput;

        // Reset "One-Shot" events
        accumulatedInput.leftClickPressed = false;
        accumulatedInput.rightClickPressed = false;
        accumulatedInput.interactPressed = false;
        accumulatedInput.bPressed = false;
        accumulatedInput.rPressed = false;
        accumulatedInput.lPressed = false;
        accumulatedInput.spacePressed = false;
        accumulatedInput.iPressed = false;
        accumulatedInput.jPressed = false;
        accumulatedInput.kPressed = false;

        // Reset F11
        accumulatedInput.f11Pressed = false;

        accumulatedInput.mouseDX = 0.0f;
        accumulatedInput.mouseDY = 0.0f;

        // Poll continuous states
        current.w = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        current.a = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
        current.s = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        current.d = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
        current.shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
        current.space = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
        current.ctrl =
            (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);

        current.leftClick =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        current.rightClick =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        return current;
    }
};

// MAIN ENTRY POINT
int main(int argc, char *argv[]) {
    try {
        std::cout << "  MOUSE   - Look Around\n";
        std::cout << "  W/A/S/D - Move\n";
        std::cout << "  SHIFT   - Turbo Speed\n";
        std::cout << "  SPACE   - Fly Up\n";
        std::cout << "  CTRL    - Fly Down\n";
        std::cout << "  R       - Return to Gallery\n";
        std::cout << "  E       - Interact\n";
        std::cout << "  B       - Spawn Bench\n";
        std::cout << "  L       - Buy Overhead Light ($500)\n";
        std::cout << "  ESC     - Quit\n";

#if RENDER_MODE == 0
        std::cout << "Renderer: Path Tracing\n";
#else
        std::cout << "Renderer: Ray Tracing\n";
#endif

        rtgl::InteropViewer viewer{};
        rtgl::init_interop_viewer(viewer, Game::Config::WINDOW_WIDTH,
                                  Game::Config::WINDOW_HEIGHT,
                                  Game::Config::WINDOW_TITLE, 0);

        Game::State gameState;
        UnifiedScene scene(Game::Config::WINDOW_WIDTH,
                           Game::Config::WINDOW_HEIGHT);

        Game::start(gameState, scene);

        Renderer renderer;
        renderer.init(scene);

        InputManager inputManager;
        inputManager.init(viewer.window);

        double previousTime = glfwGetTime();
        float fpsAccum = 0.0f;
        int frameCount = 0;
        float currentFps = 0.0f;

        initBlueNoise();

        while (!glfwWindowShouldClose(viewer.window)) {
            double currentTime = glfwGetTime();
            float deltaTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;

            if (deltaTime > 0.1f)
                deltaTime = 0.1f;

            fpsAccum += deltaTime;
            frameCount++;
            if (fpsAccum >= 1.0f) {
                currentFps = frameCount / fpsAccum;
                fpsAccum = 0.0f;
                frameCount = 0;
            }

            glfwPollEvents();
            Game::Input input = inputManager.poll(viewer.window);

            if (input.f11Pressed) {
                rtgl::toggle_fullscreen(viewer);
            }

            if (input.escape) {
                glfwSetWindowShouldClose(viewer.window, 1);
                continue;
            }

            Game::update(gameState, scene, input, deltaTime);
            renderer.update();

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

            std::string title = Game::getWindowTitle(gameState, currentFps);
            glfwSetWindowTitle(viewer.window, title.c_str());
        }

        Game::onGameOver(gameState);
        renderer.cleanup();
        rtgl::destroy_interop_viewer(viewer);

        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }
}