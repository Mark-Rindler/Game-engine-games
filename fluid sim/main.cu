

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

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#endif

// STB Image
#define STB_IMAGE_IMPLEMENTATION

// Renderer Selection
#if RENDER_MODE == 0
#define UNIFIED_SCENE_ENABLE_PT
#elif RENDER_MODE == 1
#define UNIFIED_SCENE_ENABLE_RT
#endif

// Custom Includes
#define BLUE_NOISE_IMPLEMENTATION
#include "common/PTRTtransfer.cuh"
#include "common/bluenoise.cuh"

#if RENDER_MODE == 0
#include "pathtracer/app_utils.cuh"
#else
#include "raytracer/RTapp_utils.cuh"
#endif

// Game logic
#include "game.cuh"

// RENDERER ABSTRACTION
class Renderer {
  public:
    std::unique_ptr<Scene> scene;
    UnifiedScene *unifiedScene = nullptr;
    bool initialized = false;

    void init(UnifiedScene &uScene) {
        if (initialized) {
            cleanup();
        }

        unifiedScene = &uScene;

#if RENDER_MODE == 0
        scene = UnifiedSceneBuilder::buildPTScene(uScene);
        scene->setPerformancePreset(Game::Config::PT_PRESET);
#else
        scene = UnifiedSceneBuilder::buildRTScene(uScene);
#endif

        scene->uploadToGPU();
        initialized = true;
    }

    void update() {
        if (!scene || !unifiedScene)
            return;

        bool needsUpload = false;

#if RENDER_MODE == 0
        UnifiedSceneBuilder::updatePTCamera(*scene, *unifiedScene);
        if (unifiedScene->hasDirtyMeshes()) {
            UnifiedSceneBuilder::updatePTScene(*scene, *unifiedScene);
            needsUpload = true;
        }
#else
        UnifiedSceneBuilder::updateRTCamera(*scene, *unifiedScene);
        if (unifiedScene->hasDirtyMeshes()) {
            UnifiedSceneBuilder::updateRTScene(*scene, *unifiedScene);
            needsUpload = true;
        }
#endif

        if (needsUpload) {
            scene->uploadToGPU();
        }
    }

    void render(uint8_t *devicePixels) {
        scene->render_to_device(devicePixels);
    }

    void cleanup() {
        scene.reset();
        unifiedScene = nullptr;
        initialized = false;
    }

    ~Renderer() { cleanup(); }
};

// Double-Buffer Manager for Async Copies
class DoubleBufferManager {
  public:
    uint8_t *d_pixels[2] = {nullptr, nullptr}; // Device buffers
    uint8_t *h_pinned[2] = {nullptr, nullptr}; // Pinned host buffers
    cudaStream_t streams[2] = {nullptr, nullptr};
    cudaEvent_t renderComplete[2] = {nullptr, nullptr};
    cudaEvent_t copyComplete[2] = {nullptr, nullptr};

    size_t frameBytes = 0;
    int currentBuffer = 0;
    bool firstFrame = true;

    bool init(size_t bytes) {
        frameBytes = bytes;

        for (int i = 0; i < 2; ++i) {
            CUDA_CHECK(cudaMalloc((void **)&d_pixels[i], frameBytes));
            CUDA_CHECK(cudaMallocHost((void **)&h_pinned[i], frameBytes));
            CUDA_CHECK(cudaStreamCreate(&streams[i]));
            CUDA_CHECK(cudaEventCreate(&renderComplete[i]));
            CUDA_CHECK(cudaEventCreate(&copyComplete[i]));
        }

        return true;
    }

    void cleanup() {
        // Synchronize all streams before cleanup
        for (int i = 0; i < 2; ++i) {
            if (streams[i]) {
                cudaStreamSynchronize(streams[i]);
            }
        }

        for (int i = 0; i < 2; ++i) {
            if (d_pixels[i]) {
                cudaFree(d_pixels[i]);
                d_pixels[i] = nullptr;
            }
            if (h_pinned[i]) {
                cudaFreeHost(h_pinned[i]);
                h_pinned[i] = nullptr;
            }
            if (renderComplete[i]) {
                cudaEventDestroy(renderComplete[i]);
                renderComplete[i] = nullptr;
            }
            if (copyComplete[i]) {
                cudaEventDestroy(copyComplete[i]);
                copyComplete[i] = nullptr;
            }
            if (streams[i]) {
                cudaStreamDestroy(streams[i]);
                streams[i] = nullptr;
            }
        }
    }

    // Returns the device buffer to render into
    uint8_t *getDeviceBuffer() { return d_pixels[currentBuffer]; }

    // Start async copy after render completes
    void startAsyncCopy() {
        int buf = currentBuffer;

        // Record event when render is complete (on default stream)
        CUDA_CHECK(cudaEventRecord(renderComplete[buf], 0));

        // Make copy stream wait for render to complete
        CUDA_CHECK(cudaStreamWaitEvent(streams[buf], renderComplete[buf], 0));

        // Start async copy
        CUDA_CHECK(cudaMemcpyAsync(h_pinned[buf], d_pixels[buf], frameBytes,
                                   cudaMemcpyDeviceToHost, streams[buf]));

        // Record event when copy is complete
        CUDA_CHECK(cudaEventRecord(copyComplete[buf], streams[buf]));
    }

    // Wait for previous frame's copy to complete and return its host buffer
    // Returns nullptr if this is the first frame
    uint8_t *waitAndGetHostBuffer() {
        if (firstFrame) {
            firstFrame = false;
            return nullptr;
        }

        // Previous buffer index
        int prevBuf = 1 - currentBuffer;

        // Wait for the async copy to complete
        CUDA_CHECK(cudaEventSynchronize(copyComplete[prevBuf]));

        return h_pinned[prevBuf];
    }

    // Get the final frame's host buffer (for the last frame)
    uint8_t *waitForCurrentBuffer() {
        int buf = currentBuffer;
        CUDA_CHECK(cudaEventSynchronize(copyComplete[buf]));
        return h_pinned[buf];
    }

    void swapBuffers() { currentBuffer = 1 - currentBuffer; }

    ~DoubleBufferManager() { cleanup(); }
};

// Simple CLI parsing helpers
static bool arg_equals(const char *a, const char *b) {
    return std::strcmp(a, b) == 0;
}

static int arg_to_int(const char *s, int fallback) {
    if (!s)
        return fallback;
    char *end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (!end || end == s)
        return fallback;
    return (int)v;
}

struct OfflineConfig {
    int width = Game::Config::WINDOW_WIDTH;
    int height = Game::Config::WINDOW_HEIGHT;
    int fps = 60;
    int totalFrames = 600; // 10s @ 60fps
    std::string outputMp4 = "out.mp4";

    bool vflip = true;

    std::string pixFmt = "rgb24";

    int crf = 18;

    // x264 preset: ultrafast..veryslow (slower = smaller/better)
    std::string preset = "slow";

    // Use double buffering for async copies
    bool useDoubleBuffering = true;
};

static OfflineConfig parse_args(int argc, char **argv) {
    OfflineConfig cfg;
    for (int i = 1; i < argc; ++i) {
        if (arg_equals(argv[i], "-o") && i + 1 < argc) {
            cfg.outputMp4 = argv[++i];
        } else if (arg_equals(argv[i], "-w") && i + 1 < argc) {
            cfg.width = arg_to_int(argv[++i], cfg.width);
        } else if (arg_equals(argv[i], "-h") && i + 1 < argc) {
            cfg.height = arg_to_int(argv[++i], cfg.height);
        } else if (arg_equals(argv[i], "-fps") && i + 1 < argc) {
            cfg.fps = arg_to_int(argv[++i], cfg.fps);
        } else if (arg_equals(argv[i], "-frames") && i + 1 < argc) {
            cfg.totalFrames = arg_to_int(argv[++i], cfg.totalFrames);
        } else if (arg_equals(argv[i], "-vflip")) {
            cfg.vflip = true;
        } else if (arg_equals(argv[i], "-pix_fmt") && i + 1 < argc) {
            cfg.pixFmt = argv[++i];
        } else if (arg_equals(argv[i], "-crf") && i + 1 < argc) {
            cfg.crf = arg_to_int(argv[++i], cfg.crf);
        } else if (arg_equals(argv[i], "-preset") && i + 1 < argc) {
            cfg.preset = argv[++i];
        } else if (arg_equals(argv[i], "-no-double-buffer")) {
            cfg.useDoubleBuffering = false;
        } else if (arg_equals(argv[i], "--help") ||
                   arg_equals(argv[i], "-help")) {
            std::cout
                << "Offline MP4 render options:\n"
                << "  -o <file.mp4>      Output path (default: out.mp4)\n"
                << "  -w <width>         Width (default: "
                   "Config::WINDOW_WIDTH)\n"
                << "  -h <height>        Height (default: "
                   "Config::WINDOW_HEIGHT)\n"
                << "  -fps <fps>         Frames per second (default: 60)\n"
                << "  -frames <N>        Total frames (default: 600)\n"
                << "  -vflip             Vertically flip output in ffmpeg\n"
                << "  -pix_fmt <rgb24|bgr24>  Input pixel format for ffmpeg "
                   "(default: rgb24)\n"
                << "  -crf <int>         H.264 CRF quality (default: 18)\n"
                << "  -preset <n>        x264 preset (default: slow)\n"
                << "  -no-double-buffer  Disable async double-buffering\n";
            std::exit(0);
        }
    }
    // basic sanity
    cfg.width = std::max(1, cfg.width);
    cfg.height = std::max(1, cfg.height);
    cfg.fps = std::max(1, cfg.fps);
    cfg.totalFrames = std::max(1, cfg.totalFrames);
    cfg.crf = std::max(0, std::min(51, cfg.crf));
    return cfg;
}

// FFmpeg pipe helpers
static FILE *open_ffmpeg_pipe(const OfflineConfig &cfg) {
    char cmd[2048];
    const char *vf = cfg.vflip ? " -vf vflip " : " ";

#if defined(_WIN32)
    std::snprintf(cmd, sizeof(cmd),
                  "ffmpeg -y -f rawvideo -pix_fmt %s -s %dx%d -r %d -i -%s-an "
                  "-c:v libx264 -preset %s -crf %d -pix_fmt yuv420p \"%s\"",
                  cfg.pixFmt.c_str(), cfg.width, cfg.height, cfg.fps, vf,
                  cfg.preset.c_str(), cfg.crf, cfg.outputMp4.c_str());
    FILE *pipe = _popen(cmd, "wb");
    if (pipe) {
        _setmode(_fileno(pipe), _O_BINARY);
    }
    return pipe;
#else
    std::snprintf(cmd, sizeof(cmd),
                  "ffmpeg -y -f rawvideo -pix_fmt %s -s %dx%d -r %d -i -%s-an "
                  "-c:v libx264 -preset %s -crf %d -pix_fmt yuv420p \"%s\"",
                  cfg.pixFmt.c_str(), cfg.width, cfg.height, cfg.fps, vf,
                  cfg.preset.c_str(), cfg.crf, cfg.outputMp4.c_str());
    return popen(cmd, "w");
#endif
}

static int close_ffmpeg_pipe(FILE *pipe) {
    if (!pipe)
        return -1;
#if defined(_WIN32)
    return _pclose(pipe);
#else
    return pclose(pipe);
#endif
}

// Render loop with double buffering
static int render_with_double_buffering(const OfflineConfig &cfg,
                                        Game::State &gameState,
                                        UnifiedScene &scene, Renderer &renderer,
                                        FILE *ff) {
    const size_t frameBytes = (size_t)cfg.width * (size_t)cfg.height * 3ull;
    const float dt = 1.0f / (float)cfg.fps;
    Game::Input input{};

    DoubleBufferManager buffers;
    if (!buffers.init(frameBytes)) {
        std::cerr << "Failed to initialize double buffer manager\n";
        return 1;
    }

    std::cout << "Using double-buffered async copies\n";

    for (int frame = 0; frame < cfg.totalFrames; ++frame) {
        // Update game logic & scene
        Game::update(gameState, scene, input, dt);
        renderer.update();

        // Render to current device buffer
        renderer.render(buffers.getDeviceBuffer());
        CUDA_CHECK(cudaGetLastError());

        // Start async copy of current frame
        buffers.startAsyncCopy();

        // While GPU is copying current frame, write previous frame to ffmpeg
        uint8_t *prevHostBuffer = buffers.waitAndGetHostBuffer();
        if (prevHostBuffer) {
            size_t written = std::fwrite(prevHostBuffer, 1, frameBytes, ff);
            if (written != frameBytes) {
                std::cerr << "Error: fwrite wrote " << written
                          << " bytes (expected " << frameBytes << ").\n";
                std::cerr << "ffmpeg may have terminated early.\n";
                return 1;
            }
        }

        // Swap buffers for next frame (avoid swapping after the last frame)
        if (frame != cfg.totalFrames - 1) {
            buffers.swapBuffers();
        }

        if ((frame % 30) == 0) {
            std::cout << "Rendered frame " << frame << " / " << cfg.totalFrames
                      << "\n";
        }
    }

    // Write the final frame
    uint8_t *finalBuffer = buffers.waitForCurrentBuffer();
    size_t written = std::fwrite(finalBuffer, 1, frameBytes, ff);
    if (written != frameBytes) {
        std::cerr << "Warning: final frame write incomplete\n";
    }

    return 0;
}

// Render loop without double buffering (original synchronous method)
static int render_synchronous(const OfflineConfig &cfg, Game::State &gameState,
                              UnifiedScene &scene, Renderer &renderer,
                              FILE *ff) {
    const size_t frameBytes = (size_t)cfg.width * (size_t)cfg.height * 3ull;
    const float dt = 1.0f / (float)cfg.fps;
    Game::Input input{};

    uint8_t *d_pixels = nullptr;
    uint8_t *h_pinned = nullptr;

    CUDA_CHECK(cudaMalloc((void **)&d_pixels, frameBytes));
    CUDA_CHECK(cudaMallocHost((void **)&h_pinned, frameBytes));

    std::cout << "Using synchronous copies (no double-buffering)\n";

    int result = 0;
    for (int frame = 0; frame < cfg.totalFrames; ++frame) {
        Game::update(gameState, scene, input, dt);
        renderer.update();

        renderer.render(d_pixels);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaDeviceSynchronize());

        CUDA_CHECK(
            cudaMemcpy(h_pinned, d_pixels, frameBytes, cudaMemcpyDeviceToHost));

        size_t written = std::fwrite(h_pinned, 1, frameBytes, ff);
        if (written != frameBytes) {
            std::cerr << "Error: fwrite wrote " << written
                      << " bytes (expected " << frameBytes << ").\n";
            result = 1;
            break;
        }

        if ((frame % 30) == 0) {
            std::cout << "Rendered frame " << frame << " / " << cfg.totalFrames
                      << "\n";
        }
    }

    CUDA_CHECK(cudaFreeHost(h_pinned));
    CUDA_CHECK(cudaFree(d_pixels));

    return result;
}

// MAIN ENTRY POINT (OFFLINE)
int main(int argc, char *argv[]) {
    try {
        OfflineConfig cfg = parse_args(argc, argv);

#if RENDER_MODE == 0
        std::cout << "Renderer: Path Tracing\n";
#else
        std::cout << "Renderer: Ray Tracing\n";
#endif
        std::cout << "Offline encode -> " << cfg.outputMp4 << "\n";
        std::cout << "Resolution: " << cfg.width << "x" << cfg.height << " @ "
                  << cfg.fps << " fps\n";
        std::cout << "Frames: " << cfg.totalFrames << " (duration ~ "
                  << (double)cfg.totalFrames / (double)cfg.fps << "s)\n";
        std::cout << "FFmpeg pix_fmt (input): " << cfg.pixFmt
                  << (cfg.vflip ? " (vflip ON)\n" : "\n");

        // Initialize Game State & Scene
        Game::State gameState;
        UnifiedScene scene(cfg.width, cfg.height);
        scene.setBVHParams(1, 1);

        Game::start(gameState, scene);

        // Initialize Renderer
        Renderer renderer;
        renderer.init(scene);

        initBlueNoise();

        // Open ffmpeg pipe
        FILE *ff = open_ffmpeg_pipe(cfg);
        if (!ff) {
            std::cerr << "Failed to start ffmpeg. Make sure ffmpeg is "
                         "installed and on PATH.\n";
            renderer.cleanup();
            return 1;
        }

        // Run render loop
        int result;
        if (cfg.useDoubleBuffering) {
            result = render_with_double_buffering(cfg, gameState, scene,
                                                  renderer, ff);
        } else {
            result = render_synchronous(cfg, gameState, scene, renderer, ff);
        }

        // Close ffmpeg (finalizes mp4)
        int ffStatus = close_ffmpeg_pipe(ff);
        if (ffStatus != 0) {
            std::cerr << "Warning: ffmpeg exited with status " << ffStatus
                      << "\n";
        }

        // Cleanup
        Game::onGameOver(gameState);
        renderer.cleanup();

        if (result == 0) {
            std::cout << "Done: wrote " << cfg.outputMp4 << "\n";
        }
        return result;

    } catch (const std::exception &e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }
}