#ifndef GAME_CUH
#define GAME_CUH

#include "common/PTRTtransfer.cuh"
#include "common/vec3.cuh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace Game {

// CONFIG
struct Config {
    static constexpr int WINDOW_WIDTH = 1920;
    static constexpr int WINDOW_HEIGHT = 1080;
    static constexpr const char *WINDOW_TITLE =
        "Ocean Planet + Buoyancy Physics";

    // Planet parameters
    static constexpr float PLANET_RADIUS = 150.0f;
    // Higher tessellation so ship displacement + wake are visible.
    static constexpr int PLANET_SUBDIV_THETA = 256;
    static constexpr int PLANET_SUBDIV_PHI = 128;

    // Seafloor depth
    static constexpr float SEAFLOOR_DEPTH = 16.0f;

    // Mean water depth for finite depth dispersion
    static constexpr float MEAN_WATER_DEPTH = 50.0f;

    // Physics constants
    static constexpr float G = 9.81f;
    static constexpr float WATER_DENSITY = 1025.0f;
    static constexpr float AIR_DENSITY = 1.225f;
    static constexpr float KINEMATIC_VISCOSITY = 1.0e-6f;

    // Ocean spectrum parameters
    static constexpr float WIND_SPEED = 22.0f;
    static constexpr float WIND_DIR_DEG = 25.0f;
    static constexpr float FETCH = 80000.0f;
    static constexpr int NUM_WAVE_COMPONENTS = 128;
    static constexpr float MIN_WAVELENGTH = 0.15f;
    static constexpr float MAX_WAVELENGTH = 200.0f;
    static constexpr float SPECTRUM_SCALE = 0.9f;
    static constexpr float DIRECTIONAL_SPREAD = 3.5f;

    // Swell parameters
    static constexpr bool ENABLE_SWELL = true;
    static constexpr float SWELL_DIR_DEG = -15.0f;
    static constexpr float SWELL_HEIGHT = 3.5f;
    static constexpr float SWELL_PERIOD = 12.0f;

    // Ship physical properties
    static constexpr float SHIP_MASS = 5200.0f;
    static constexpr float SHIP_LENGTH = 8.0f;
    static constexpr float SHIP_BEAM = 3.2f;
    static constexpr float SHIP_DRAFT = 1.2f;
    static constexpr float SHIP_HEIGHT = 4.0f;
    static constexpr float SHIP_FREEBOARD = 1.5f;
    static constexpr float SHIP_SCALE = 0.9f;

    // Ship model file (single mesh)
    static constexpr const char *SHIP_OBJ = "models/ship_pinnace_2k.obj";

    // If TRUE, we will compute shipModelYOffset from ship OBJ bounds so the
    // keel sits correctly. If parsing fails, we fall back to
    // SHIP_MODEL_Y_OFFSET.
    static constexpr bool SHIP_AUTO_COMPUTE_MODEL_Y_OFFSET = false;
    // Hull form coefficients
    static constexpr float BLOCK_COEFF = 0.45f;
    static constexpr float WATERPLANE_COEFF = 0.72f;
    static constexpr float PRISMATIC_COEFF = 0.62f;
    static constexpr float MIDSHIP_COEFF = 0.78f;

    static constexpr float BUOYANCY_VOLUME_SCALE = 1.2f;
    static constexpr float WAVE_STEEPNESS_SCALE = 1.6f;

    // Ship initial drop height
    static constexpr float SHIP_DROP_HEIGHT = 5.0f;
    static constexpr float SHIP_MODEL_Y_OFFSET = 10.5f;

    static constexpr float SHIP_HEAVE_K = 60000.0f;
    static constexpr float SHIP_HEAVE_DAMP = 32000.0f;
    static constexpr float SHIP_ROT_SMOOTH = 10.0f;
    static constexpr float SHIP_MAX_HEAVE_ACCEL = 25.0f;
    static constexpr float SHIP_MAX_HEAVE_VEL = 8.0f;

    static constexpr float SHIP_UPWARD_K_SCALE = 0.35f;
    static constexpr float SHIP_WAVEHEIGHT_SMOOTH = 2.0f;

    static constexpr float SHIP_WATER_CONTACT_EPS = 0.06f;

    static constexpr float SHIP_CONTACT_SMOOTH = 4.0f;
    static constexpr float SHIP_INWATER_GRACE = 0.25f;

    // Ship-water visual interaction
    static constexpr float SHIP_PART_DEPTH = 1.50f;
    static constexpr float SHIP_PART_PUSH = 0.45f;
    static constexpr float SHIP_PART_RADIUS_SCALE_W = 1.18f;
    static constexpr float SHIP_PART_RADIUS_SCALE_L = 1.35f;

    // Bow wave parameters
    static constexpr float SHIP_BOW_WAVE_HEIGHT = 1.8f;
    static constexpr float SHIP_BOW_WAVE_LENGTH = 5.0f;

    // Kelvin wake parameters
    static constexpr float KELVIN_ANGLE = 19.47f * PI / 180.0f; // ~19.5 degrees
    static constexpr float WAKE_AMPLITUDE = 1.15f;
    static constexpr float WAKE_WAVELENGTH = 2.0f;
    static constexpr float WAKE_DECAY_RATE = 0.025f;
    static constexpr float WAKE_MAX_LENGTH = 60.0f;
    static constexpr float TRANSVERSE_WAKE_AMP = 0.5f;
    static constexpr float DIVERGENT_WAKE_AMP = 0.7f;

    // Ship attitude: how strongly/quickly we tilt to match the wave slope
    static constexpr float WAVE_FOLLOW_STRENGTH = 1.15f;
    static constexpr float WAVE_FOLLOW_RATE = 6.0f;
    static constexpr float WAVE_FOLLOW_MAX_ANGLE =
        0.35f; // radians (about 17 deg)

    // Attitude spring
    static constexpr float ATTITUDE_KP = 18.0f;
    static constexpr float ATTITUDE_KD = 10.0f;

    // Buoyancy sampling (higher resolution for accurate physics)
    static constexpr int BUOY_SAMPLES_X = 9;
    static constexpr int BUOY_SAMPLES_Z = 17;

    // Virtual sailing speed
    static constexpr float SAIL_SPEED = 8.0f;
    static constexpr float SAIL_SPEED_RESPONSE =
        2.5f; // 1/s (higher = snappier acceleration to target)
    static constexpr float SAIL_TURN_RATE = 0.4f;

    // Enhanced fluid dynamics parameters
    static constexpr float INERTIA_COEFFICIENT =
        1.2f;                                       // Cm for Morison equation
    static constexpr float DRAG_COEFFICIENT = 0.8f; // Cd for form drag
    static constexpr float WAVE_DRIFT_COEFFICIENT =
        0.015f; // Second order drift
    static constexpr float SLAMMING_THRESHOLD =
        0.5f; // m / s entry velocity for slamming
    static constexpr float BILGE_KEEL_EFFECT =
        1.0f; // 0-2, higher = more roll damping
    static constexpr float SURGE_SWAY_DAMPING =
        0.1f;                                  // Horizontal motion damping
    static constexpr float YAW_DAMPING = 2.0f; // Heading change damping

    // Orbit camera
    static constexpr float ORBIT_RADIUS = 256.0f;
    static constexpr float ORBIT_HEIGHT = -170.0f;
    static constexpr float ORBIT_SPEED = 0.08f;
    static constexpr float ORBIT_MOUSE_SENS = 0.004f;
    static constexpr float ORBIT_PITCH_MIN = -0.1f;
    static constexpr float ORBIT_PITCH_MAX = 0.8f;

    // Island parameters
    static constexpr float ISLAND_THETA = 0.85f;
    static constexpr float ISLAND_PHI = PI * 0.5f;
    static constexpr float ISLAND_RADIUS = PLANET_RADIUS * 0.55f;
    static constexpr float ISLAND_HEIGHT = 4.5f;

    static constexpr int NUM_EXTRA_ISLANDS = 32;
    static constexpr unsigned int ISLAND_SEED = 1337u;
    static constexpr float EXTRA_ISLAND_RADIUS_MIN = PLANET_RADIUS * 0.12f;
    static constexpr float EXTRA_ISLAND_RADIUS_MAX = PLANET_RADIUS * 0.28f;
    static constexpr float EXTRA_ISLAND_HEIGHT_MIN = 1.5f;
    static constexpr float EXTRA_ISLAND_HEIGHT_MAX = 5.0f;
    static constexpr float SHIP_TRACK_CLEARANCE = 0.35f;

    // Ship circumnavigation
    static constexpr float CIRCUMNAV_PERIOD =
        25.0f; // Slower for bigger planet feel
    static constexpr bool AUTO_CIRCUMNAVIGATE = true;
    static constexpr float ORBIT_YAW_OFFSET = 0.0f;

    static constexpr const char *PT_PRESET = "quality";
};

// INPUT
struct Input {
    bool w = false, a = false, s = false, d = false;
    bool shift = false;
    bool space = false;
    bool e = false, r = false;
    bool escape = false;
    bool jumpPressed = false;
    bool ePressed = false;
    bool rPressed = false;
    float mouseDX = 0.0f;
    float mouseDY = 0.0f;
    bool mouseCaptured = true;
};

// MATH HELPERS
inline float clampf(float x, float a, float b) {
    return std::max(a, std::min(b, x));
}

inline int clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

inline float wrapAnglePi(float a) {
    // Wrap to (-pi, pi]
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f)
        a += TWO_PI;
    return a - PI;
}

inline float wrapSigned(float x, float period) {
    // Wrap to [-period/2, period/2)
    if (period <= 0.0f)
        return x;
    x = std::fmod(x, period);
    const float half = 0.5f * period;
    if (x >= half)
        x -= period;
    if (x < -half)
        x += period;
    return x;
}

inline float smoothstep(float edge0, float edge1, float x) {
    const float denom = (edge1 - edge0);
    if (std::abs(denom) < 1e-6f) {
        return (x < edge0) ? 0.0f : 1.0f;
    }
    float t = clampf((x - edge0) / denom, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

inline float smoothstep01(float x) {
    x = clampf(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

inline float dot3(const vec3 &a, const vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline vec3 sphericalToCartesian(float theta, float phi, float r) {
    float sinPhi = std::sin(phi);
    float cosPhi = std::cos(phi);
    float sinTheta = std::sin(theta);
    float cosTheta = std::cos(theta);
    return vec3(r * sinPhi * cosTheta, r * cosPhi, r * sinPhi * sinTheta);
}

inline vec3 rotateY(const vec3 &p, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return vec3(p.x * c + p.z * s, p.y, -p.x * s + p.z * c);
}

inline vec3 rotateX(const vec3 &p, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return vec3(p.x, p.y * c - p.z * s, p.y * s + p.z * c);
}

inline vec3 rotateZ(const vec3 &p, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return vec3(p.x * c - p.y * s, p.x * s + p.y * c, p.z);
}

// Rotate a local-space offset by the same Euler convention we use for
// UnifiedTransform. Our rotation vector is (pitch=x, yaw=y, roll=z) in radians.
inline vec3 rotateEulerXYZ(const vec3 &p, const vec3 &rot) {
    vec3 r = p;
    r = rotateX(r, rot.x);
    r = rotateY(r, rot.y);
    r = rotateZ(r, rot.z);
    return r;
}

// WAVE COMPONENT STRUCTURE
struct WaveComponent {
    vec3 dir;
    float amplitude;
    float wavelength;
    float k;
    float omega;
    float phase;
    float steepness;
    float groupVel;
    float energy;
    float frequency;
};

// DISPERSION RELATION
inline float dispersionOmega(float k, float depth) {
    const float g = Config::G;
    const float kd = k * depth;

    float tanh_kd;
    if (kd > 15.0f) {
        tanh_kd = 1.0f;
    } else if (kd < 0.01f) {
        tanh_kd = kd;
    } else {
        tanh_kd = std::tanh(kd);
    }

    return std::sqrt(g * k * tanh_kd);
}

inline float dispersionWavenumber(float omega, float depth, int maxIter = 25) {
    // Solve: omega^2 = g * k * tanh(k * depth)
    // Uses Newton-Raphson iteration with robust fallbacks.

    const float g = Config::G;
    const float omega_sq = omega * omega;

    // Deep-water initial guess: k = omega^2 / g
    float k = std::max(1e-6f, omega_sq / std::max(1e-6f, g));

    // For very shallow water, use shallow-water approximation as initial guess
    if (depth < 5.0f && depth > 0.01f) {
        const float k_shallow = omega / std::sqrt(g * depth + 1e-6f);
        // Blend between deep and shallow guesses
        const float blend = std::min(1.0f, depth / 5.0f);
        k = k * blend + k_shallow * (1.0f - blend);
    }

    bool converged = false;
    float lastDk = 1e10f;

    for (int i = 0; i < maxIter; ++i) {
        const float kd = k * depth;

        float tanh_kd, sech2_kd;
        if (kd > 15.0f) {
            // Deep water limit: tanh(kd) -> 1, sech^2(kd) -> 0
            tanh_kd = 1.0f;
            sech2_kd = 0.0f;
        } else if (kd < 1e-4f) {
            // Shallow water limit: tanh(kd) -> kd, sech^2(kd) -> 1
            tanh_kd = kd;
            sech2_kd = 1.0f;
        } else {
            tanh_kd = std::tanh(kd);
            const float c = std::cosh(kd);
            sech2_kd = 1.0f / (c * c);
        }

        // f(k) = g * k * tanh(k*d) - omega^2 = 0
        const float f = g * k * tanh_kd - omega_sq;

        // f'(k) = g * tanh(k*d) + g * k * d * sech^2(k*d)
        const float df = g * tanh_kd + g * k * depth * sech2_kd;

        if (std::abs(df) < 1e-12f) {
            if (f > 0) {
                k *= 0.9f; // f > 0 means k is too large
            } else {
                k *= 1.1f; // f < 0 means k is too small
            }
            continue;
        }

        const float dk = f / df;

        // Check for convergence
        const float relChange = std::abs(dk) / std::max(1e-6f, k);
        if (relChange < 1e-7f) {
            converged = true;
            break;
        }

        if (std::abs(dk) > lastDk * 1.5f && i > 3) {
            // Iteration is diverging  use damped step
            k -= dk * 0.5f;
        } else {
            k -= dk;
        }

        lastDk = std::abs(dk);
        k = std::max(1e-6f, k);
    }

    // If Newton's method didn't converge, fall back to deep-water
    // approx
    if (!converged) {
        // Deep water: omega^2 = g * k, so k = omega^2 / g
        const float k_deep = omega_sq / g;
        // Use deep-water result but clamp to reasonable range
        k = std::max(1e-6f, std::min(k_deep, 100.0f));
    }

    return std::max(1e-6f, k);
}

inline float groupVelocity(float omega, float k, float depth) {
    const float kd = k * depth;
    const float c_phase = omega / k;

    float n;
    if (kd > 15.0f) {
        n = 0.5f;
    } else if (kd < 0.01f) {
        n = 1.0f;
    } else {
        const float sinh_2kd = std::sinh(2.0f * kd);
        n = 0.5f * (1.0f + 2.0f * kd / sinh_2kd);
    }

    return c_phase * n;
}

// OCEAN SPECTRA
inline float jonswapSpectrum(float omega, float windSpeed, float fetch) {

    const double g = static_cast<double>(Config::G);
    const double omega_d = static_cast<double>(omega);
    const double windSpeed_d = static_cast<double>(windSpeed);
    const double fetch_d = static_cast<double>(fetch);

    // Peak frequency
    const double omegaP =
        22.0 * std::pow(g * g / (windSpeed_d * fetch_d), 1.0 / 3.0);

    // Phillips constant
    const double alpha =
        0.076 * std::pow(windSpeed_d * windSpeed_d / (fetch_d * g), 0.22);

    // JONSWAP parameters
    const double gamma = 3.3;
    const double sigma = (omega_d <= omegaP) ? 0.07 : 0.09;

    // Guard against very small omega to prevent omega^5 underflow
    const double omega_safe = std::max(omega_d, 1e-10);

    // Compute omega^5 safely
    const double omega5 =
        omega_safe * omega_safe * omega_safe * omega_safe * omega_safe;

    // Guard against division by zero or very small numbers
    if (omega5 < 1e-50) {
        return 0.0f; // Effectively zero contribution
    }

    // Pierson-Moskowitz spectrum base
    const double exp_arg = -1.25 * std::pow(omegaP / omega_safe, 4.0);

    // Clamp exponential argument to prevent underflow
    const double exp_arg_clamped = std::max(exp_arg, -700.0); // exp(-700) ≈ 0

    const double pm = (alpha * g * g / omega5) * std::exp(exp_arg_clamped);

    // JONSWAP peak enhancement factor
    const double r_arg = -std::pow(omega_d - omegaP, 2.0) /
                         (2.0 * sigma * sigma * omegaP * omegaP + 1e-20);
    const double r = std::exp(std::max(r_arg, -700.0));

    const double result = pm * std::pow(gamma, r);

    // Final safety check and cast to float
    if (!std::isfinite(result) || result < 0.0) {
        return 0.0f;
    }

    return static_cast<float>(result);
}

inline float tmaSpectrum(float omega, float windSpeed, float fetch,
                         float depth) {
    float S_jonswap = jonswapSpectrum(omega, windSpeed, fetch);

    const float g = Config::G;
    const float omega_h = omega * std::sqrt(depth / g);

    float phi;
    if (omega_h <= 1.0f) {
        phi = 0.5f * omega_h * omega_h;
    } else if (omega_h < 2.0f) {
        phi = 1.0f - 0.5f * (2.0f - omega_h) * (2.0f - omega_h);
    } else {
        phi = 1.0f;
    }

    return S_jonswap * phi;
}

inline float directionalSpreading(float theta, float thetaMain,
                                  float spreadExp) {
    float diff = theta - thetaMain;
    while (diff > PI)
        diff -= TWO_PI;
    while (diff < -PI)
        diff += TWO_PI;
    if (std::abs(diff) > PI * 0.5f)
        return 0.0f;
    return std::pow(std::cos(diff), spreadExp);
}

// HYDRODYNAMIC COEFFICIENTS (Enhanced 6-DOF)
struct HydroCoefficients {
    // Added mass coefficients (Aij)
    float A11, A22, A33; // Surge, sway, heave added mass
    float A44, A55, A66; // Roll, pitch, yaw added mass
    float A24, A35;      // Coupled added mass (sway-roll, heave-pitch)

    // Damping coefficients (Bij)
    float B11, B22, B33; // Surge, sway, heave damping
    float B44, B55, B66; // Roll, pitch, yaw damping
    float B44_visc;      // Viscous roll damping
    float B24, B35;      // Coupled damping

    // Restoring coefficients (Cij)
    float C33, C44, C55; // Heave, roll, pitch restoring
    float C35;           // Coupled heave-pitch restoring

    float Awp;            // Waterplane area
    float nabla;          // Displaced volume
    float KB, KG;         // Keel-to-buoyancy, keel-to-gravity heights
    float BM_T, BM_L;     // Transverse and longitudinal metacentric radii
    float GM_T, GM_L;     // Transverse and longitudinal metacentric heights
    float I_WP_T, I_WP_L; // Waterplane moments of inertia
    float I44, I55, I66;  // Mass moments of inertia
    float omega_char;     // Characteristic frequency

    // Wetted surface
    float wettedSurface;

    // Hull geometry
    float hullVolume;
    float maxSubmergedVolume;
    float deadriseAngle; // For slamming calculations

    // Natural frequencies
    float omega_heave;
    float omega_roll;
    float omega_pitch;
};

inline HydroCoefficients computeHydroCoefficients() {
    HydroCoefficients H = {};

    const float L = Config::SHIP_LENGTH * Config::SHIP_SCALE;
    const float B = Config::SHIP_BEAM * Config::SHIP_SCALE;
    const float T = Config::SHIP_DRAFT * Config::SHIP_SCALE;
    const float mass = Config::SHIP_MASS;
    const float rho = Config::WATER_DENSITY;
    const float g = Config::G;

    const float C_B = Config::BLOCK_COEFF;
    const float C_WP = Config::WATERPLANE_COEFF;
    const float C_M = Config::MIDSHIP_COEFF;

    H.nabla = mass / rho;
    H.Awp = C_WP * L * B;

    // Total hull volume
    H.hullVolume = C_B * L * B * Config::SHIP_HEIGHT * Config::SHIP_SCALE;
    H.maxSubmergedVolume = C_B * L * B * T;

    // Wetted surface (Holtrop approximation)
    H.wettedSurface = L * (2.0f * T + B) * std::sqrt(C_M) *
                      (0.453f + 0.4425f * C_B - 0.2862f * C_M);
    H.wettedSurface = std::max(H.wettedSurface, L * B * 0.5f);

    // Deadrise angle (typical for displacement hull)
    H.deadriseAngle = 0.25f; // abt 14 degrees

    H.I_WP_T = (1.0f / 12.0f) * C_WP * L * B * B * B * 0.7f;
    H.I_WP_L = (1.0f / 12.0f) * C_WP * B * L * L * L * 0.5f;

    H.KB = T * (0.78f - 0.285f * C_B);

    const float freeboard = Config::SHIP_FREEBOARD * Config::SHIP_SCALE;
    H.KG = 0.52f * (T + freeboard);

    H.BM_T = H.I_WP_T / H.nabla;
    H.BM_L = H.I_WP_L / H.nabla;

    H.GM_T = H.KB + H.BM_T - H.KG;
    H.GM_L = H.KB + H.BM_L - H.KG;

    H.GM_T = std::max(0.15f, H.GM_T);
    H.GM_L = std::max(0.5f, H.GM_L);

    // Mass moments of inertia
    const float k44 = 0.38f * B; // Roll radius of gyration
    const float k55 = 0.25f * L; // Pitch radius of gyration
    const float k66 = 0.25f * L; // Yaw radius of gyration
    H.I44 = mass * k44 * k44;
    H.I55 = mass * k55 * k55;
    H.I66 = mass * k66 * k66;

    H.omega_char = std::sqrt(rho * g * H.Awp / (mass + rho * H.Awp * T * 0.8f));

    // Surge (using strip theory approximation)
    const float C_a_surge = 0.05f * C_B;
    H.A11 = rho * C_a_surge * H.nabla;

    // Sway (lateral added mass is significant)
    const float C_a_sway = 0.85f + 0.15f * (L / B);
    H.A22 = rho * (PI / 2.0f) * (T * T) * L * C_a_sway;

    // Heave
    const float C_a_heave = 0.85f * C_M;
    H.A33 = rho * (PI / 2.0f) * (B / 2.0f) * (B / 2.0f) * L * C_a_heave;

    // Roll
    const float C_a_roll = 0.25f;
    H.A44 = rho * k44 * k44 * H.nabla * C_a_roll;

    // Pitch
    const float C_a_pitch = 0.8f;
    H.A55 = rho * k55 * k55 * H.nabla * C_a_pitch;

    // Yaw
    const float C_a_yaw = 0.15f;
    H.A66 = rho * k66 * k66 * H.nabla * C_a_yaw;

    // Coupled terms
    H.A24 = 0.1f * H.A22 * k44;  // Sway-roll coupling
    H.A35 = 0.05f * H.A33 * k55; // Heave-pitch coupling

    const float omega = H.omega_char;

    // DAMPING COEFFICIENTS
    // Surge (mostly skin friction)
    const float C_b_surge = 0.01f;
    H.B11 = rho * omega * C_b_surge * H.wettedSurface;

    // Sway (lateral)
    const float C_b_sway = 0.08f;
    H.B22 = rho * omega * H.nabla * B * C_b_sway;

    // Heave
    const float C_b_heave = 0.15f;
    H.B33 = rho * omega * H.Awp * B * C_b_heave;

    // Roll
    const float C_b_roll = 0.02f;
    H.B44 = rho * omega * H.nabla * B * C_b_roll;

    // Pitch
    const float C_b_pitch = 0.08f;
    H.B55 = rho * omega * H.nabla * L * C_b_pitch;

    // Yaw
    const float C_b_yaw = 0.03f;
    H.B66 = rho * omega * H.nabla * L * C_b_yaw;

    // Viscous roll damping (bilge keels, vortex shedding)
    H.B44_visc = 0.25f * rho * L * T * T * B;

    // Coupled damping
    H.B24 = 0.05f * H.B22 * k44;
    H.B35 = 0.03f * H.B33 * k55;

    // RESTORING COEFFICIENTS
    H.C33 = rho * g * H.Awp;
    H.C44 = rho * g * H.nabla * H.GM_T;
    H.C55 = rho * g * H.nabla * H.GM_L;
    H.C35 = 0.0f; // Typically zero for symmetric hulls

    // Natural frequencies
    H.omega_heave = std::sqrt(H.C33 / (mass + H.A33));
    H.omega_roll = std::sqrt(H.C44 / (H.I44 + H.A44));
    H.omega_pitch = std::sqrt(H.C55 / (H.I55 + H.A55));

    return H;
}

// WAVE FORCES STRUCTURE (Enhanced for full 6-DOF)
struct WaveForces {
    // Linear forces (surge, sway, heave)
    float F1; // Surge force (forward/backward)
    float F2; // Sway force (left/right)
    float F3; // Heave force (up/down)

    // Moments (roll, pitch, yaw)
    float M4; // Roll moment
    float M5; // Pitch moment
    float M6; // Yaw moment

    // Wave kinematics at ship location
    vec3 velocity;     // Orbital velocity (m/s)
    vec3 acceleration; // Orbital acceleration (m/s^2)
    float elevation;   // Wave elevation (m)
    float slopeX;      // Wave slope in X direction
    float slopeZ;      // Wave slope in Z direction

    // Pressure field
    float dynamicPressure; // Dynamic pressure under ship

    // Second-order effects
    float driftForceX; // Mean wave drift force X
    float driftForceZ; // Mean wave drift force Z
};

// AIRY WAVE THEORY PARTICLE KINEMATICS
// Computes orbital velocities and accelerations at a point
// Only valid at or below wave surface, returns zero above water
inline void computeWaveKinematics(const std::vector<WaveComponent> &waves,
                                  float x, float z, float depth, float time,
                                  float verticalPos, vec3 &velocity,
                                  vec3 &acceleration, float &elevation,
                                  float &slopeX, float &slopeZ,
                                  float &dynamicPressure) {
    velocity = vec3(0.0f);
    acceleration = vec3(0.0f);
    elevation = 0.0f;
    slopeX = 0.0f;
    slopeZ = 0.0f;
    dynamicPressure = 0.0f;

    const float g = Config::G;
    const float rho = Config::WATER_DENSITY;

    // First pass: compute wave elevation only
    for (const auto &w : waves) {
        const float A = w.amplitude;
        const float k = w.k;
        const float dx = w.dir.x;
        const float dz = w.dir.z;

        const float phase = k * (dx * x + dz * z) - w.omega * time + w.phase;
        const float s = std::sin(phase);
        const float c = std::cos(phase);

        // Wave elevation (with Stokes second-order correction)
        elevation += A * s;
        elevation += 0.25f * A * A * k * (1.0f + c * c);

        // Slope (always compute for attitude)
        slopeX += A * k * dx * c;
        slopeZ += A * k * dz * c;
    }

    // If above water surface, wave kinematics are zero
    if (verticalPos > elevation + 0.1f) {
        return;
    }

    // Second pass: compute kinematics only when in/near water
    for (const auto &w : waves) {
        const float A = w.amplitude;
        const float k = w.k;
        const float omega = w.omega;
        const float dx = w.dir.x;
        const float dz = w.dir.z;

        const float phase = k * (dx * x + dz * z) - omega * time + w.phase;
        const float s = std::sin(phase);
        const float c = std::cos(phase);

        // Depth attenuation factors (Airy theory)
        // verticalPos is relative to MWL, negative means below water
        const float kd = k * depth;

        // Clamp vertical position to be at or below water surface
        const float zClamped = std::min(verticalPos, elevation);
        const float kz = k * (depth + zClamped);

        float coshFactor, sinhFactor;

        // Prevent overflow by clamping the exponential argument
        if (kd > 10.0f) {
            // Deep water approximation
            const float expArg = clampf(kz - kd, -20.0f, 20.0f);
            coshFactor = std::exp(expArg);
            sinhFactor = coshFactor;
        } else if (kd < 0.01f) {
            // Very shallow - linear approximation
            coshFactor = 1.0f;
            sinhFactor = clampf(kz / (depth + 1e-6f), -10.0f, 10.0f);
        } else {
            // Finite depth
            const float sinhKd = std::sinh(kd);
            const float coshKd = std::cosh(kd);
            const float kzClamped = clampf(kz, -15.0f, 15.0f);

            if (sinhKd > 1e-6f) {
                coshFactor = std::cosh(kzClamped) / sinhKd;
                sinhFactor = std::sinh(kzClamped) / sinhKd;
            } else {
                coshFactor = 1.0f;
                sinhFactor = kzClamped / (kd + 1e-6f);
            }

            // Clamp to reasonable values
            coshFactor = clampf(coshFactor, 0.0f, 100.0f);
            sinhFactor = clampf(sinhFactor, -100.0f, 100.0f);
        }

        // Horizontal velocity components
        const float uMag = A * omega * coshFactor;
        velocity.x += uMag * dx * c;
        velocity.z += uMag * dz * c;

        // Vertical velocity
        velocity.y += A * omega * sinhFactor * s;

        // Horizontal acceleration
        const float aMag = A * omega * omega * coshFactor;
        acceleration.x += aMag * dx * s;
        acceleration.z += aMag * dz * s;

        // Vertical acceleration
        acceleration.y += -A * omega * omega * sinhFactor * c;

        // Dynamic pressure (only when submerged)
        if (verticalPos <= elevation) {
            float pressureFactor;
            if (kd > 10.0f) {
                const float expArg = clampf(k * zClamped, -20.0f, 20.0f);
                pressureFactor = std::exp(expArg);
            } else {
                pressureFactor = std::cosh(clampf(kz, -15.0f, 15.0f)) /
                                 std::cosh(clampf(kd, 0.01f, 15.0f));
            }
            pressureFactor = clampf(pressureFactor, 0.0f, 100.0f);
            dynamicPressure += rho * g * A * pressureFactor * s;
        }
    }

    // Final safety clamp on outputs
    velocity.x = clampf(velocity.x, -50.0f, 50.0f);
    velocity.y = clampf(velocity.y, -50.0f, 50.0f);
    velocity.z = clampf(velocity.z, -50.0f, 50.0f);
    acceleration.x = clampf(acceleration.x, -100.0f, 100.0f);
    acceleration.y = clampf(acceleration.y, -100.0f, 100.0f);
    acceleration.z = clampf(acceleration.z, -100.0f, 100.0f);
}

// WAVE DRIFT FORCE (Second-order mean force)
inline void computeWaveDriftForce(const std::vector<WaveComponent> &waves,
                                  float shipLength, float shipBeam,
                                  float heading, float &driftX, float &driftZ) {
    driftX = 0.0f;
    driftZ = 0.0f;

    const float g = Config::G;
    const float rho = Config::WATER_DENSITY;
    const float Cd_drift = 0.015f;

    for (const auto &w : waves) {
        const float A = w.amplitude;
        const float k = w.k;

        const float waveAngle = std::atan2(w.dir.x, w.dir.z);
        const float relAngle = waveAngle - heading;

        const float Fd_mag = 0.5f * rho * g * k * A * A * shipBeam * Cd_drift;
        const float beamFactor = 1.0f + std::cos(2.0f * relAngle);

        driftX += Fd_mag * w.dir.x * beamFactor;
        driftZ += Fd_mag * w.dir.z * beamFactor;
    }
}

// VISCOUS DRAG FORCES
// Skin friction + form drag on hull
inline void computeViscousDrag(float shipLength, float shipBeam, float draft,
                               float wetArea, const vec3 &relVelocity,
                               float &dragX, float &dragY, float &dragZ) {
    const float rho = Config::WATER_DENSITY;
    const float nu = Config::KINEMATIC_VISCOSITY;

    // Reynolds number based on ship length
    const float U = std::sqrt(relVelocity.x * relVelocity.x +
                              relVelocity.z * relVelocity.z + 1e-6f);
    const float Re = U * shipLength / nu;

    // Friction coefficient (ITTC 1957 formula)
    const float logRe = std::log10(std::max(1e6f, Re));
    const float Cf = 0.075f / ((logRe - 2.0f) * (logRe - 2.0f));

    // Form factor (accounts for pressure drag)
    const float k_form = 0.1f * (shipBeam / shipLength) + 0.003f;
    const float Ct = Cf * (1.0f + k_form);

    // Skin friction drag
    const float Df = 0.5f * rho * U * U * wetArea * Ct;

    // Decompose into components
    if (U > 0.01f) {
        dragX = -Df * relVelocity.x / U;
        dragZ = -Df * relVelocity.z / U;
    } else {
        dragX = 0.0f;
        dragZ = 0.0f;
    }

    // Vertical drag (damping on heave motion)
    const float Cd_heave = 1.2f;
    const float Awp = Config::WATERPLANE_COEFF * shipLength * shipBeam;
    dragY =
        -0.5f * rho * Cd_heave * Awp * relVelocity.y * std::abs(relVelocity.y);
}

// ROLL DAMPING (Including bilge keel effects)
inline float computeRollDamping(float rollVel, float rollAngle,
                                float shipLength, float shipBeam, float draft,
                                float forwardSpeed, float waveFreq) {
    const float rho = Config::WATER_DENSITY;
    const float B = shipBeam;
    const float L = shipLength;
    const float T = draft;

    // Wave damping component
    const float B_wave = 0.02f * rho * B * B * B * L * waveFreq;

    // Friction damping (skin friction on hull during roll)
    const float B_fric = 0.1f * rho * L * T * B * B * std::abs(rollVel);

    // Eddy damping (vortex shedding from bilge)
    const float B_eddy = 0.05f * rho * L * T * T * B * std::abs(rollVel);

    // Lift damping (forward speed dependent)
    const float B_lift = 0.015f * rho * L * T * B * forwardSpeed;

    // Bilge keel contribution (if present)
    const float bilgeKeelArea = 0.02f * L * T;
    const float B_bilge =
        0.5f * rho * bilgeKeelArea * B * rollVel * std::abs(rollVel);

    // Total damping moment
    return -(B_wave + B_fric + B_eddy + B_lift) * rollVel - B_bilge;
}

// FROUDE-KRYLOV FORCES
// Forces from undisturbed wave pressure field
inline void computeFroudeKrylovForces(const std::vector<WaveComponent> &waves,
                                      float x, float z, float time,
                                      float shipLength, float shipBeam,
                                      float draft, float heading, float &F1,
                                      float &F2, float &F3, float &M4,
                                      float &M5) {
    F1 = F2 = F3 = M4 = M5 = 0.0f;

    const float g = Config::G;
    const float rho = Config::WATER_DENSITY;
    const float depth = Config::MEAN_WATER_DEPTH;

    const float cosH = std::cos(heading);
    const float sinH = std::sin(heading);

    // Smith correction factor for finite wave amplitude
    auto smithCorrection = [](float ka) -> float {
        if (ka < 0.01f)
            return 1.0f;
        return std::sin(ka) / ka;
    };

    for (const auto &w : waves) {
        const float A = w.amplitude;
        const float k = w.k;
        const float omega = w.omega;

        const float phase =
            k * (w.dir.x * x + w.dir.z * z) - omega * time + w.phase;
        const float s = std::sin(phase);
        const float c = std::cos(phase);

        // Wave angle relative to ship
        const float mu = std::atan2(w.dir.x, w.dir.z) - heading;
        const float cosMu = std::cos(mu);
        const float sinMu = std::sin(mu);

        // Smith corrections for ship dimensions
        const float kL = k * shipLength * 0.5f;
        const float kB = k * shipBeam * 0.5f;
        const float alphaL = smithCorrection(kL * std::abs(cosMu));
        const float alphaB = smithCorrection(kB * std::abs(sinMu));

        // Depth attenuation
        const float kd = k * depth;
        const float kT = k * draft;
        float depthFactor;

        // Handle small kT to avoid division by zero
        // As kT -> 0: (1 - exp(-kT))/kT -> 1 and sinh(kT)/(kT*cosh(kd)) ->
        // 1/cosh(kd)
        if (kT < 0.01f) {
            depthFactor = 1.0f / std::cosh(clampf(kd, 0.0f, 15.0f));
        } else if (kd > 10.0f) {
            // Deep water
            const float expKT = std::exp(clampf(-kT, -20.0f, 0.0f));
            depthFactor = (1.0f - expKT) / kT;
        } else {
            // Finite depth
            const float sinhKT = std::sinh(clampf(kT, -15.0f, 15.0f));
            const float coshKD = std::cosh(clampf(kd, 0.0f, 15.0f));
            depthFactor = sinhKT / (kT * coshKD + 1e-6f);
        }

        // Clamp depth factor to reasonable range
        depthFactor = clampf(depthFactor, 0.0f, 2.0f);

        // Displaced volume
        const float nabla = Config::BLOCK_COEFF * shipLength * shipBeam * draft;

        // Heave force (F3)
        const float F3_amp =
            rho * g * k * A * nabla * alphaL * alphaB * depthFactor;
        F3 += F3_amp * s;

        // Surge force (F1)
        const float F1_amp =
            rho * g * k * A * nabla * alphaL * alphaB * cosMu * depthFactor;
        F1 += F1_amp * c;

        // Sway force (F2)
        const float F2_amp =
            rho * g * k * A * nabla * alphaL * alphaB * sinMu * depthFactor;
        F2 += F2_amp * c;

        // Roll moment (M4)
        const float r44 = 0.35f * shipBeam;
        M4 += F2_amp * r44 * c;

        // Pitch moment (M5)
        const float r55 = 0.25f * shipLength;
        M5 += -F1_amp * r55 * s;
    }
}

// SLAMMING FORCE (Wagner theory based)
inline float computeSlammingForce(float entryVelocity, float wettedLength,
                                  float shipBeam, float deadriseAngle) {
    if (entryVelocity >= 0.0f)
        return 0.0f;

    const float rho = Config::WATER_DENSITY;
    const float V = -entryVelocity;
    const float beta = deadriseAngle;

    // Wagner coefficient
    const float tanBeta = std::tan(beta + 0.001f);
    const float piOverBeta = PI / (2.0f * beta + 0.01f);
    const float Cw = piOverBeta * piOverBeta / (tanBeta * tanBeta);

    // Slamming coefficient
    const float Cs = std::min(10.0f, Cw);

    // Slamming force
    const float Fs = 0.5f * rho * Cs * shipBeam * wettedLength * V * V;

    return -Fs;
}

// ISLAND SPEC
struct IslandSpec {
    vec3 dirLocal;
    float aLand;
    float aShelf;
    float height;
};

// SHIP STATE - Full 6-DOF with complete dynamics
struct ShipState {
    // Heave (vertical motion)
    float y;      // Height of hull bottom above mean water level (m)
    float yVel;   // Vertical velocity (m/s)
    float yAccel; // Vertical acceleration (m/s^2)

    // Surge (forward/backward motion relative to heading)
    float surge;    // Surge displacement (m)
    float surgeVel; // Surge velocity (m/s)

    // Sway (lateral motion perpendicular to heading)
    float sway;    // Sway displacement (m)
    float swayVel; // Sway velocity (m/s)

    // Roll (rotation about longitudinal axis)
    float phi;      // Roll angle (rad)
    float phiVel;   // Roll angular velocity (rad/s)
    float phiAccel; // Roll angular acceleration (rad/s²)

    // Pitch (rotation about transverse axis)
    float theta;      // Pitch angle (rad)
    float thetaVel;   // Pitch angular velocity (rad/s)
    float thetaAccel; // Pitch angular acceleration (rad/s²)

    // Yaw (heading)
    float psi;    // Yaw angle (rad)
    float psiVel; // Yaw angular velocity (rad/s)

    // State flags
    bool inWater;         // contact with water (includes short "air" grace)
    float timeInWater;    // seconds accumulated while considered inWater
    bool hasTouchedWater; // true after first splash (ends initial drop)
    float airTime;        // seconds since fully out of water

    // Wave-following state (for smooth coupling)
    vec3 waveVelocity;     // Local wave orbital velocity
    vec3 waveAcceleration; // Local wave orbital acceleration
    float localWaveHeight; // Wave elevation at ship center
    float waveFollowBlend; // 0..1 smoothed water-contact factor for pitch/roll
                           // coupling

    // Energy tracking (for stability monitoring)
    float kineticEnergy;
    float potentialEnergy;
};

struct VirtualPosition {
    float x;
    float z;
    float heading;
    float speed;

    // Additional for wave-relative motion
    float relVelX; // Velocity relative to wave orbital motion
    float relVelZ;
};

// FORWARD DECLARATIONS
inline void sampleGerstnerSphere(const std::vector<WaveComponent> &waves,
                                 float theta, float phi, float baseRadius,
                                 float time, const VirtualPosition &virtPos,
                                 const std::vector<IslandSpec> &islands,
                                 vec3 &outPos, vec3 &outNormal);

// GAME STATE
struct State {
    float t = 0.0f;

    ObjectHandle ocean;
    ObjectHandle seafloor;
    ObjectHandle islandGrass; // Separate mesh for grassy island tops
    ObjectHandle shipHull;

    std::vector<vec3> planetBaseVerts;
    std::vector<int> planetIndices;
    std::vector<float> planetTheta;
    std::vector<float> planetPhi;

    // Cached local-space terrain vertices (built once after islands are
    // generated)
    std::vector<vec3> seafloorBaseVertsLocal;
    std::vector<vec3> grassBaseVertsLocal;
    std::vector<bool> grassMask;

    // Reusable scratch buffers to avoid per-frame allocations
    std::vector<vec3> tmpDisplacedVerts;
    std::vector<vec3> tmpTriangleVerts;
    std::vector<bool> tmpBool;

    std::vector<WaveComponent> waves;
    std::vector<IslandSpec> islands;

    ShipState shipState;
    VirtualPosition virtualPos;
    bool shipEngineOn = true;

    float planetRotationY = 0.0f;
    float planetRotationX = 0.0f;

    // Unwrapped rotations used to keep wave-phase continuous (avoid pops at
    // wrap boundaries)
    float planetRotationYAccum = 0.0f;
    float planetRotationXAccum = 0.0f;

    float orbitYaw = 0.0f;
    float orbitPitch = 0.25f;

    // Model origin offset from hull bottom (meters). Used for placing the mesh.
    float shipModelYOffset = Config::SHIP_MODEL_Y_OFFSET;

    HydroCoefficients hydro;

    float currentWaveHeight = 0.0f;
    float buoyancyForce = 0.0f;
    float submergedVolume = 0.0f;
    float submergedFraction = 0.0f;

    float Hs = 0.0f;
    float Tp = 0.0f;
    float Tz = 0.0f;

    vec3 smoothCameraTarget = vec3(0.0f);
    bool smoothCameraTargetInitialized = false;
};

// OCEAN SPECTRUM GENERATION
inline void generateOceanSpectrum(State &state, unsigned int seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> uniform01(0.0f, 1.0f);
    std::uniform_real_distribution<float> uniformAngle(-PI, PI);

    state.waves.clear();
    state.waves.reserve(Config::NUM_WAVE_COMPONENTS +
                        10); // Extra for swells and chop

    const float windDirRad = Config::WIND_DIR_DEG * PI / 180.0f;
    const float depth = Config::MEAN_WATER_DEPTH;

    const float minOmega =
        std::sqrt(Config::G * TWO_PI / Config::MAX_WAVELENGTH);
    const float maxOmega =
        std::sqrt(Config::G * TWO_PI / Config::MIN_WAVELENGTH);

    float m0 = 0.0f, m2 = 0.0f;
    float Smax = 0.0f;
    float omegaPeak = 0.0f;

    for (int i = 0; i < Config::NUM_WAVE_COMPONENTS; ++i) {
        float t = (float)i / (float)(Config::NUM_WAVE_COMPONENTS - 1);
        float omega = minOmega * std::pow(maxOmega / minOmega, t);

        float S = tmaSpectrum(omega, Config::WIND_SPEED, Config::FETCH, depth);

        if (S > Smax) {
            Smax = S;
            omegaPeak = omega;
        }

        float dOmega;
        if (i == 0) {
            float omega_next =
                minOmega * std::pow(maxOmega / minOmega,
                                    1.0f / (Config::NUM_WAVE_COMPONENTS - 1));
            dOmega = omega_next - omega;
        } else if (i == Config::NUM_WAVE_COMPONENTS - 1) {
            float omega_prev =
                minOmega *
                std::pow(maxOmega / minOmega,
                         (float)(i - 1) / (Config::NUM_WAVE_COMPONENTS - 1));
            dOmega = omega - omega_prev;
        } else {
            float omega_prev =
                minOmega *
                std::pow(maxOmega / minOmega,
                         (float)(i - 1) / (Config::NUM_WAVE_COMPONENTS - 1));
            float omega_next =
                minOmega *
                std::pow(maxOmega / minOmega,
                         (float)(i + 1) / (Config::NUM_WAVE_COMPONENTS - 1));
            dOmega = 0.5f * (omega_next - omega_prev);
        }

        float variance = S * dOmega;
        float amplitude = std::sqrt(2.0f * variance) * Config::SPECTRUM_SCALE;

        m0 += variance;
        m2 += omega * omega * variance;

        float theta = windDirRad + (uniform01(rng) - 0.5f) * PI * 0.7f;
        float spreading =
            directionalSpreading(theta, windDirRad, Config::DIRECTIONAL_SPREAD);
        amplitude *= std::sqrt(spreading);

        if (amplitude < 0.001f)
            continue;

        float k = dispersionWavenumber(omega, depth);
        float wavelength = TWO_PI / k;
        float cg = groupVelocity(omega, k, depth);

        float maxSteepness = 0.142f * std::tanh(k * depth);
        float steepness =
            clampf(amplitude * k * 0.5f, 0.0f, maxSteepness * 0.8f);
        steepness *= 1.0f / (1.0f + 0.1f * (float)i);
        steepness *= Config::WAVE_STEEPNESS_SCALE;
        steepness = std::min(steepness, maxSteepness * 0.95f);

        WaveComponent wave;
        wave.dir = vec3(std::sin(theta), 0.0f, std::cos(theta));
        wave.amplitude = amplitude;
        wave.wavelength = wavelength;
        wave.k = k;
        wave.omega = omega;
        wave.phase = uniformAngle(rng);
        wave.steepness = steepness;
        wave.groupVel = cg;
        wave.energy =
            0.5f * Config::WATER_DENSITY * Config::G * amplitude * amplitude;
        wave.frequency = omega / TWO_PI;

        state.waves.push_back(wave);
    }

    if (Config::ENABLE_SWELL) {
        const float swellDirRad = Config::SWELL_DIR_DEG * PI / 180.0f;
        const float swellOmega = TWO_PI / Config::SWELL_PERIOD;

        float k_swell = dispersionWavenumber(swellOmega, depth);
        float wavelength_swell = TWO_PI / k_swell;
        float cg_swell = groupVelocity(swellOmega, k_swell, depth);

        WaveComponent swell;
        swell.dir = vec3(std::sin(swellDirRad), 0.0f, std::cos(swellDirRad));
        swell.amplitude = Config::SWELL_HEIGHT * 0.5f;
        swell.wavelength = wavelength_swell;
        swell.k = k_swell;
        swell.omega = swellOmega;
        swell.phase = 0.0f;
        swell.steepness =
            clampf(swell.amplitude * k_swell * 0.25f, 0.0f, 0.35f);
        swell.groupVel = cg_swell;
        swell.energy = 0.5f * Config::WATER_DENSITY * Config::G *
                       swell.amplitude * swell.amplitude;
        swell.frequency = swellOmega / TWO_PI;
        state.waves.push_back(swell);

        WaveComponent swell2;
        const float omega2 = swellOmega * 1.08f;
        float k2 = dispersionWavenumber(omega2, depth);
        swell2.dir = vec3(std::sin(swellDirRad + 0.12f), 0.0f,
                          std::cos(swellDirRad + 0.12f));
        swell2.amplitude = Config::SWELL_HEIGHT * 0.3f;
        swell2.wavelength = TWO_PI / k2;
        swell2.k = k2;
        swell2.omega = omega2;
        swell2.phase = 1.5f;
        swell2.steepness = clampf(swell2.amplitude * k2 * 0.2f, 0.0f, 0.3f);
        swell2.groupVel = groupVelocity(omega2, k2, depth);
        swell2.energy = 0.5f * Config::WATER_DENSITY * Config::G *
                        swell2.amplitude * swell2.amplitude;
        swell2.frequency = omega2 / TWO_PI;
        state.waves.push_back(swell2);

        WaveComponent crossSwell1;
        const float crossDir1 = swellDirRad + PI * 0.5f; // 90 degrees offset
        const float crossOmega1 = TWO_PI / (Config::SWELL_PERIOD * 0.85f);
        float k_cross1 = dispersionWavenumber(crossOmega1, depth);
        crossSwell1.dir = vec3(std::sin(crossDir1), 0.0f, std::cos(crossDir1));
        crossSwell1.amplitude = Config::SWELL_HEIGHT * 0.35f;
        crossSwell1.wavelength = TWO_PI / k_cross1;
        crossSwell1.k = k_cross1;
        crossSwell1.omega = crossOmega1;
        crossSwell1.phase = 2.1f;
        crossSwell1.steepness =
            clampf(crossSwell1.amplitude * k_cross1 * 0.22f, 0.0f, 0.32f);
        crossSwell1.groupVel = groupVelocity(crossOmega1, k_cross1, depth);
        crossSwell1.energy = 0.5f * Config::WATER_DENSITY * Config::G *
                             crossSwell1.amplitude * crossSwell1.amplitude;
        crossSwell1.frequency = crossOmega1 / TWO_PI;
        state.waves.push_back(crossSwell1);

        // Cross-swell 2: opposite-ish direction for confused seas
        WaveComponent crossSwell2;
        const float crossDir2 = swellDirRad + PI * 0.7f; // 126 degrees offset
        const float crossOmega2 = TWO_PI / (Config::SWELL_PERIOD * 1.15f);
        float k_cross2 = dispersionWavenumber(crossOmega2, depth);
        crossSwell2.dir = vec3(std::sin(crossDir2), 0.0f, std::cos(crossDir2));
        crossSwell2.amplitude = Config::SWELL_HEIGHT * 0.25f;
        crossSwell2.wavelength = TWO_PI / k_cross2;
        crossSwell2.k = k_cross2;
        crossSwell2.omega = crossOmega2;
        crossSwell2.phase = 0.7f;
        crossSwell2.steepness =
            clampf(crossSwell2.amplitude * k_cross2 * 0.2f, 0.0f, 0.3f);
        crossSwell2.groupVel = groupVelocity(crossOmega2, k_cross2, depth);
        crossSwell2.energy = 0.5f * Config::WATER_DENSITY * Config::G *
                             crossSwell2.amplitude * crossSwell2.amplitude;
        crossSwell2.frequency = crossOmega2 / TWO_PI;
        state.waves.push_back(crossSwell2);

        // Short chop waves from yet another direction
        WaveComponent chop1;
        const float chopDir = swellDirRad - PI * 0.35f;
        const float chopOmega = TWO_PI / 4.5f; // Short period chop
        float k_chop = dispersionWavenumber(chopOmega, depth);
        chop1.dir = vec3(std::sin(chopDir), 0.0f, std::cos(chopDir));
        chop1.amplitude = 0.8f;
        chop1.wavelength = TWO_PI / k_chop;
        chop1.k = k_chop;
        chop1.omega = chopOmega;
        chop1.phase = 3.2f;
        chop1.steepness = clampf(chop1.amplitude * k_chop * 0.35f, 0.0f, 0.4f);
        chop1.groupVel = groupVelocity(chopOmega, k_chop, depth);
        chop1.energy = 0.5f * Config::WATER_DENSITY * Config::G *
                       chop1.amplitude * chop1.amplitude;
        chop1.frequency = chopOmega / TWO_PI;
        state.waves.push_back(chop1);
    }

    if (m0 > 0.0f) {
        state.Hs = 4.0f * std::sqrt(m0) * Config::SPECTRUM_SCALE;
        state.Tp = TWO_PI / omegaPeak;
        state.Tz = TWO_PI * std::sqrt(m0 / m2);
    }
}

// WAVE SAMPLING
inline float sampleWaveElevation(const std::vector<WaveComponent> &waves,
                                 float x, float z, float time) {
    float eta = 0.0f;
    for (const auto &w : waves) {
        const float phase =
            w.k * (w.dir.x * x + w.dir.z * z) - w.omega * time + w.phase;
        eta += w.amplitude * std::sin(phase);
    }
    return eta;
}

inline void sampleWaveHeightAndSlope(const std::vector<WaveComponent> &waves,
                                     float x, float z, float time,
                                     float &height, float &slopeX,
                                     float &slopeZ) {
    height = 0.0f;
    slopeX = 0.0f;
    slopeZ = 0.0f;

    for (const auto &w : waves) {
        const float A = w.amplitude;
        const float k = w.k;
        const float phase =
            k * (w.dir.x * x + w.dir.z * z) - w.omega * time + w.phase;
        const float s = std::sin(phase);
        const float c = std::cos(phase);

        height += A * s;
        height += 0.25f * A * A * k * (1.0f + c * c); // Stokes correction

        slopeX += A * k * w.dir.x * c;
        slopeZ += A * k * w.dir.z * c;
    }
}

// SIMPLE OBJ BOUNDS PARSER (for aligning split ship meshes)
// Reads only 'v x y z' lines and computes an AABB in OBJ local space.
inline bool computeObjBounds(const std::string &path, vec3 &outMin,
                             vec3 &outMax) {
    std::ifstream in(path);
    if (!in.is_open())
        return false;

    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();

    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 2)
            continue;
        // Vertex position
        if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream ss(line.substr(2));
            float x, y, z;
            ss >> x >> y >> z;
            if (!ss.fail()) {
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                minZ = std::min(minZ, z);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
                maxZ = std::max(maxZ, z);
            }
        }
    }

    if (!std::isfinite(minX) || !std::isfinite(maxX))
        return false;

    outMin = vec3(minX, minY, minZ);
    outMax = vec3(maxX, maxY, maxZ);
    return true;
}

inline vec3 aabbCenter(const vec3 &mn, const vec3 &mx) {
    return vec3(0.5f * (mn.x + mx.x), 0.5f * (mn.y + mx.y),
                0.5f * (mn.z + mx.z));
}

// KELVIN WAKE PATTERN
// Proper ship wake with transverse and divergent wave systems
inline float computeKelvinWake(float x, float z, float shipSpeed, float time) {
    if (z >= 0.0f || shipSpeed < 0.5f)
        return 0.0f; // Only behind ship, only when moving

    const float s = -z; // Distance behind ship
    if (s > Config::WAKE_MAX_LENGTH)
        return 0.0f;

    const float g = Config::G;
    const float Fr =
        shipSpeed / std::sqrt(g * Config::SHIP_LENGTH * Config::SHIP_SCALE);

    // Kelvin angle is 19.47 degrees
    const float kelvinAngle = Config::KELVIN_ANGLE;
    const float tanKelvin = std::tan(kelvinAngle);

    // Wake envelope
    const float wakeHalfWidth = s * tanKelvin;
    if (std::abs(x) > wakeHalfWidth * 1.5f)
        return 0.0f;

    // Decay with distance
    const float decay = std::exp(-s * Config::WAKE_DECAY_RATE);

    // Angle from centerline
    const float angleFromCenter = std::atan2(std::abs(x), s);

    float wakeHeight = 0.0f;

    // 1. Transverse waves (perpendicular to ship motion)
    // These travel at ship speed and have wavelength lambda = 2piU^2/g
    {
        const float lambda_t = TWO_PI * shipSpeed * shipSpeed / g;
        const float k_t = TWO_PI / std::max(0.1f, lambda_t);

        // Transverse waves are strongest near centerline
        const float lateralFalloff =
            std::exp(-(x * x) / (wakeHalfWidth * wakeHalfWidth * 0.3f + 0.1f));

        const float phase_t = k_t * s - shipSpeed * k_t * time * 0.5f;
        wakeHeight += Config::TRANSVERSE_WAKE_AMP * decay * lateralFalloff *
                      std::sin(phase_t);
    }

    // 2. Divergent waves (angled outward from ship track)
    // These form the V-shape of the Kelvin wake
    {
        // Divergent waves exist between 0 and Kelvin angle
        if (angleFromCenter < kelvinAngle && angleFromCenter > 0.05f) {
            const float divergentStrength =
                std::sin(PI * angleFromCenter / kelvinAngle);

            // Wavelength varies with angle
            const float cosAngle = std::cos(angleFromCenter);
            const float lambda_d =
                TWO_PI * shipSpeed * shipSpeed * cosAngle * cosAngle / g;
            const float k_d = TWO_PI / std::max(0.1f, lambda_d);

            // Distance along the ray from ship
            const float rayDist = std::sqrt(x * x + s * s);

            const float phase_d = k_d * rayDist - shipSpeed * k_d * time * 0.3f;

            // Sign based on which side of wake
            const float side = (x > 0.0f) ? 1.0f : -1.0f;

            wakeHeight += Config::DIVERGENT_WAKE_AMP * decay *
                          divergentStrength * std::sin(phase_d + side * 0.5f);
        }
    }

    // 3. Cusp waves (at the Kelvin angle boundary)
    {
        const float cuspWidth = 0.1f; // Angular width of cusp region
        const float cuspDist = std::abs(angleFromCenter - kelvinAngle);
        if (cuspDist < cuspWidth) {
            const float cuspStrength = 1.0f - cuspDist / cuspWidth;
            const float lambda_c = TWO_PI * shipSpeed * shipSpeed / g * 0.5f;
            const float k_c = TWO_PI / std::max(0.1f, lambda_c);

            const float rayDist = std::sqrt(x * x + s * s);
            const float phase_c = k_c * rayDist - shipSpeed * k_c * time * 0.4f;

            wakeHeight +=
                0.3f * decay * cuspStrength * cuspStrength * std::sin(phase_c);
        }
    }

    return wakeHeight * Fr * Config::WAKE_AMPLITUDE;
}

// TRUE BUOYANCY PHYSICS
// Ship falls from height, displaces water, floats naturally
struct BuoyancyResult {
    float force;           // Upward buoyancy force (N)
    float submergedVolume; // Volume of hull underwater (m³)
    float submergedFrac;   // Fraction of hull underwater (0-1)
    float torquePitch;     // Pitch restoring torque (N·m)
    float torqueRoll;      // Roll restoring torque (N·m)
    float centerX;         // Center of buoyancy offset X
    float centerZ;         // Center of buoyancy offset Z
    float avgWaveHeight;   // Average wave height under hull
};

inline BuoyancyResult computeTrueBuoyancy(const State &state, float time) {
    BuoyancyResult result = {};

    const float L = Config::SHIP_LENGTH * Config::SHIP_SCALE;
    const float B = Config::SHIP_BEAM * Config::SHIP_SCALE;
    const float T = Config::SHIP_DRAFT * Config::SHIP_SCALE;
    const float H = Config::SHIP_HEIGHT * Config::SHIP_SCALE;

    const float rho = Config::WATER_DENSITY;
    const float g = Config::G;

    const ShipState &ship = state.shipState;
    const VirtualPosition &vpos = state.virtualPos;

    const float cosH = std::cos(ship.psi);
    const float sinH = std::sin(ship.psi);
    const float cosPhi = std::cos(ship.phi);
    const float sinPhi = std::sin(ship.phi);
    const float cosTheta = std::cos(ship.theta);
    const float sinTheta = std::sin(ship.theta);

    const int nX = Config::BUOY_SAMPLES_X;
    const int nZ = Config::BUOY_SAMPLES_Z;
    const float halfL = L * 0.5f;
    const float halfB = B * 0.5f;

    float totalVolume = 0.0f;
    float momentX = 0.0f;
    float momentZ = 0.0f;
    float totalWaveH = 0.0f;
    int sampleCount = 0;

    // Sample hull sections
    for (int iz = 0; iz < nZ; ++iz) {
        float localZ = -halfL + L * (float)iz / (float)(nZ - 1);
        float zNorm = localZ / halfL;

        // Hull taper (narrower at bow/stern)
        float taperFactor = 1.0f - 0.4f * zNorm * zNorm;
        float localHalfB = halfB * taperFactor;

        for (int ix = 0; ix < nX; ++ix) {
            float localX =
                -localHalfB + 2.0f * localHalfB * (float)ix / (float)(nX - 1);

            // Transform to world coordinates (relative to virtual position)
            float worldX = localX * cosH - localZ * sinH;
            float worldZ = localX * sinH + localZ * cosH;

            // Sample wave height at this point
            float waveH = sampleWaveElevation(state.waves, vpos.x + worldX,
                                              vpos.z + worldZ, time);

            totalWaveH += waveH;
            sampleCount++;

            // Hull bottom height at this section, accounting for pitch and roll
            float hullBottomOffset = -localZ * sinTheta + localX * sinPhi;
            float hullBottom = ship.y + hullBottomOffset;

            // Submersion calculation
            float submersion = waveH - hullBottom;

            if (submersion > 0.0f) {
                // Clamp to hull height
                submersion = std::min(submersion, H);

                // Section area
                float sectionWidth = 2.0f * localHalfB / (nX - 1);
                float sectionLength = L / (nZ - 1);
                float sectionArea =
                    sectionWidth *
                    sectionLength; // taper already applied via localHalfB

                // (0.45). We already tapered the geometry geometrically above.
                float sectionVolume = sectionArea * submersion *
                                      Config::MIDSHIP_COEFF *
                                      Config::BUOYANCY_VOLUME_SCALE;

                totalVolume += sectionVolume;

                // Moments for center of buoyancy calculation
                momentX += sectionVolume * localX;
                momentZ += sectionVolume * localZ;
            }
        }
    }

    result.submergedVolume = totalVolume;
    result.submergedFrac =
        totalVolume / std::max(0.001f, state.hydro.hullVolume);
    result.submergedFrac = clampf(result.submergedFrac, 0.0f, 1.0f);

    // Buoyancy force = ρ x g x V_submerged
    result.force = rho * g * totalVolume;

    // Center of buoyancy relative to hull center
    if (totalVolume > 0.001f) {
        result.centerX = momentX / totalVolume;
        result.centerZ = momentZ / totalVolume;
    }

    // Restoring torques from off-center buoyancy
    result.torquePitch = -result.force * result.centerZ;
    result.torqueRoll = result.force * result.centerX;

    // Average wave height
    result.avgWaveHeight =
        (sampleCount > 0) ? (totalWaveH / sampleCount) : 0.0f;

    return result;
}

// SHIP PHYSICS WITH TRUE BUOYANCY
inline void updateShipPhysicsStep(State &state, const Input &input, float dt) {
    ShipState &ship = state.shipState;
    VirtualPosition &vpos = state.virtualPos;

    // 1) Simple sailing / rotate the planet under the ship
    const float R = Config::PLANET_RADIUS;

    // Steering + throttle (no "drop" / airborne gating; keep moving even if a
    // crest launches the hull)
    float turnInput = 0.0f;
    if (input.a)
        turnInput -= 1.0f;
    if (input.d)
        turnInput += 1.0f;

    float thrustInput = 0.0f;
    if (input.w)
        thrustInput += 1.0f;
    if (input.s)
        thrustInput -= 0.6f; // slower reverse

    if (!state.shipEngineOn)
        thrustInput = 0.0f;

    vpos.heading =
        wrapAnglePi(vpos.heading + turnInput * Config::SAIL_TURN_RATE * dt);

    const float targetSpeed = thrustInput * Config::SAIL_SPEED;
    const float speedBlend = 1.0f - std::exp(-Config::SAIL_SPEED_RESPONSE * dt);
    vpos.speed += (targetSpeed - vpos.speed) * speedBlend;

    // Integrate virtual planar motion (arc-length meters). The ship model
    // stays at the north pole
    const vec3 forward(std::sin(vpos.heading), 0.0f, std::cos(vpos.heading));
    vpos.x += forward.x * vpos.speed * dt;
    vpos.z += forward.z * vpos.speed * dt;

    // Convert unwrapped travel distance -> planet rotation (so the world moves
    // under the ship)
    state.planetRotationYAccum = -vpos.x / R;
    state.planetRotationXAccum = vpos.z / R;
    state.planetRotationY = wrapAnglePi(state.planetRotationYAccum);
    state.planetRotationX = wrapAnglePi(state.planetRotationXAccum);

    ship.psi = vpos.heading;
    ship.psiVel = 0.0f;

    // 2) Wave sampling
    const float cosH = std::cos(ship.psi);
    const float sinH = std::sin(ship.psi);

    const float B = Config::SHIP_BEAM * Config::SHIP_SCALE;
    const float L = Config::SHIP_LENGTH * Config::SHIP_SCALE;

    // Sample points: beam edges, and ~80% of half-length so we don't sit on the
    // extreme bow/stern.
    const float halfW = 0.50f * B;
    const float halfL = 0.40f * L;

    auto sampleH = [&](float localX, float localZ) -> float {
        const float worldX = localX * cosH - localZ * sinH;
        const float worldZ = localX * sinH + localZ * cosH;
        return sampleWaveElevation(state.waves, vpos.x + worldX,
                                   vpos.z + worldZ, state.t);
    };

    const float hC = sampleWaveElevation(state.waves, vpos.x, vpos.z, state.t);
    const float hFL = sampleH(-halfW, halfL);
    const float hFR = sampleH(halfW, halfL);
    const float hBL = sampleH(-halfW, -halfL);
    const float hBR = sampleH(halfW, -halfL);

    // Use center height as the ship's "waterline" reference.
    state.currentWaveHeight = hC;

    // Heave target keep the hull bottom about one draft below the surface
    const float T = Config::SHIP_DRAFT * Config::SHIP_SCALE;

    // Smooth the water height target so fast crests don't "kick" the hull into
    // the air
    const float aH = 1.0f - std::exp(-Config::SHIP_WAVEHEIGHT_SMOOTH * dt);
    ship.localWaveHeight += (hC - ship.localWaveHeight) * aH;

    const float desiredY = ship.localWaveHeight - T;

    // Springdamper heave
    // Make upward response softer than downward to keep the boat feeling
    // heavy
    const float yErr = desiredY - ship.y;
    const float k = (yErr > 0.0f)
                        ? (Config::SHIP_HEAVE_K * Config::SHIP_UPWARD_K_SCALE)
                        : Config::SHIP_HEAVE_K;

    float force = k * yErr - Config::SHIP_HEAVE_DAMP * ship.yVel;

    float acc = force / std::max(1e-3f, Config::SHIP_MASS);
    acc = clampf(acc, -Config::SHIP_MAX_HEAVE_ACCEL,
                 Config::SHIP_MAX_HEAVE_ACCEL);

    ship.yAccel = acc;
    ship.yVel += acc * dt;
    ship.yVel = clampf(ship.yVel, -Config::SHIP_MAX_HEAVE_VEL,
                       Config::SHIP_MAX_HEAVE_VEL);
    ship.y += ship.yVel * dt;

    // Contact depth at center (after heave integration); used for wave follow
    // gating and wake
    const float depthKeel = hC - ship.y; // >0 => hull bottom under surface
    const bool inWaterNow = (depthKeel > Config::SHIP_WATER_CONTACT_EPS);

    // Pitch/Roll from sampled heights (tangent-plane approximation).
    const float frontH = 0.5f * (hFL + hFR);
    const float backH = 0.5f * (hBL + hBR);
    const float leftH = 0.5f * (hFL + hBL);
    const float rightH = 0.5f * (hFR + hBR);

    float wavePitch =
        -std::atan2(frontH - backH, 2.0f * std::max(0.01f, halfL));
    float waveRoll = std::atan2(rightH - leftH, 2.0f * std::max(0.01f, halfW));

    const float maxA = Config::WAVE_FOLLOW_MAX_ANGLE;
    wavePitch = clampf(wavePitch, -maxA, maxA);
    waveRoll = clampf(waveRoll, -maxA, maxA);

    const float contactRaw =
        clampf((depthKeel - Config::SHIP_WATER_CONTACT_EPS) /
                   std::max(0.001f, 2.0f * T),
               0.0f, 1.0f);

    const float aContact = 1.0f - std::exp(-Config::SHIP_CONTACT_SMOOTH * dt);
    ship.waveFollowBlend += (contactRaw - ship.waveFollowBlend) * aContact;

    const float targetPitch = ship.waveFollowBlend * wavePitch;
    const float targetRoll = ship.waveFollowBlend * waveRoll;

    // Smooth attitude toward target (works both in and out of water).
    const float aSmooth = 1.0f - std::exp(-Config::SHIP_ROT_SMOOTH * dt);
    ship.theta += (targetPitch - ship.theta) * aSmooth;
    ship.phi += (targetRoll - ship.phi) * aSmooth;

    // 3) Minimal water-contact metrics (used by wake + HUD)
    if (inWaterNow) {
        ship.airTime = 0.0f;
        ship.timeInWater += dt;
        ship.hasTouchedWater = true;
    } else {
        ship.airTime += dt;
        ship.timeInWater = 0.0f;
    }

    ship.inWater = inWaterNow || (ship.airTime < Config::SHIP_INWATER_GRACE);

    state.submergedFraction =
        clampf(depthKeel / std::max(0.001f, 2.0f * T), 0.0f, 1.0f);
    state.submergedVolume = 0.0f;
    state.buoyancyForce =
        Config::SHIP_MASS * Config::G * (state.submergedFraction * 2.0f);

    float m0 = 0.0f;
    for (const auto &w : state.waves)
        m0 += 0.5f * w.amplitude * w.amplitude;
    state.Hs = 4.0f * std::sqrt(std::max(0.0f, m0));
}

// Wrapper with sub-stepping to avoid frame rate dependent physics artifacts
inline void updateShipPhysics(State &state, const Input &input, float dt) {
    // Cap dt to avoid spiral-of-death if the app hitches.
    dt = clampf(dt, 0.0f, 0.10f);

    const float maxStep = 1.0f / 120.0f; // 120 Hz sub-step
    int steps = (int)std::ceil(dt / maxStep);
    steps = clampi(steps, 1, 8);

    const float h = dt / (float)steps;
    for (int i = 0; i < steps; ++i) {
        updateShipPhysicsStep(state, input, h);
    }
}

// SPHERICAL GERSTNER WAVES
inline void sampleGerstnerSphere(const std::vector<WaveComponent> &waves,
                                 float theta, float phi, float baseRadius,
                                 float time, const VirtualPosition &virtPos,
                                 const std::vector<IslandSpec> &islands,
                                 vec3 &outPos, vec3 &outNormal) {
    vec3 basePos = sphericalToCartesian(theta, phi, baseRadius);
    vec3 up = basePos.normalized();

    float sinPhi = std::sin(phi);
    float cosTheta = std::cos(theta);
    float sinTheta = std::sin(theta);

    vec3 east(-sinTheta, 0.0f, cosTheta);
    if (east.length_squared() < 1e-6f)
        east = vec3(1.0f, 0.0f, 0.0f);
    east = east.normalized();

    vec3 north = cross(up, east).normalized();

    float localX = basePos.x + virtPos.x;
    float localZ = basePos.z + virtPos.z;

    float calm = 1.0f;
    for (const auto &isl : islands) {
        const float aCalm = isl.aLand * 1.25f;
        const float cAng = clampf(dot3(up, isl.dirLocal), -1.0f, 1.0f);
        const float ang = std::acos(cAng);
        if (ang < aCalm) {
            float u = 1.0f - (ang / std::max(1e-6f, aCalm));
            float s = u * u * (3.0f - 2.0f * u);
            calm = std::min(calm, 1.0f - 0.9f * s);
        }
    }

    float dispX = 0.0f, dispZ = 0.0f, dispY = 0.0f;
    vec3 dPdx(1.0f, 0.0f, 0.0f);
    vec3 dPdz(0.0f, 0.0f, 1.0f);

    for (const auto &w : waves) {
        const float A = w.amplitude * calm;
        const float k = w.k;
        const float Q = w.steepness;
        const float omega = w.omega;

        const float dx = w.dir.x;
        const float dz = w.dir.z;

        const float phase =
            k * (dx * localX + dz * localZ) - omega * time + w.phase;
        const float s = std::sin(phase);
        const float c = std::cos(phase);

        dispX += Q * A * dx * c;
        dispZ += Q * A * dz * c;
        dispY += A * s;
        dispY += 0.25f * A * A * k * (1.0f + c * c);

        dPdx.x += -Q * A * k * dx * dx * s;
        dPdx.y += A * k * dx * c;
        dPdx.z += -Q * A * k * dz * dx * s;

        dPdz.x += -Q * A * k * dx * dz * s;
        dPdz.y += A * k * dz * c;
        dPdz.z += -Q * A * k * dz * dz * s;
    }

    vec3 displacement = east * dispX + north * dispZ + up * dispY;
    outPos = basePos + displacement;

    vec3 tangentNormal = cross(dPdz, dPdx);
    if (tangentNormal.length_squared() < 1e-12f) {
        tangentNormal = vec3(0, 1, 0);
    }
    tangentNormal = tangentNormal.normalized();

    outNormal = (east * tangentNormal.x + up * tangentNormal.y +
                 north * tangentNormal.z)
                    .normalized();
}

// SHIP-WATER VISUAL INTERACTION WITH KELVIN WAKE
inline void applyShipWaterInteraction(const State &state, vec3 &posWorld) {
    // Wake + near field displacement around the ship.
    // This is purely a visual deformation applied to the ocean surface mesh.

    const float R = Config::PLANET_RADIUS;

    const float shipHeading = state.shipState.psi;
    const float cosH = std::cos(shipHeading);
    const float sinH = std::sin(shipHeading);

    const vec3 shipForward(sinH, 0.0f, cosH);
    const vec3 shipRight(cosH, 0.0f, -sinH);

    const float waterR = R + state.currentWaveHeight;
    const vec3 waterlinePos(0.0f, waterR, 0.0f);

    const vec3 rel = posWorld - waterlinePos;
    const float x = dot3(rel, shipRight);
    const float z = dot3(rel, shipForward);

    const float halfL = 0.5f * Config::SHIP_LENGTH * Config::SHIP_SCALE *
                        Config::SHIP_PART_RADIUS_SCALE_L;
    const float halfW = 0.5f * Config::SHIP_BEAM * Config::SHIP_SCALE *
                        Config::SHIP_PART_RADIUS_SCALE_W;

    // Quick reject far away from the ship (saves a lot of work per-vertex).
    if (std::fabs(x) > halfW * 5.0f || std::fabs(z) > halfL * 8.0f)
        return;

    const float q = std::sqrt((x * x) / (halfW * halfW + 1e-6f) +
                              (z * z) / (halfL * halfL + 1e-6f));

    const vec3 n = posWorld.normalized();

    const float wet = clampf(state.submergedFraction, 0.0f, 1.0f);
    const float speedFactor =
        clampf(state.virtualPos.speed / Config::SAIL_SPEED, 0.0f, 1.8f);

    const float effect = (0.30f + 0.70f * wet) * (0.35f + 0.65f * speedFactor);

    if (q < 1.55f) {
        const float t = smoothstep01(clampf(1.0f - (q / 1.55f), 0.0f, 1.0f));

        const float carve = Config::SHIP_PART_DEPTH * effect * (t * t);
        posWorld -= n * carve;

        // Side push that throws water outwards from the hull.
        const float push = Config::SHIP_PART_PUSH * effect * (t * t);
        const float sgn = (x >= 0.0f) ? 1.0f : -1.0f;
        posWorld += shipRight * (sgn * push);

        // A gentle upward berm near the sides (helps the wake "read" visually).
        const float side = clampf(std::fabs(x) / (halfW + 1e-6f), 0.0f, 1.0f);
        const float berm = 0.40f * Config::SHIP_PART_DEPTH * effect * t * side;
        posWorld += n * berm;
    }

    // bow wave
    const float bowLen = Config::SHIP_BOW_WAVE_LENGTH * Config::SHIP_SCALE;
    if (z < 0.0f && z > -bowLen && state.virtualPos.speed > 0.2f) {
        const float u = 1.0f - (-z / bowLen);
        const float lateral =
            std::exp(-(x * x) / (2.0f * halfW * halfW + 1e-6f));

        const float bow =
            Config::SHIP_BOW_WAVE_HEIGHT * effect * u * u * lateral;
        posWorld += n * bow;
    }

    if (z > 0.0f && z < 12.0f * Config::SHIP_SCALE &&
        state.virtualPos.speed > 0.5f) {
        const float s = z; // Distance behind ship
        const float lateral =
            std::exp(-(x * x) / (2.5f * halfW * halfW + 1e-6f));
        const float decay = std::exp(-0.25f * s);

        const float k = TWO_PI / std::max(0.25f, Config::WAKE_WAVELENGTH);
        const float chop = std::sin(k * (1.6f * s) - 2.5f * state.t) +
                           0.35f * std::sin(k * (3.4f * s) - 4.7f * state.t);

        const float stern =
            0.35f * Config::WAKE_AMPLITUDE * effect * lateral * decay * chop;
        posWorld += n * stern;
    }

    // kelvin wake
    if (z > 0.0f && state.virtualPos.speed > 0.5f) {
        const float wakeH =
            computeKelvinWake(x, -z, state.virtualPos.speed, state.t);
        posWorld += n * wakeH * (0.60f + 0.40f * wet);
    }
}

// PLANET MESH
inline void buildPlanetMesh(State &state) {
    const int nTheta = Config::PLANET_SUBDIV_THETA;
    const int nPhi = Config::PLANET_SUBDIV_PHI;
    const float R = Config::PLANET_RADIUS;

    state.planetBaseVerts.clear();
    state.planetIndices.clear();
    state.planetTheta.clear();
    state.planetPhi.clear();

    for (int j = 0; j <= nPhi; ++j) {
        float phi = PI * (float)j / (float)nPhi;
        for (int i = 0; i <= nTheta; ++i) {
            float theta = TWO_PI * (float)i / (float)nTheta;
            float wrappedTheta = (i == nTheta) ? 0.0f : theta;

            vec3 pos = sphericalToCartesian(wrappedTheta, phi, R);
            state.planetBaseVerts.push_back(pos);
            state.planetTheta.push_back(wrappedTheta);
            state.planetPhi.push_back(phi);
        }
    }

    for (int j = 0; j < nPhi; ++j) {
        for (int i = 0; i < nTheta; ++i) {
            int row0 = j * (nTheta + 1);
            int row1 = (j + 1) * (nTheta + 1);

            int i00 = row0 + i;
            int i10 = row0 + i + 1;
            int i01 = row1 + i;
            int i11 = row1 + i + 1;

            state.planetIndices.push_back(i00);
            state.planetIndices.push_back(i01);
            state.planetIndices.push_back(i11);

            state.planetIndices.push_back(i00);
            state.planetIndices.push_back(i11);
            state.planetIndices.push_back(i10);
        }
    }
}

inline void rebuildPlanetTriangles(State &state, UnifiedScene &scene) {
    const float R = Config::PLANET_RADIUS;
    const size_t numVerts = state.planetBaseVerts.size();
    const size_t numTris = state.planetIndices.size() / 3;

    state.tmpDisplacedVerts.resize(numVerts);

    for (size_t i = 0; i < numVerts; ++i) {
        const float theta = state.planetTheta[i];
        const float phi = state.planetPhi[i];

        vec3 pos, normal;
        sampleGerstnerSphere(state.waves, theta, phi, R, state.t,
                             state.virtualPos, state.islands, pos, normal);

        pos = rotateY(pos, state.planetRotationY);
        pos = rotateX(pos, state.planetRotationX);

        applyShipWaterInteraction(state, pos);
        state.tmpDisplacedVerts[i] = pos;
    }

    state.tmpTriangleVerts.resize(state.planetIndices.size());
    for (size_t t = 0; t < numTris; ++t) {
        state.tmpTriangleVerts[t * 3 + 0] =
            state.tmpDisplacedVerts[state.planetIndices[t * 3 + 0]];
        state.tmpTriangleVerts[t * 3 + 1] =
            state.tmpDisplacedVerts[state.planetIndices[t * 3 + 1]];
        state.tmpTriangleVerts[t * 3 + 2] =
            state.tmpDisplacedVerts[state.planetIndices[t * 3 + 2]];
    }

    if (state.ocean.isValid() && state.ocean.index < scene.meshes.size()) {
        auto &meshVerts = scene.meshes[state.ocean.index].triangleVerts;
        meshVerts.swap(state.tmpTriangleVerts);
        scene.markMeshDirty(state.ocean.index);
    }
}

// SEAFLOOR AND ISLAND GRASS

inline float angularRadiusFromChord(float chordRadius) {
    const float denom = std::max(1e-4f, 2.0f * Config::PLANET_RADIUS);
    return 2.0f * std::asin(clampf(chordRadius / denom, 0.0f, 0.999f));
}

inline float seafloorRadiusAtDirLocal(const vec3 &dirUnitLocal,
                                      const State &state, float &outHeight) {
    const float seaLevelR = Config::PLANET_RADIUS;
    const float baseR = Config::PLANET_RADIUS - Config::SEAFLOOR_DEPTH;

    float rOut = baseR;
    outHeight = -Config::SEAFLOOR_DEPTH;

    for (const auto &isl : state.islands) {
        const float c = clampf(dot3(dirUnitLocal, isl.dirLocal), -1.0f, 1.0f);
        const float ang = std::acos(c);

        float r = baseR;
        float h = -Config::SEAFLOOR_DEPTH;

        if (ang <= isl.aLand) {
            float u = 1.0f - (ang / std::max(1e-6f, isl.aLand));
            float s = u * u * (3.0f - 2.0f * u);
            s = s * s;
            r = seaLevelR + isl.height * s;
            h = isl.height * s;
        } else if (ang < isl.aShelf) {
            float v =
                (ang - isl.aLand) / std::max(1e-6f, (isl.aShelf - isl.aLand));
            float s = v * v * (3.0f - 2.0f * v);
            r = lerpf(seaLevelR, baseR, s);
            h = lerpf(0.0f, -Config::SEAFLOOR_DEPTH, s);
        }

        if (r > rOut) {
            rOut = r;
            outHeight = h;
        }
    }

    return rOut;
}

inline void buildSeafloorAndGrassBase(State &state) {
    const size_t numVerts = state.planetTheta.size();
    if (numVerts == 0)
        return;

    state.seafloorBaseVertsLocal.resize(numVerts);
    state.grassBaseVertsLocal.resize(numVerts);
    state.grassMask.resize(numVerts);

    const float grassThreshold = 0.3f;

    for (size_t i = 0; i < numVerts; ++i) {
        const float theta = state.planetTheta[i];
        const float phi = state.planetPhi[i];

        const vec3 dirLocal =
            sphericalToCartesian(theta, phi, 1.0f).normalized();
        float height;
        const float r = seafloorRadiusAtDirLocal(dirLocal, state, height);

        state.seafloorBaseVertsLocal[i] = dirLocal * r;

        const bool isGrass = (height > grassThreshold);
        state.grassMask[i] = isGrass;

        const float grassR = r + (isGrass ? 0.05f : -1000.0f);
        state.grassBaseVertsLocal[i] = dirLocal * grassR;
    }
}

inline void rebuildSeafloorTriangles(State &state, UnifiedScene &scene) {
    if (!state.seafloor.isValid())
        return;

    const size_t numVerts = state.planetTheta.size();
    const size_t numTris = state.planetIndices.size() / 3;

    state.tmpDisplacedVerts.resize(numVerts);

    const bool haveCache = (state.seafloorBaseVertsLocal.size() == numVerts);
    for (size_t i = 0; i < numVerts; ++i) {
        vec3 pos;
        if (haveCache) {
            pos = state.seafloorBaseVertsLocal[i];
        } else {
            const float theta = state.planetTheta[i];
            const float phi = state.planetPhi[i];
            const vec3 dirLocal =
                sphericalToCartesian(theta, phi, 1.0f).normalized();
            float height;
            const float r = seafloorRadiusAtDirLocal(dirLocal, state, height);
            pos = dirLocal * r;
        }

        pos = rotateY(pos, state.planetRotationY);
        pos = rotateX(pos, state.planetRotationX);
        state.tmpDisplacedVerts[i] = pos;
    }

    state.tmpTriangleVerts.resize(state.planetIndices.size());
    for (size_t t = 0; t < numTris; ++t) {
        const int i0 = state.planetIndices[t * 3 + 0];
        const int i1 = state.planetIndices[t * 3 + 1];
        const int i2 = state.planetIndices[t * 3 + 2];

        // Reverse winding so normals point inward (underwater view)
        state.tmpTriangleVerts[t * 3 + 0] = state.tmpDisplacedVerts[i0];
        state.tmpTriangleVerts[t * 3 + 1] = state.tmpDisplacedVerts[i2];
        state.tmpTriangleVerts[t * 3 + 2] = state.tmpDisplacedVerts[i1];
    }

    if (state.seafloor.index < scene.meshes.size()) {
        auto &meshVerts = scene.meshes[state.seafloor.index].triangleVerts;
        meshVerts.swap(state.tmpTriangleVerts);
        scene.markMeshDirty(state.seafloor.index);
    }
}

// Build separate mesh for grass-covered island tops
inline void rebuildIslandGrassTriangles(State &state, UnifiedScene &scene) {
    if (!state.islandGrass.isValid())
        return;

    const size_t numVerts = state.planetTheta.size();
    const size_t numTris = state.planetIndices.size() / 3;

    state.tmpDisplacedVerts.resize(numVerts);

    const bool haveCache = (state.grassBaseVertsLocal.size() == numVerts) &&
                           (state.grassMask.size() == numVerts);

    if (!haveCache) {
        state.tmpBool.assign(numVerts, false);
    }

    for (size_t i = 0; i < numVerts; ++i) {
        vec3 pos;
        bool isGrass = false;

        if (haveCache) {
            pos = state.grassBaseVertsLocal[i];
            isGrass = state.grassMask[i];
        } else {
            const float theta = state.planetTheta[i];
            const float phi = state.planetPhi[i];

            const vec3 dirLocal =
                sphericalToCartesian(theta, phi, 1.0f).normalized();
            float height;
            const float r = seafloorRadiusAtDirLocal(dirLocal, state, height);

            const float grassThreshold = 0.3f;
            isGrass = (height > grassThreshold);

            const float grassR = r + (isGrass ? 0.05f : -1000.0f);
            pos = dirLocal * grassR;

            state.tmpBool[i] = isGrass;
        }

        pos = rotateY(pos, state.planetRotationY);
        pos = rotateX(pos, state.planetRotationX);
        state.tmpDisplacedVerts[i] = pos;
    }

    const std::vector<bool> &mask = haveCache ? state.grassMask : state.tmpBool;

    state.tmpTriangleVerts.clear();
    state.tmpTriangleVerts.reserve(state.planetIndices.size());

    for (size_t t = 0; t < numTris; ++t) {
        const int i0 = state.planetIndices[t * 3 + 0];
        const int i1 = state.planetIndices[t * 3 + 1];
        const int i2 = state.planetIndices[t * 3 + 2];

        if (mask[i0] && mask[i1] && mask[i2]) {
            // Reverse winding to match seafloor
            state.tmpTriangleVerts.push_back(state.tmpDisplacedVerts[i0]);
            state.tmpTriangleVerts.push_back(state.tmpDisplacedVerts[i2]);
            state.tmpTriangleVerts.push_back(state.tmpDisplacedVerts[i1]);
        }
    }

    if (state.islandGrass.index < scene.meshes.size()) {
        auto &meshVerts = scene.meshes[state.islandGrass.index].triangleVerts;
        meshVerts.swap(state.tmpTriangleVerts);
        scene.markMeshDirty(state.islandGrass.index);
    }
}

// SHIP VISUAL UPDATE
inline void updateShipVisual(State &state, UnifiedScene &scene) {
    // Hull is the primary visual + the reference for transform updates.
    if (!state.shipHull.isValid())
        return;

    const ShipState &ship = state.shipState;

    float shipRadius = Config::PLANET_RADIUS + ship.y + state.shipModelYOffset;
    vec3 shipPos(0.0f, shipRadius, 0.0f);

    const vec3 shipRot(ship.theta, ship.psi + PI, ship.phi);

    state.shipHull.setPosition(shipPos);
    state.shipHull.setRotation(shipRot);
}

// ORBIT CAMERA
inline void updateOrbitCamera(State &state, UnifiedScene &scene,
                              const Input &input, float dt) {
    state.orbitYaw += Config::ORBIT_SPEED * dt;
    state.orbitYaw += input.mouseDX * Config::ORBIT_MOUSE_SENS;
    state.orbitPitch += -input.mouseDY * Config::ORBIT_MOUSE_SENS;
    state.orbitPitch = clampf(state.orbitPitch, Config::ORBIT_PITCH_MIN,
                              Config::ORBIT_PITCH_MAX);

    float shipRadius =
        Config::PLANET_RADIUS + state.shipState.y + state.shipModelYOffset;
    vec3 shipPos(0.0f, shipRadius, 0.0f);

    const float cy = std::cos(state.orbitYaw), sy = std::sin(state.orbitYaw);
    const float cp = std::cos(state.orbitPitch),
                sp = std::sin(state.orbitPitch);

    const float r = Config::ORBIT_RADIUS;
    vec3 offset(r * cp * sy, r * sp + Config::ORBIT_HEIGHT, r * cp * cy);

    if (!state.smoothCameraTargetInitialized) {
        state.smoothCameraTarget = shipPos;
        state.smoothCameraTargetInitialized = true;
    }

    state.smoothCameraTarget =
        state.smoothCameraTarget +
        (shipPos - state.smoothCameraTarget) * std::min(1.0f, 5.0f * dt);

    vec3 camPos = state.smoothCameraTarget + offset;

    scene.camera.setPosition(camPos);
    scene.camera.setTarget(state.smoothCameraTarget + vec3(0.0f, 3.0f, 0.0f));
}

// GAME LIFECYCLE
inline void start(State &state, UnifiedScene &scene) {
    scene.setHDRI("sky.hdr", 1.0f);

    scene.addDirectionalLight(vec3(-0.35f, -1.0f, -0.25f).normalized(),
                              vec3(1.0f, 0.98f, 0.92f), 0.9f);

    scene.addPointLight(vec3(0.0f, 0.0f, 0.0f), vec3(0.25f, 0.30f, 0.35f),
                        0.15f, 1000.0f);

    scene.addDirectionalLight(vec3(0.35f, 1.0f, 0.25f).normalized(),
                              vec3(0.45f, 0.50f, 0.60f), 0.35f);

    // Compute hydrodynamic coefficients
    state.hydro = computeHydroCoefficients();

    // Generate ocean spectrum
    generateOceanSpectrum(state);

    // Build planet mesh
    buildPlanetMesh(state);

    // Build island set BIGGER ISLANDS
    state.islands.clear();
    state.islands.reserve(1 + Config::NUM_EXTRA_ISLANDS);

    auto addIslandSpec = [&](const vec3 &dir, float chordRadius, float height) {
        IslandSpec isl;
        isl.dirLocal = dir.normalized();
        isl.aLand = angularRadiusFromChord(chordRadius);
        isl.aShelf = std::min(PI, isl.aLand * 1.5f);
        isl.height = height;
        state.islands.push_back(isl);
    };

    // Main island
    addIslandSpec(
        sphericalToCartesian(Config::ISLAND_THETA, Config::ISLAND_PHI, 1.0f),
        Config::ISLAND_RADIUS, Config::ISLAND_HEIGHT);

    const float yaw = Config::ORBIT_YAW_OFFSET;
    const vec3 shipTrackPlaneN(-std::cos(yaw), 0.0f, std::sin(yaw));

    std::mt19937 rng(Config::ISLAND_SEED);
    std::uniform_real_distribution<float> uni01(0.0f, 1.0f);
    std::uniform_real_distribution<float> uniRadius(
        Config::EXTRA_ISLAND_RADIUS_MIN, Config::EXTRA_ISLAND_RADIUS_MAX);
    std::uniform_real_distribution<float> uniHeight(
        Config::EXTRA_ISLAND_HEIGHT_MIN, Config::EXTRA_ISLAND_HEIGHT_MAX);

    const int maxTries = Config::NUM_EXTRA_ISLANDS * 100;
    int tries = 0;
    while ((int)state.islands.size() < 1 + Config::NUM_EXTRA_ISLANDS &&
           tries++ < maxTries) {
        const float y = uni01(rng) * 2.0f - 1.0f;
        const float a = uni01(rng) * TWO_PI;
        const float rr = std::sqrt(std::max(0.0f, 1.0f - y * y));
        const vec3 dir(std::cos(a) * rr, y, std::sin(a) * rr);

        const float chordR = uniRadius(rng);
        const float h = uniHeight(rng);
        const float aLand = angularRadiusFromChord(chordR);

        // Keep islands away from ship track
        const float distToPlane =
            std::asin(std::abs(dot3(dir, shipTrackPlaneN)));
        if (distToPlane < (aLand + Config::SHIP_TRACK_CLEARANCE))
            continue;

        // Check overlap with existing islands
        bool ok = true;
        for (const auto &other : state.islands) {
            const float ang =
                std::acos(clampf(dot3(dir, other.dirLocal), -1.0f, 1.0f));
            if (ang < (aLand + other.aLand) * 0.85f + 0.08f) {
                ok = false;
                break;
            }
        }
        if (!ok)
            continue;

        addIslandSpec(dir, chordR, h);
    }

    buildSeafloorAndGrassBase(state);

    // Ocean mesh
    UnifiedMeshDesc oceanDesc;
    oceanDesc.type = UnifiedMeshDesc::Type::Triangles;
    oceanDesc.material = UnifiedMaterial::Water();
    oceanDesc.triangleVerts.clear();
    oceanDesc.name = "OceanPlanet";
    oceanDesc.isDynamic = true;

    state.ocean = scene.addMesh(oceanDesc);
    rebuildPlanetTriangles(state, scene);

    // Initialize ship state - START ABOVE WATER FOR DROP
    state.shipState.y = Config::SHIP_DROP_HEIGHT; // rahghhhhh
    state.shipState.yVel = 0.0f;
    state.shipState.yAccel = 0.0f;
    state.shipState.surge = 0.0f;
    state.shipState.surgeVel = 0.0f;
    state.shipState.sway = 0.0f;
    state.shipState.swayVel = 0.0f;
    state.shipState.phi = 0.0f;
    state.shipState.phiVel = 0.0f;
    state.shipState.phiAccel = 0.0f;
    state.shipState.theta = 0.0f;
    state.shipState.thetaVel = 0.0f;
    state.shipState.thetaAccel = 0.0f;
    state.shipState.psi = 0.0f;
    state.shipState.psiVel = 0.0f;
    state.shipState.inWater = false;
    state.shipState.timeInWater = 0.0f;
    state.shipState.hasTouchedWater = false;
    state.shipState.airTime = 0.0f;
    state.shipState.waveVelocity = vec3(0.0f);
    state.shipState.waveAcceleration = vec3(0.0f);
    state.shipState.localWaveHeight = 0.0f;
    state.shipState.waveFollowBlend = 0.0f;
    state.shipState.kineticEnergy = 0.0f;
    state.shipState.potentialEnergy = 0.0f;

    state.virtualPos.x = 0.0f;
    state.virtualPos.z = 0.0f;
    state.virtualPos.heading = 0.0f;
    state.virtualPos.speed = 0.0f;
    state.virtualPos.relVelX = 0.0f;
    state.virtualPos.relVelZ = 0.0f;

    state.planetRotationY = 0.0f;
    state.planetRotationX = 0.0f;
    state.planetRotationYAccum = 0.0f;
    state.planetRotationXAccum = 0.0f;

    state.shipModelYOffset = Config::SHIP_MODEL_Y_OFFSET;

    vec3 shipMinTmp(0.0f), shipMaxTmp(0.0f);
    vec3 shipCenterTmp(0.0f);
    bool haveShipBounds = false;

    if (Config::SHIP_AUTO_COMPUTE_MODEL_Y_OFFSET) {
        if (computeObjBounds(Config::SHIP_OBJ, shipMinTmp, shipMaxTmp)) {
            haveShipBounds = true;
            shipCenterTmp = aabbCenter(shipMinTmp, shipMaxTmp);

            // Model origin to keel distance in OBJ local space.
            state.shipModelYOffset = (-shipMinTmp.y) * Config::SHIP_SCALE;
        }
    }

    UnifiedMaterial shipMat = UnifiedMaterial::WoodOak();
    shipMat.name = "Ship";

    UnifiedMeshDesc shipDesc =
        UnifiedMeshDesc::FromOBJ(Config::SHIP_OBJ, shipMat);
    shipDesc.name = "Ship";
    shipDesc.isDynamic = true;

    if (haveShipBounds && !shipDesc.triangleVerts.empty()) {
        const vec3 centerXZ(shipCenterTmp.x, 0.0f, shipCenterTmp.z);
        for (auto &v : shipDesc.triangleVerts) {
            v.x -= centerXZ.x;
            v.z -= centerXZ.z;
        }
    }

    state.shipHull = scene.addMesh(shipDesc);
    state.shipHull.setScale(vec3(Config::SHIP_SCALE));

    float shipRadius = Config::PLANET_RADIUS + Config::SHIP_DROP_HEIGHT +
                       state.shipModelYOffset;
    vec3 shipPos(0.0f, shipRadius, 0.0f);
    vec3 shipRot(0.0f, PI, 0.0f);

    state.shipHull.setPosition(shipPos);
    state.shipHull.setRotation(shipRot);
    // seafloor
    UnifiedMaterial sandMat = UnifiedMaterial::PlainClay();
    sandMat.albedo = vec3(0.76f, 0.70f, 0.50f); // sandy color
    sandMat.roughness = 0.9f;

    UnifiedMeshDesc floorDesc;
    floorDesc.type = UnifiedMeshDesc::Type::Triangles;
    floorDesc.setMaterial(sandMat);
    floorDesc.triangleVerts.clear();
    floorDesc.name = "Seafloor";
    floorDesc.isDynamic = true;

    state.seafloor = scene.addMesh(floorDesc);
    rebuildSeafloorTriangles(state, scene);

    // island grass layer
    UnifiedMaterial grassMat;
    grassMat.albedo = vec3(0.20f, 0.45f, 0.12f); // grass green
    grassMat.roughness = 0.85f;
    grassMat.metallic = 0.0f;
    grassMat.specular = vec3(0.02f);
    grassMat.name = "IslandGrass";

    UnifiedMeshDesc grassDesc;
    grassDesc.type = UnifiedMeshDesc::Type::Triangles;
    grassDesc.setMaterial(grassMat);
    grassDesc.triangleVerts.clear();
    grassDesc.name = "IslandGrass";
    grassDesc.isDynamic = true;

    state.islandGrass = scene.addMesh(grassDesc);
    rebuildIslandGrassTriangles(state, scene);

    updateOrbitCamera(state, scene, Input{}, 0.0f);
}

inline void update(State &state, UnifiedScene &scene, const Input &input,
                   float dt) {
    state.t += dt;

    if (input.ePressed) {
        state.shipEngineOn = !state.shipEngineOn;
    }

    if (input.rPressed) {
        // reset ship state to drop again
        state.shipState.y = Config::SHIP_DROP_HEIGHT;
        state.shipState.yVel = 0.0f;
        state.shipState.yAccel = 0.0f;
        state.shipState.surge = 0.0f;
        state.shipState.surgeVel = 0.0f;
        state.shipState.sway = 0.0f;
        state.shipState.swayVel = 0.0f;
        state.shipState.phi = 0.0f;
        state.shipState.phiVel = 0.0f;
        state.shipState.phiAccel = 0.0f;
        state.shipState.theta = 0.0f;
        state.shipState.thetaVel = 0.0f;
        state.shipState.thetaAccel = 0.0f;
        state.shipState.psi = 0.0f;
        state.shipState.psiVel = 0.0f;
        state.shipState.inWater = false;
        state.shipState.timeInWater = 0.0f;
        state.shipState.hasTouchedWater = false;
        state.shipState.airTime = 0.0f;
        state.shipState.waveVelocity = vec3(0.0f);
        state.shipState.waveAcceleration = vec3(0.0f);
        state.shipState.localWaveHeight = 0.0f;
        state.shipState.waveFollowBlend = 0.0f;
        state.shipState.kineticEnergy = 0.0f;
        state.shipState.potentialEnergy = 0.0f;

        state.virtualPos.x = 0.0f;
        state.virtualPos.z = 0.0f;
        state.virtualPos.heading = 0.0f;
        state.virtualPos.speed = 0.0f;
        state.virtualPos.relVelX = 0.0f;
        state.virtualPos.relVelZ = 0.0f;

        state.planetRotationY = 0.0f;
        state.planetRotationX = 0.0f;
        state.planetRotationYAccum = 0.0f;
        state.planetRotationXAccum = 0.0f;
    }

    // Update physics
    updateShipPhysics(state, input, dt);

    // Update visuals
    updateShipVisual(state, scene);
    rebuildPlanetTriangles(state, scene);
    rebuildSeafloorTriangles(state, scene);
    rebuildIslandGrassTriangles(state, scene);
    updateOrbitCamera(state, scene, input, dt);
}

inline std::string getWindowTitle(const State &state, float fps) {
    char buf[400];
    if (!state.shipState.hasTouchedWater) {
        std::snprintf(buf, sizeof(buf),
                      "%s | FPS: %d | DROPPING! Height: %.1fm Vel: %.1fm/s",
                      Config::WINDOW_TITLE, (int)fps, state.shipState.y,
                      state.shipState.yVel);
    } else if (!state.shipState.inWater) {
        std::snprintf(buf, sizeof(buf),
                      "%s | FPS: %d | AIRBORNE Height: %.1fm Vel: %.1fm/s | %s",
                      Config::WINDOW_TITLE, (int)fps, state.shipState.y,
                      state.shipState.yVel,
                      state.shipEngineOn ? "SAILING" : "STOPPED");
    } else {
        std::snprintf(buf, sizeof(buf),
                      "%s | FPS: %d | Hs=%.1fm | Submerged: %.0f%% | %s",
                      Config::WINDOW_TITLE, (int)fps, state.Hs,
                      state.submergedFraction * 100.0f,
                      state.shipEngineOn ? "SAILING" : "STOPPED");
    }
    return std::string(buf);
}

inline void onGameOver(const State &) {}

} // namespace Game

#endif // GAME_CUH