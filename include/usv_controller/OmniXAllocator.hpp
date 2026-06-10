#pragma once
//
// Fixed-geometry omni-X thrust allocator for Selene.
//
// The boat has 4 azimuth pods locked at a fixed ±45° "X" (the ArduSub Lua only ever parked
// the steering servos statically; allocation was done by the autopilot's motor mixer). With
// the pods fixed and the ESCs bidirectional, only the 4 thrust magnitudes vary, so the
// allocation is a *constant* linear map from desired body wrench (Fx, Fy, Mz) to 4 signed
// thrusts.
//
// Body frame: x forward (surge), y starboard (sway), z down; Mz = yaw moment (right-hand
// about +z down => positive turns the bow to starboard).
//
// For pod i at body position (px_i, py_i) whose thrust acts along unit direction
// (cos φ_i, sin φ_i), a signed thrust T_i produces:
//     Fx += T_i cos φ_i
//     Fy += T_i sin φ_i
//     Mz += T_i (px_i sin φ_i − py_i cos φ_i)
// Stacking the 4 pods gives B (3×4) with column_i = [cosφ_i, sinφ_i, px_i sinφ_i − py_i cosφ_i].
// We invert with the minimum-norm (least-effort) solution T = Bᵀ (B Bᵀ)⁻¹ τ, then normalize.
//
// All geometry is injected (ROS params) so the physical layout is configured, not hard-coded.

#include <array>
#include <cmath>

class OmniXAllocator {
public:
    struct Pod {
        double px;     // body x position [m] (forward +)
        double py;     // body y position [m] (starboard +)
        double phi;    // fixed thrust direction in body XY plane [rad], from +x toward +y
    };

    // pods: the 4 pods in output order (matches ActuatorMotors indices 0..3).
    // max_thrust: per-pod thrust [N] that maps to normalized ±1.0 output.
    OmniXAllocator(const std::array<Pod, 4> &pods, double max_thrust)
        : max_thrust_(max_thrust > 1e-6 ? max_thrust : 1.0)
    {
        // Build B (3x4) and its min-norm pseudo-inverse Bpinv = Bᵀ (B Bᵀ)⁻¹ (4x3).
        double B[3][4];
        for (int i = 0; i < 4; ++i) {
            const double c = std::cos(pods[i].phi);
            const double s = std::sin(pods[i].phi);
            B[0][i] = c;
            B[1][i] = s;
            B[2][i] = pods[i].px * s - pods[i].py * c;
        }

        // M = B Bᵀ (3x3, symmetric).
        double M[3][3] = {{0}};
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col)
                for (int k = 0; k < 4; ++k)
                    M[r][col] += B[r][k] * B[col][k];

        double Minv[3][3];
        invertible_ = invert3x3(M, Minv);

        // Bpinv = Bᵀ Minv  (4x3).
        for (int i = 0; i < 4; ++i)
            for (int col = 0; col < 3; ++col) {
                double v = 0.0;
                for (int k = 0; k < 3; ++k) v += B[k][i] * Minv[k][col];
                Bpinv_[i][col] = invertible_ ? v : 0.0;
            }
    }

    // Map desired body wrench to 4 normalized thruster commands in [-1, 1].
    // If any command saturates, all four are scaled down together to preserve direction.
    std::array<float, 4> allocate(double Fx, double Fy, double Mz) const {
        std::array<double, 4> T{};
        double peak = 0.0;
        for (int i = 0; i < 4; ++i) {
            T[i] = Bpinv_[i][0] * Fx + Bpinv_[i][1] * Fy + Bpinv_[i][2] * Mz;
            peak = std::max(peak, std::abs(T[i]) / max_thrust_);
        }
        const double scale = peak > 1.0 ? 1.0 / peak : 1.0;
        std::array<float, 4> out{};
        for (int i = 0; i < 4; ++i)
            out[i] = static_cast<float>(T[i] / max_thrust_ * scale);
        return out;
    }

    bool ok() const { return invertible_; }

private:
    static bool invert3x3(const double m[3][3], double inv[3][3]) {
        const double det =
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if (std::abs(det) < 1e-12) return false;
        const double id = 1.0 / det;
        inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id;
        inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * id;
        inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
        inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * id;
        inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
        inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * id;
        inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id;
        inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * id;
        inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
        return true;
    }

    double max_thrust_;
    bool invertible_{false};
    double Bpinv_[4][3]{};  // 4x3 min-norm pseudo-inverse
};
