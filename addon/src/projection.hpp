// World -> screen projection for BeamNG's camera. Portable C++17 (unit-tested off Windows).
//
// Convention verified from BeamNG's lua/ge/extensions/core/camera.lua: the camera's
// forward axis is (0,1,0), right is (1,0,0), up is (0,0,1), each rotated by the
// camera quaternion (x,y,z,w). World is Z-up, metres.
#pragma once
#include <cmath>

namespace rf {

struct Vec3 { float x, y, z; };
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

// Rotate v by unit quaternion q = (x,y,z,w).
inline Vec3 rotate(const float q[4], Vec3 v) {
    Vec3 qv{q[0], q[1], q[2]};
    Vec3 t = cross(qv, v);
    t = {2 * t.x, 2 * t.y, 2 * t.z};
    Vec3 c = cross(qv, t);
    return {v.x + q[3] * t.x + c.x, v.y + q[3] * t.y + c.y, v.z + q[3] * t.z + c.z};
}

struct Projected {
    bool  ok = false;
    float px = 0, py = 0;   // pixels, origin top-left
    float depth = 0;        // metres along the view axis
    float focalPx = 0;      // pixels per (metre at 1 m distance): size_px = size_m * focalPx / depth
};

inline Projected project(const float camPos[3], const float camRot[4], float fovDeg, bool fovIsVertical,
                         float w, float h, Vec3 p) {
    Projected r;
    if (w < 1 || h < 1 || fovDeg < 1 || fovDeg > 170) return r;
    float q[4] = {camRot[0], camRot[1], camRot[2], camRot[3]};
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len < 1e-6f) return r;
    for (float &c : q) c /= len;
    // Measured from a real BeamNG log: the camera quaternion maps WORLD -> CAMERA, so use its
    // inverse (conjugate) to get the camera axes in world space.
    q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2];

    Vec3 fwd = rotate(q, {0, 1, 0}), right = rotate(q, {1, 0, 0}), up = rotate(q, {0, 0, 1});
    Vec3 d = p - Vec3{camPos[0], camPos[1], camPos[2]};
    float z = dot(d, fwd);
    if (z < 0.2f) return r;

    float tanHalf = std::tan(fovDeg * 0.5f * 3.14159265f / 180.0f);
    float aspect = w / h;
    float tanV = fovIsVertical ? tanHalf : tanHalf / aspect;
    float tanH = tanV * aspect;
    float ndcX = dot(d, right) / (z * tanH);
    float ndcY = dot(d, up) / (z * tanV);
    r.px = (ndcX * 0.5f + 0.5f) * w;
    r.py = (0.5f - ndcY * 0.5f) * h;
    r.depth = z;
    r.focalPx = (h * 0.5f) / tanV;
    r.ok = std::isfinite(r.px) && std::isfinite(r.py);
    return r;
}

}  // namespace rf
