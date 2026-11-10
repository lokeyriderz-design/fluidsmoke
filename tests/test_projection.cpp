#include <cstdio>
#include "../addon/src/projection.hpp"
static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)
static bool near(float a, float b, float e = 0.5f) { return std::fabs(a - b) < e; }
int main() {
    const float cam[3] = {100, 200, 50};
    const float idq[4] = {0, 0, 0, 1};          // looks along +Y
    float W = 1920, H = 1080;
    auto p = rf::project(cam, idq, 65, true, W, H, {100, 210, 50});
    CHECK(p.ok && near(p.px, 960) && near(p.py, 540) && near(p.depth, 10));
    p = rf::project(cam, idq, 65, true, W, H, {101, 210, 50});   // +X is to the right
    CHECK(p.ok && p.px > 960);
    p = rf::project(cam, idq, 65, true, W, H, {100, 210, 51});   // +Z is up -> smaller y
    CHECK(p.ok && p.py < 540);
    p = rf::project(cam, idq, 65, true, W, H, {100, 190, 50});   // behind
    CHECK(!p.ok);
    // vertical FOV 90: a point 10 m ahead and 10 m up sits on the top edge
    p = rf::project(cam, idq, 90, true, W, H, {100, 210, 60});
    CHECK(p.ok && near(p.py, 0, 0.01f));
    // horizontal FOV 90: a point 10 m ahead and 10 m right sits on the right edge
    p = rf::project(cam, idq, 90, false, W, H, {110, 210, 50});
    CHECK(p.ok && near(p.px, W, 0.01f));
    // quaternion is world->camera (inverted): this one makes the camera look along +X
    const float s = 0.70710678f; const float q90[4] = {0, 0, s, s};
    p = rf::project(cam, q90, 65, true, W, H, {110, 200, 50});
    CHECK(p.ok && near(p.px, 960) && near(p.py, 540));
    p = rf::project(cam, q90, 65, true, W, H, {100, 200, 50 + 5}); // not in front of (0 depth) -> rejected
    CHECK(!p.ok);
    const float zq[4] = {0, 0, 0, 0};
    CHECK(!rf::project(cam, zq, 65, true, W, H, {100, 210, 50}).ok);
    std::printf(fails ? "%d FAILED\n" : "all projection tests passed\n", fails);
    return fails ? 1 : 0;
}
