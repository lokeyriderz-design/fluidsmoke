#include <cstdio>
#include <string>
#include "../addon/src/protocol.hpp"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)

static bool parse(const std::string& s, rf::Frame& f, std::string& e) { return rf::parsePacket(s.data(), s.size(), f, e); }

int main() {
    rf::Frame f; std::string e;
    // exactly the layout the Lua side formats
    std::string good =
        "RF1 42 1234567 100.500 -20.250 33.000 0.00000 0.70711 0.00000 0.70711 65.00 2\n"
        "W 7 0 101.100 -19.900 30.500 12.00 0.50 0.00 0.320 6.25 1.10 340.5 5200.0 3 3\n"
        "W 7 1 101.100 -21.100 30.500 12.00 0.50 0.00 0.320 0.00 0.00 0.0 4800.0 3 3";
    CHECK(parse(good, f, e));
    CHECK(f.seq == 42);
    CHECK(f.wheels.size() == 2);
    CHECK(std::fabs(f.camPos[1] + 20.25f) < 1e-4f);
    CHECK(std::fabs(f.camRot[1] - 0.70711f) < 1e-4f);
    CHECK(std::fabs(f.fovDeg - 65.f) < 1e-4f);
    CHECK(f.wheels[0].vehId == 7 && f.wheels[1].wheelIdx == 1);
    CHECK(std::fabs(f.wheels[0].slip - 6.25f) < 1e-4f);
    CHECK(std::fabs(f.wheels[0].energy - 340.5f) < 1e-3f);
    CHECK(f.wheels[0].mat1 == 3);
    CHECK(parse(good + "\n", f, e));                       // trailing newline
    CHECK(parse("RF1 1 5 0 0 0 0 0 0 1 60 0\r\n", f, e));  // CRLF
    CHECK(parse("RF1 1 5 0 0 0 0 0 0 1 60 0", f, e) && f.wheels.empty());
    CHECK(!parse("", f, e));
    CHECK(!parse("XX1 1 5 0 0 0 0 0 0 1 60 0", f, e));
    CHECK(!parse("RF1 1 5 0 0 0 0 0 0 1 60", f, e));
    CHECK(!parse("RF1 1 5 0 0 nan 0 0 0 1 60 0", f, e));
    CHECK(!parse("RF1 1 5 0 0 inf 0 0 0 1 60 0", f, e));
    CHECK(!parse("RF1 1 5 0 0 0 0 0 0 1 60 1", f, e));     // count mismatch
    CHECK(!parse("RF1 1 5 0 0 0 0 0 0 1 60 0\nW 1 2 3", f, e));
    CHECK(!parse("RF1 1 5 0 0 0 0 0 0 1 60 99", f, e));
    CHECK(!parse("RF1 -1 5 0 0 0 0 0 0 1 60 0", f, e));
    std::string junk; for (int i = 0; i < 4000; ++i) junk.push_back(char((i * 131 + 7) & 0xff));
    CHECK(!parse(junk, f, e));                              // garbage must not crash
    std::printf(fails ? "%d FAILED\n" : "all protocol tests passed\n", fails);
    return fails ? 1 : 0;
}
