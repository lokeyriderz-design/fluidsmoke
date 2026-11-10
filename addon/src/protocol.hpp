// RealFluid wire protocol (text, one UDP datagram per frame, "RF1").
//
//   line 0:   RF1 <seq> <timeMs> <camX> <camY> <camZ> <qx> <qy> <qz> <qw> <fovDeg> <nWheels>
//   line 1..: W <vehId> <wheelIdx> <x> <y> <z> <vx> <vy> <vz> <radius> <slip> <sideSlip> <energy> <downForce> <mat1> <mat2>
//
// Lines are separated by '\n'. All numbers are plain decimal text. World space is
// BeamNG's (metres, Z up). This header is portable C++17 so it can be unit-tested
// off Windows.
#pragma once
#include <charconv>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rf {

struct Wheel {
    int   vehId = 0, wheelIdx = 0;
    float pos[3] = {0, 0, 0};
    float vel[3] = {0, 0, 0};
    float radius = 0, slip = 0, sideSlip = 0, energy = 0, downForce = 0;
    int   mat1 = 0, mat2 = 0;
};

struct Frame {
    uint32_t seq = 0;
    double   timeMs = 0;
    float    camPos[3] = {0, 0, 0};
    float    camRot[4] = {0, 0, 0, 1};  // x y z w
    float    fovDeg = 60;
    std::vector<Wheel> wheels;
};

namespace detail {
inline void splitTokens(std::string_view line, std::vector<std::string_view>& out) {
    out.clear();
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) ++i;
        size_t j = i;
        while (j < line.size() && line[j] != ' ' && line[j] != '\t' && line[j] != '\r') ++j;
        if (j > i) out.push_back(line.substr(i, j - i));
        i = j;
    }
}
inline bool toDouble(std::string_view s, double& v) {
    if (!s.empty() && s.front() == '+') s.remove_prefix(1);
    auto r = std::from_chars(s.data(), s.data() + s.size(), v);  // locale independent
    return r.ec == std::errc() && r.ptr == s.data() + s.size() && std::isfinite(v);
}
inline bool toFloat(std::string_view s, float& f) {
    double d;
    if (!toDouble(s, d)) return false;
    f = static_cast<float>(d);
    return true;
}
inline bool toInt(std::string_view s, int& n) {
    double d;
    if (!toDouble(s, d) || std::fabs(d) > 2.0e9) return false;
    n = static_cast<int>(d);
    return true;
}
}  // namespace detail

constexpr size_t kMaxWheels = 32;

// Returns true and fills `out` on success; on failure returns false with `err` set.
inline bool parsePacket(const char* data, size_t len, Frame& out, std::string& err) {
    using namespace detail;
    std::string_view text(data, len);
    std::vector<std::string_view> tok;
    out = Frame{};

    size_t nl = text.find('\n');
    std::string_view head = text.substr(0, nl);
    splitTokens(head, tok);
    if (tok.size() != 12 || tok[0] != "RF1") { err = "bad header"; return false; }

    double seq, t;
    int nWheels;
    if (!toDouble(tok[1], seq) || seq < 0 || seq > 4.0e9) { err = "bad seq"; return false; }
    out.seq = static_cast<uint32_t>(seq);
    if (!toDouble(tok[2], t)) { err = "bad time"; return false; }
    out.timeMs = t;
    for (int i = 0; i < 3; ++i) if (!toFloat(tok[3 + i], out.camPos[i])) { err = "bad cam pos"; return false; }
    for (int i = 0; i < 4; ++i) if (!toFloat(tok[6 + i], out.camRot[i])) { err = "bad cam rot"; return false; }
    if (!toFloat(tok[10], out.fovDeg)) { err = "bad fov"; return false; }
    if (!toInt(tok[11], nWheels) || nWheels < 0 || nWheels > static_cast<int>(kMaxWheels)) { err = "bad wheel count"; return false; }

    size_t pos = (nl == std::string_view::npos) ? text.size() : nl + 1;
    while (pos < text.size()) {
        size_t e = text.find('\n', pos);
        std::string_view line = text.substr(pos, e == std::string_view::npos ? std::string_view::npos : e - pos);
        pos = (e == std::string_view::npos) ? text.size() : e + 1;
        splitTokens(line, tok);
        if (tok.empty()) continue;
        if (tok[0] != "W" || tok.size() != 16) { err = "bad wheel line"; return false; }
        Wheel w;
        bool ok = toInt(tok[1], w.vehId) && toInt(tok[2], w.wheelIdx);
        for (int i = 0; i < 3 && ok; ++i) ok = toFloat(tok[3 + i], w.pos[i]);
        for (int i = 0; i < 3 && ok; ++i) ok = toFloat(tok[6 + i], w.vel[i]);
        ok = ok && toFloat(tok[9], w.radius) && toFloat(tok[10], w.slip) && toFloat(tok[11], w.sideSlip) &&
             toFloat(tok[12], w.energy) && toFloat(tok[13], w.downForce) && toInt(tok[14], w.mat1) && toInt(tok[15], w.mat2);
        if (!ok) { err = "bad wheel field"; return false; }
        out.wheels.push_back(w);
        if (out.wheels.size() > kMaxWheels) { err = "too many wheels"; return false; }
    }
    if (static_cast<int>(out.wheels.size()) != nWheels) { err = "wheel count mismatch"; return false; }
    return true;
}

}  // namespace rf
