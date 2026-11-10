// RealFluid - milestone 1: ReShade add-on skeleton.
//
// What this does (and only this):
//   * registers with ReShade as an add-on,
//   * listens on UDP 127.0.0.1:43871 for the BeamNG Lua sender (see src/protocol.hpp),
//   * logs, once per second, what it received and how the game renders (graphics API,
//     back buffer format, which depth-stencil buffers the game binds),
//   * draws NOTHING and changes NO rendering state.
//
// The log file (realfluid.log, next to the game exe, else %TEMP%) is what milestone 2
// (the GPU renderer) is designed from, so please send it back after a test drive.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <reshade.hpp>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>

#include "protocol.hpp"

extern "C" __declspec(dllexport) const char *NAME = "RealFluid";
extern "C" __declspec(dllexport) const char *DESCRIPTION =
    "Fluid tire smoke for BeamNG.drive (milestone 1: telemetry + diagnostics, no rendering).";

namespace {

constexpr unsigned short kPort = 43871;
constexpr ULONGLONG kDiagWindowMs = 20000;  // how long to watch depth-stencil binds

// ---------------------------------------------------------------- logging
std::mutex g_logMutex;
FILE *g_log = nullptr;

void openLog(HMODULE module) {
    char path[MAX_PATH] = {};
    DWORD n = GetModuleFileNameA(module, path, MAX_PATH);
    while (n > 0 && path[n - 1] != '\\' && path[n - 1] != '/') --n;
    path[n] = '\0';
    std::string primary = std::string(path) + "realfluid.log";
    g_log = std::fopen(primary.c_str(), "w");
    if (!g_log) {
        char tmp[MAX_PATH] = {};
        GetTempPathA(MAX_PATH, tmp);
        g_log = std::fopen((std::string(tmp) + "realfluid.log").c_str(), "w");
    }
}

void logf(const char *fmt, ...) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (!g_log) return;
    std::fprintf(g_log, "[%8.2fs] ", GetTickCount64() / 1000.0);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(g_log, fmt, ap);
    va_end(ap);
    std::fputc('\n', g_log);
    std::fflush(g_log);
}

// ---------------------------------------------------------------- telemetry
std::mutex g_stateMutex;
rf::Frame g_frame;                 // latest good packet
ULONGLONG g_lastRecvTick = 0;
std::atomic<uint64_t> g_packets{0}, g_bad{0}, g_bytes{0};

std::atomic<int> g_users{0};
std::atomic<bool> g_stop{false}, g_threadDone{true};

void udpThread() {
    g_threadDone = false;
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        logf("udp: WSAStartup failed (%d)", WSAGetLastError());
        g_threadDone = true;
        return;
    }
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    bool bound = false;
    if (s != INVALID_SOCKET) {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(kPort);
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        bound = bind(s, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0;
        if (!bound) logf("udp: bind 127.0.0.1:%u failed (WSA error %d) - port in use?", kPort, WSAGetLastError());
        DWORD timeoutMs = 200;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeoutMs), sizeof(timeoutMs));
    } else {
        logf("udp: socket() failed (%d)", WSAGetLastError());
    }
    if (bound) logf("udp: listening on 127.0.0.1:%u", kPort);

    char buf[4096];
    rf::Frame tmp;
    std::string err;
    while (!g_stop && bound) {
        int n = recvfrom(s, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
        if (n == SOCKET_ERROR) {
            int e = WSAGetLastError();
            if (e != WSAETIMEDOUT && e != WSAECONNRESET) {
                logf("udp: recvfrom error %d", e);
                Sleep(50);
            }
            continue;
        }
        g_bytes += static_cast<uint64_t>(n);
        if (rf::parsePacket(buf, static_cast<size_t>(n), tmp, err)) {
            std::lock_guard<std::mutex> lock(g_stateMutex);
            g_frame = tmp;
            g_lastRecvTick = GetTickCount64();
            ++g_packets;
        } else {
            if (++g_bad <= 5) logf("udp: rejected packet (%s), %d bytes", err.c_str(), n);
        }
    }
    if (s != INVALID_SOCKET) closesocket(s);
    WSACleanup();
    logf("udp: thread exit");
    g_threadDone = true;
}

// ---------------------------------------------------------------- diagnostics
const char *apiName(reshade::api::device_api api) {
    using reshade::api::device_api;
    switch (api) {
        case device_api::d3d9: return "Direct3D 9";
        case device_api::d3d10: return "Direct3D 10";
        case device_api::d3d11: return "Direct3D 11";
        case device_api::d3d12: return "Direct3D 12";
        case device_api::opengl: return "OpenGL";
        case device_api::vulkan: return "Vulkan";
        default: return "unknown";
    }
}

std::atomic<ULONGLONG> g_diagUntil{0};
std::mutex g_depthMutex;
std::unordered_set<uint64_t> g_seenDepth;

void on_init_device(reshade::api::device *device) {
    logf("device created: API = %s", apiName(device->get_api()));
    if (g_users++ == 0) {
        g_stop = false;
        std::thread(udpThread).detach();  // not started from DllMain (loader lock)
    }
}

void on_destroy_device(reshade::api::device *) {
    logf("device destroyed");
    if (--g_users == 0) {
        g_stop = true;
        for (int i = 0; i < 30 && !g_threadDone; ++i) Sleep(20);
    }
}

void on_bind_render_targets_and_depth_stencil(reshade::api::command_list *cmd_list, uint32_t,
                                              const reshade::api::resource_view *, reshade::api::resource_view dsv) {
    if (dsv.handle == 0) return;
    ULONGLONG until = g_diagUntil.load();
    if (until == 0 || GetTickCount64() > until) return;
    {
        std::lock_guard<std::mutex> lock(g_depthMutex);
        if (g_seenDepth.size() >= 64 || !g_seenDepth.insert(dsv.handle).second) return;
    }
    reshade::api::device *dev = cmd_list->get_device();
    reshade::api::resource res = dev->get_resource_from_view(dsv);
    reshade::api::resource_desc d = dev->get_resource_desc(res);
    logf("depth-stencil bound: %ux%u format=%u (ReShade format enum value)", d.texture.width, d.texture.height,
         static_cast<unsigned>(d.texture.format));
}

void on_reshade_present(reshade::api::effect_runtime *runtime) {
    static uint64_t frames = 0, framesAtLastLog = 0;
    static ULONGLONG lastLog = 0, firstFrame = 0;
    static uint32_t lastW = 0, lastH = 0;
    ++frames;
    ULONGLONG now = GetTickCount64();
    if (firstFrame == 0) {
        firstFrame = now;
        g_diagUntil = now + kDiagWindowMs;
        lastLog = now;
    }

    reshade::api::resource bb = runtime->get_current_back_buffer();
    reshade::api::resource_desc d = runtime->get_device()->get_resource_desc(bb);
    if (d.texture.width != lastW || d.texture.height != lastH) {
        lastW = d.texture.width;
        lastH = d.texture.height;
        logf("back buffer: %ux%u format=%u (ReShade format enum value)", lastW, lastH, static_cast<unsigned>(d.texture.format));
    }

    if (now - lastLog < 1000) return;
    double fps = (frames - framesAtLastLog) * 1000.0 / double(now - lastLog);
    framesAtLastLog = frames;
    lastLog = now;

    rf::Frame snap;
    ULONGLONG recvTick;
    {
        std::lock_guard<std::mutex> lock(g_stateMutex);
        snap = g_frame;
        recvTick = g_lastRecvTick;
    }
    if (g_packets == 0) {
        logf("fps=%.0f  no telemetry yet (is the BeamNG mod loaded? check BeamNG.log for 'realfluid')", fps);
        return;
    }
    float maxSlip = 0, maxSide = 0;
    for (const auto &w : snap.wheels) {
        maxSlip = std::fmax(maxSlip, std::fabs(w.slip));
        maxSide = std::fmax(maxSide, std::fabs(w.sideSlip));
    }
    logf("fps=%.0f  packets=%llu bad=%llu bytes=%llu  seq=%u age=%llums  cam=(%.1f,%.1f,%.1f) fov=%.0f  wheels=%zu maxSlip=%.2f maxSide=%.2f",
         fps, (unsigned long long)g_packets.load(), (unsigned long long)g_bad.load(), (unsigned long long)g_bytes.load(),
         snap.seq, (unsigned long long)(now - recvTick), snap.camPos[0], snap.camPos[1], snap.camPos[2], snap.fovDeg,
         snap.wheels.size(), maxSlip, maxSide);
}

}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD fdwReason, LPVOID) {
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(hModule)) return FALSE;
            openLog(hModule);
            logf("RealFluid add-on milestone 1 loaded");
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
            reshade::register_event<reshade::addon_event::reshade_present>(on_reshade_present);
            reshade::register_event<reshade::addon_event::bind_render_targets_and_depth_stencil>(
                on_bind_render_targets_and_depth_stencil);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(hModule);
            break;
    }
    return TRUE;
}
