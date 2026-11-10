// RealFluid - milestone 2a: telemetry + screen-space test markers.
//
//   * registers with ReShade as an add-on,
//   * listens on UDP 127.0.0.1:43871 for the BeamNG Lua sender (see src/protocol.hpp),
//   * after ReShade's effects, draws a coloured disc on screen at each tire's contact
//     point (green = gripping, red = slipping). This only tests that the camera maths,
//     the telemetry and the GPU drawing line up. It is NOT the smoke yet and it does
//     not test depth (markers are not hidden by objects in this build),
//   * logs once per second (realfluid.log, next to the game exe, else %TEMP%).
//
// Optional realfluid.ini next to the .addon: overlay=1|0, fovVertical=1|0, markerScale=100
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <d3dcompiler.h>

#include <reshade.hpp>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "projection.hpp"
#include "protocol.hpp"

extern "C" __declspec(dllexport) const char *NAME = "RealFluid";
extern "C" __declspec(dllexport) const char *DESCRIPTION =
    "Fluid tire smoke for BeamNG.drive (milestone 2a: telemetry + tire markers, no smoke yet).";

namespace {

constexpr unsigned short kPort = 43871;
constexpr ULONGLONG kDiagWindowMs = 20000;  // how long to watch depth-stencil binds

// ---------------------------------------------------------------- logging
std::mutex g_logMutex;
FILE *g_log = nullptr;
std::string g_dllDir;

void openLog(HMODULE module) {
    char path[MAX_PATH] = {};
    DWORD n = GetModuleFileNameA(module, path, MAX_PATH);
    while (n > 0 && path[n - 1] != '\\' && path[n - 1] != '/') --n;
    path[n] = '\0';
    g_dllDir = path;
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

void destroyOverlayIfDevice(reshade::api::device *device);  // defined with the overlay code below

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

void on_destroy_device(reshade::api::device *device) {
    logf("device destroyed");
    destroyOverlayIfDevice(device);
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


// ---------------------------------------------------------------- overlay (milestone 2a)
std::atomic<int> g_cfgOverlay{1}, g_cfgFovVertical{1}, g_cfgScale{100};
std::atomic<uint64_t> g_overlayDraws{0};
std::atomic<int> g_lastMarkers{0};

reshade::api::device *g_overlayDevice = nullptr;
reshade::api::pipeline_layout g_layout{0};
reshade::api::pipeline g_pipeline{0};
reshade::api::format g_pipelineFormat = reshade::api::format::unknown;
bool g_pipelineFailed = false;

constexpr int kMaxMarkers = 12;
constexpr uint32_t kConstCount = 4 + 4 * kMaxMarkers;  // 52 DWORDs (root constant limit is 64)

void readConfig() {
    std::string path = g_dllDir + "realfluid.ini";
    g_cfgOverlay = static_cast<int>(GetPrivateProfileIntA("realfluid", "overlay", 1, path.c_str()));
    g_cfgFovVertical = static_cast<int>(GetPrivateProfileIntA("realfluid", "fovVertical", 1, path.c_str()));
    int scale = static_cast<int>(GetPrivateProfileIntA("realfluid", "markerScale", 100, path.c_str()));
    g_cfgScale = scale < 20 ? 20 : (scale > 400 ? 400 : scale);
}

const char *kShaderSource = R"HLSL(
cbuffer Params : register(b0) { float4 g[13]; };
struct VSOut { float4 pos : SV_Position; };
VSOut VSMain(uint id : SV_VertexID) {
    VSOut o;
    float2 uv = float2((id << 1) & 2, id & 2);
    o.pos = float4(uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}
float4 PSMain(VSOut i) : SV_Target {
    float4 acc = float4(0, 0, 0, 0);          // premultiplied alpha
    int n = (int)g[0].x;
    [loop] for (int k = 0; k < n; ++k) {
        float4 m = g[1 + k];                  // xy = pixel position, z = radius px, w = slip 0..1
        float d = length(i.pos.xy - m.xy);
        float disc = saturate(m.z - d + 0.5) * 0.45;
        float dot_ = saturate(3.5 - d);
        float a = max(disc, dot_);
        float3 col = lerp(float3(0.1, 1.0, 0.2), float3(1.0, 0.1, 0.05), saturate(m.w));
        acc.rgb = col * a + acc.rgb * (1.0 - a);
        acc.a = a + acc.a * (1.0 - a);
    }
    return acc;
}
)HLSL";

bool compileShader(const char *entry, const char *target, std::vector<uint8_t> &out) {
    ID3DBlob *code = nullptr, *errs = nullptr;
    HRESULT hr = D3DCompile(kShaderSource, std::strlen(kShaderSource), "realfluid_overlay", nullptr, nullptr, entry,
                            target, D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errs);
    if (FAILED(hr) || !code) {
        logf("overlay: shader compile failed (%s): %s", entry,
             errs ? static_cast<const char *>(errs->GetBufferPointer()) : "no message");
        if (errs) errs->Release();
        if (code) code->Release();
        return false;
    }
    out.assign(static_cast<const uint8_t *>(code->GetBufferPointer()),
               static_cast<const uint8_t *>(code->GetBufferPointer()) + code->GetBufferSize());
    code->Release();
    if (errs) errs->Release();
    return true;
}

void destroyOverlayPipeline() {
    if (!g_overlayDevice) return;
    if (g_pipeline.handle) g_overlayDevice->destroy_pipeline(g_pipeline);
    if (g_layout.handle) g_overlayDevice->destroy_pipeline_layout(g_layout);
    g_pipeline = {0};
    g_layout = {0};
    g_overlayDevice = nullptr;
    g_pipelineFormat = reshade::api::format::unknown;
}

void destroyOverlayIfDevice(reshade::api::device *device) {
    if (g_overlayDevice == device) destroyOverlayPipeline();
}

bool createOverlayPipeline(reshade::api::device *dev, reshade::api::format fmt) {
    using namespace reshade::api;
    destroyOverlayPipeline();
    g_pipelineFailed = true;  // stays true unless everything below succeeds

    std::vector<uint8_t> vs, ps;
    if (!compileShader("VSMain", "vs_5_0", vs) || !compileShader("PSMain", "ps_5_0", ps)) return false;

    pipeline_layout_param param;
    param.type = pipeline_layout_param_type::push_constants;
    param.push_constants.binding = 0;
    param.push_constants.dx_register_index = 0;
    param.push_constants.dx_register_space = 0;
    param.push_constants.count = kConstCount;
    param.push_constants.visibility = shader_stage::pixel;
    if (!dev->create_pipeline_layout(1, &param, &g_layout)) {
        logf("overlay: create_pipeline_layout failed");
        return false;
    }

    shader_desc vsd{};
    vsd.code = vs.data();
    vsd.code_size = vs.size();
    shader_desc psd{};
    psd.code = ps.data();
    psd.code_size = ps.size();

    blend_desc blend{};
    blend.blend_enable[0] = true;
    blend.source_color_blend_factor = blend_factor::one;
    blend.dest_color_blend_factor = blend_factor::one_minus_source_alpha;
    blend.color_blend_op = blend_op::add;
    blend.source_alpha_blend_factor = blend_factor::one;
    blend.dest_alpha_blend_factor = blend_factor::one_minus_source_alpha;
    blend.alpha_blend_op = blend_op::add;
    blend.render_target_write_mask[0] = 0xF;

    rasterizer_desc rast{};
    rast.fill_mode = fill_mode::solid;
    rast.cull_mode = cull_mode::none;
    rast.depth_clip_enable = true;

    depth_stencil_desc ds{};
    ds.depth_enable = false;
    ds.stencil_enable = false;

    primitive_topology topo = primitive_topology::triangle_list;
    format rtFormats[1] = {fmt};

    pipeline_subobject subs[] = {
        {pipeline_subobject_type::vertex_shader, 1, &vsd},
        {pipeline_subobject_type::pixel_shader, 1, &psd},
        {pipeline_subobject_type::blend_state, 1, &blend},
        {pipeline_subobject_type::rasterizer_state, 1, &rast},
        {pipeline_subobject_type::depth_stencil_state, 1, &ds},
        {pipeline_subobject_type::primitive_topology, 1, &topo},
        {pipeline_subobject_type::render_target_formats, 1, rtFormats},
    };
    if (!dev->create_pipeline(g_layout, static_cast<uint32_t>(sizeof(subs) / sizeof(subs[0])), subs, &g_pipeline)) {
        logf("overlay: create_pipeline failed (render target format %u)", static_cast<unsigned>(fmt));
        dev->destroy_pipeline_layout(g_layout);
        g_layout = {0};
        return false;
    }
    g_overlayDevice = dev;
    g_pipelineFormat = fmt;
    g_pipelineFailed = false;
    logf("overlay: pipeline created OK (render target format %u)", static_cast<unsigned>(fmt));
    return true;
}

void on_finish_effects(reshade::api::effect_runtime *runtime, reshade::api::command_list *cmd_list,
                       reshade::api::resource_view rtv, reshade::api::resource_view) {
    using namespace reshade::api;
    if (!g_cfgOverlay || rtv.handle == 0) return;

    device *dev = runtime->get_device();
    format fmt = dev->get_resource_view_desc(rtv).format;
    if (g_overlayDevice != dev || g_pipelineFormat != fmt) {
        if (g_pipelineFailed && g_pipelineFormat == fmt) return;  // do not retry every frame
        g_pipelineFormat = fmt;
        if (!createOverlayPipeline(dev, fmt)) {
            g_pipelineFormat = fmt;  // remember the failing format
            return;
        }
    }
    if (g_pipelineFailed || !g_pipeline.handle) return;

    rf::Frame snap;
    ULONGLONG recvTick;
    {
        std::lock_guard<std::mutex> lock(g_stateMutex);
        snap = g_frame;
        recvTick = g_lastRecvTick;
    }
    if (g_packets == 0 || GetTickCount64() - recvTick > 400) { g_lastMarkers = 0; return; }

    resource_desc rd = dev->get_resource_desc(dev->get_resource_from_view(rtv));
    const float W = static_cast<float>(rd.texture.width), H = static_cast<float>(rd.texture.height);

    float cb[kConstCount] = {};
    int count = 0;
    const float scale = g_cfgScale / 100.0f;
    for (const auto &w : snap.wheels) {
        if (count >= kMaxMarkers) break;
        // contact point = wheel centre straight down by the wheel radius
        rf::Projected p = rf::project(snap.camPos, snap.camRot, snap.fovDeg, g_cfgFovVertical != 0, W, H,
                                      {w.pos[0], w.pos[1], w.pos[2] - w.radius});
        if (!p.ok || p.px < -100 || p.px > W + 100 || p.py < -100 || p.py > H + 100) continue;
        float radiusPx = w.radius * p.focalPx / p.depth * 0.6f * scale;
        radiusPx = radiusPx < 5.0f ? 5.0f : (radiusPx > 80.0f ? 80.0f : radiusPx);
        float s = std::fmax(std::fabs(w.slip), std::fabs(w.sideSlip)) / 8.0f;
        float *m = &cb[4 + 4 * count];
        m[0] = p.px; m[1] = p.py; m[2] = radiusPx; m[3] = s > 1.0f ? 1.0f : s;
        ++count;
    }
    g_lastMarkers = count;
    if (count == 0) return;
    cb[0] = static_cast<float>(count);

    cmd_list->bind_pipeline(pipeline_stage::all_graphics, g_pipeline);
    cmd_list->bind_render_targets_and_depth_stencil(1, &rtv);
    const viewport vp = {0.0f, 0.0f, W, H, 0.0f, 1.0f};
    cmd_list->bind_viewports(0, 1, &vp);
    const rect sc = {0, 0, static_cast<int32_t>(rd.texture.width), static_cast<int32_t>(rd.texture.height)};
    cmd_list->bind_scissor_rects(0, 1, &sc);
    cmd_list->push_constants(shader_stage::pixel, g_layout, 0, 0, kConstCount, cb);
    cmd_list->draw(3, 1, 0, 0);
    ++g_overlayDraws;
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
    readConfig();

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
    logf("fps=%.0f  packets=%llu bad=%llu bytes=%llu  seq=%u age=%llums  cam=(%.1f,%.1f,%.1f) fov=%.0f  wheels=%zu maxSlip=%.2f maxSide=%.2f  overlay: draws=%llu markers=%d pipeline=%s",
         fps, (unsigned long long)g_packets.load(), (unsigned long long)g_bad.load(), (unsigned long long)g_bytes.load(),
         snap.seq, (unsigned long long)(now - recvTick), snap.camPos[0], snap.camPos[1], snap.camPos[2], snap.fovDeg,
         snap.wheels.size(), maxSlip, maxSide, (unsigned long long)g_overlayDraws.load(), g_lastMarkers.load(),
         g_pipelineFailed ? "FAILED" : (g_pipeline.handle ? "ok" : "not-created"));
}

}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD fdwReason, LPVOID) {
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(hModule)) return FALSE;
            openLog(hModule);
            logf("RealFluid add-on milestone 2a loaded");
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
            reshade::register_event<reshade::addon_event::reshade_present>(on_reshade_present);
            reshade::register_event<reshade::addon_event::reshade_finish_effects>(on_finish_effects);
            reshade::register_event<reshade::addon_event::bind_render_targets_and_depth_stencil>(
                on_bind_render_targets_and_depth_stencil);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(hModule);
            break;
    }
    return TRUE;
}
