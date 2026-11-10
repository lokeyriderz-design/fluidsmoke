// RealFluid - milestone 2b (step 2): telemetry + screen-space test markers, hidden behind scene depth.
//
//   * registers with ReShade as an add-on,
//   * listens on UDP 127.0.0.1:43871 for the BeamNG Lua sender (see src/protocol.hpp),
//   * after ReShade's effects, draws a coloured disc on screen at each tire's contact
//     point (green = gripping, red = slipping). This only tests that the camera maths,
//     the telemetry and the GPU drawing line up. It is NOT the smoke yet and it does
//     hide markers behind the car/objects by reading the game's depth buffer (set depthTest=0 to turn that off),
//   * logs once per second (realfluid.log, next to the game exe, else %TEMP%).
//
// Optional realfluid.ini next to the .addon: overlay=1|0, fovVertical=1|0, markerScale=100, depthTest=1|0, depthTolCm=40
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
#include <type_traits>
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

// ---------------------------------------------------------------- scene depth tracking (2b step 2)
// Which depth texture is the main scene depth? Same idea as ReShade's own depth detection:
// among depth textures that match the screen size, the one with the most draw calls wins.
struct DepthCand {
    std::atomic<uint64_t> res{0};
    std::atomic<uint64_t> draws{0};
    std::atomic<uint32_t> state{0};  // last resource_usage the game moved it to (0 = unknown)
    std::atomic<uint32_t> w{0}, h{0}, fmt{0};
};
constexpr int kMaxCands = 8;
DepthCand g_cands[kMaxCands];
std::atomic<uint32_t> g_bbW{0}, g_bbH{0};
std::atomic<uint32_t> g_resGen{1};
thread_local uint64_t t_lastDsv = 0, t_curRes = 0;
thread_local uint32_t t_gen = 0;

DepthCand *findCand(uint64_t res) {
    if (!res) return nullptr;
    for (auto &c : g_cands)
        if (c.res.load() == res) return &c;
    return nullptr;
}

void registerCand(uint64_t res, const reshade::api::resource_desc &d) {
    if (findCand(res)) return;
    for (auto &c : g_cands) {
        uint64_t empty = 0;
        if (c.res.compare_exchange_strong(empty, res)) {
            c.draws = 0;
            c.state = 0;
            c.w = d.texture.width;
            c.h = d.texture.height;
            c.fmt = static_cast<uint32_t>(d.texture.format);
            return;
        }
    }
}

void logDepthBind(reshade::api::command_list *cmd_list, reshade::api::resource_view dsv) {
    ULONGLONG until = g_diagUntil.load();
    if (until == 0 || GetTickCount64() > until) return;
    {
        std::lock_guard<std::mutex> lock(g_depthMutex);
        if (g_seenDepth.size() >= 64 || !g_seenDepth.insert(dsv.handle).second) return;
    }
    reshade::api::device *dev = cmd_list->get_device();
    reshade::api::resource res = dev->get_resource_from_view(dsv);
    reshade::api::resource_desc d = dev->get_resource_desc(res);
    logf("depth-stencil bound: %ux%u format=%u (ReShade format enum value) usage=0x%x srvFlag=%d levels=%u samples=%u",
         d.texture.width, d.texture.height, static_cast<unsigned>(d.texture.format),
         static_cast<unsigned>(d.usage), (d.usage & reshade::api::resource_usage::shader_resource) != 0 ? 1 : 0,
         static_cast<unsigned>(d.texture.levels), static_cast<unsigned>(d.texture.samples));
}

void on_bind_render_targets_and_depth_stencil(reshade::api::command_list *cmd_list, uint32_t,
                                              const reshade::api::resource_view *, reshade::api::resource_view dsv) {
    if (dsv.handle == 0) {
        t_curRes = 0;
        return;
    }
    logDepthBind(cmd_list, dsv);
    const uint32_t gen = g_resGen.load();
    if (dsv.handle == t_lastDsv && t_gen == gen) return;  // t_curRes is still right
    t_lastDsv = dsv.handle;
    t_gen = gen;
    t_curRes = 0;
    reshade::api::device *dev = cmd_list->get_device();
    reshade::api::resource res = dev->get_resource_from_view(dsv);
    reshade::api::resource_desc d = dev->get_resource_desc(res);
    const bool fmtOk = d.texture.format == reshade::api::format::d24_unorm_s8_uint ||
                       d.texture.format == reshade::api::format::d32_float;
    if (d.type == reshade::api::resource_type::texture_2d && fmtOk && d.texture.width == g_bbW.load() &&
        d.texture.height == g_bbH.load() && g_bbW.load() != 0) {
        registerCand(res.handle, d);
        t_curRes = res.handle;
    }
}

inline void countDraw() {
    if (!t_curRes) return;
    DepthCand *c = findCand(t_curRes);
    if (c) c->draws.fetch_add(1, std::memory_order_relaxed);
}
bool on_draw(reshade::api::command_list *, uint32_t, uint32_t, uint32_t, uint32_t) {
    countDraw();
    return false;
}
bool on_draw_indexed(reshade::api::command_list *, uint32_t, uint32_t, uint32_t, int32_t, uint32_t) {
    countDraw();
    return false;
}

// The barrier event returns void in some ReShade versions and bool in others; this adapts to either.
using BarrierDecl = reshade::addon_event_traits<reshade::addon_event::barrier>::decl;
template <typename R = std::invoke_result_t<BarrierDecl *, reshade::api::command_list *, uint32_t,
                                            const reshade::api::resource *, const reshade::api::resource_usage *,
                                            const reshade::api::resource_usage *>>
R on_barrier(reshade::api::command_list *, uint32_t count, const reshade::api::resource *resources,
             const reshade::api::resource_usage *, const reshade::api::resource_usage *new_states) {
    for (uint32_t i = 0; i < count; ++i) {
        DepthCand *c = findCand(resources[i].handle);
        if (c) c->state = static_cast<uint32_t>(new_states[i]);
    }
    if constexpr (!std::is_void_v<R>) return false;
}

// 2b step 1: only OBSERVE how the game creates its depth textures. Never modifies anything.
std::atomic<int> g_createLogged{0};
bool on_create_resource(reshade::api::device *, reshade::api::resource_desc &desc, reshade::api::subresource_data *,
                        reshade::api::resource_usage) {
    if (desc.type != reshade::api::resource_type::texture_2d) return false;
    if ((desc.usage & reshade::api::resource_usage::depth_stencil) == 0) return false;
    if (g_createLogged.fetch_add(1) >= 24) return false;
    logf("depth texture created: %ux%u format=%u usage=0x%x srvFlag=%d levels=%u samples=%u",
         desc.texture.width, desc.texture.height, static_cast<unsigned>(desc.texture.format),
         static_cast<unsigned>(desc.usage), (desc.usage & reshade::api::resource_usage::shader_resource) != 0 ? 1 : 0,
         static_cast<unsigned>(desc.texture.levels), static_cast<unsigned>(desc.texture.samples));
    return false;  // false = leave the game's resource exactly as it is
}


// ---------------------------------------------------------------- overlay (milestone 2a)
std::atomic<int> g_cfgOverlay{1}, g_cfgFovVertical{1}, g_cfgScale{100};
std::atomic<uint64_t> g_overlayDraws{0};
std::atomic<int> g_lastMarkers{0};
std::atomic<bool> g_depthActive{false};

reshade::api::device *g_overlayDevice = nullptr;
reshade::api::pipeline_layout g_layout{0};
reshade::api::pipeline g_pipeline{0};
reshade::api::pipeline_layout g_layoutD{0};  // variant that also reads the scene depth texture
reshade::api::pipeline g_pipelineD{0};
reshade::api::resource g_viewRes{0};
reshade::api::resource_view g_depthView{0};
std::mutex g_viewMutex;
std::vector<std::pair<reshade::api::resource_view, uint64_t>> g_viewGrave;  // destroyed a few frames later
uint64_t g_frameNo = 0;
uint64_t g_viewFailedRes = 0;  // do not retry a depth view that already failed
reshade::api::device *g_viewDevice = nullptr;
reshade::api::format g_pipelineFormat = reshade::api::format::unknown;
bool g_pipelineFailed = false;

constexpr int kMaxMarkers = 8;
// header(4) + 8 markers x 4 + 8 view depths (2 float4) = 44 DWORDs (root signature limit is 64, one more for the depth table)
constexpr uint32_t kConstCount = 4 + 4 * kMaxMarkers + kMaxMarkers;
std::atomic<int> g_cfgDepthTest{1}, g_cfgDepthTolCm{40};

void readConfig() {
    std::string path = g_dllDir + "realfluid.ini";
    g_cfgOverlay = static_cast<int>(GetPrivateProfileIntA("realfluid", "overlay", 1, path.c_str()));
    g_cfgFovVertical = static_cast<int>(GetPrivateProfileIntA("realfluid", "fovVertical", 1, path.c_str()));
    int scale = static_cast<int>(GetPrivateProfileIntA("realfluid", "markerScale", 100, path.c_str()));
    g_cfgScale = scale < 20 ? 20 : (scale > 400 ? 400 : scale);
    g_cfgDepthTest = static_cast<int>(GetPrivateProfileIntA("realfluid", "depthTest", 1, path.c_str()));
    int tol = static_cast<int>(GetPrivateProfileIntA("realfluid", "depthTolCm", 40, path.c_str()));
    g_cfgDepthTolCm = tol < 5 ? 5 : (tol > 300 ? 300 : tol);
}

const char *kShaderSource = R"HLSL(
#ifdef USE_DEPTH
Texture2D<float> depthTex : register(t0);
#endif
// g[0] = (marker count, depth test on, tolerance m, -)
// g[1..8] = markers: xy = pixel position, z = radius px, w = slip 0..1
// g[9..10] = view depth (metres) of each marker's contact point, 4 per float4
cbuffer Params : register(b0) { float4 g[11]; };
struct VSOut { float4 pos : SV_Position; };
VSOut VSMain(uint id : SV_VertexID) {
    VSOut o;
    float2 uv = float2((id << 1) & 2, id & 2);
    o.pos = float4(uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}
float zOf(int k) { return g[9 + (k >> 2)][k & 3]; }
float4 PSMain(VSOut i) : SV_Target {
    int n = (int)g[0].x;
    float4 acc = float4(0, 0, 0, 0);          // premultiplied alpha
    float al[8];
    float maxA = 0.0;
    [unroll] for (int k = 0; k < 8; ++k) {
        al[k] = 0.0;
        if (k < n) {
            float4 m = g[1 + k];
            float d = length(i.pos.xy - m.xy);
            float disc = saturate(m.z - d + 0.5) * 0.45;
            float dot_ = saturate(3.5 - d);
            al[k] = max(disc, dot_);
            maxA = max(maxA, al[k]);
        }
    }
    if (maxA <= 0.0) return acc;
#ifdef USE_DEPTH
    if (g[0].y > 0.5) {
        uint tw, th;
        depthTex.GetDimensions(tw, th);
        int2 hi = int2((int)tw - 1, (int)th - 1);
        float dPix = depthTex.Load(int3(clamp(int2(i.pos.xy), int2(0, 0), hi), 0));
        // Depth buffer value is d = a + b / z, with a = 1 (normal) or 0 (reversed).
        // We know the true view depth z of every wheel contact point, so we calibrate b from the
        // markers: unoccluded ones give the smallest |b|, occluded ones give a larger value.
        float bMin = 1e9;
        float aSel = 1.0;
        [unroll] for (int k = 0; k < 8; ++k) {
            if (k < n) {
                float4 m = g[1 + k];
                int2 c = clamp(int2(m.xy), int2(0, 0), hi);
                float dk = depthTex.Load(int3(c, 0));
                float aS = dk > 0.5 ? 1.0 : 0.0;
                float bk = abs((dk - aS) * zOf(k));
                if (bk < bMin) { bMin = bk; aSel = aS; }
            }
        }
        if (bMin > 1e-4 && bMin < 1e8) {
            float zs = bMin / max(abs(dPix - aSel), 1e-9);   // scene view depth at this pixel
            [unroll] for (int k = 0; k < 8; ++k)
                if (k < n && zs < zOf(k) - g[0].z) al[k] = 0.0;
        }
    }
#endif
    [unroll] for (int k = 0; k < 8; ++k) {
        if (k < n) {
            float a = al[k];
            float3 col = lerp(float3(0.1, 1.0, 0.2), float3(1.0, 0.1, 0.05), saturate(g[1 + k].w));
            acc.rgb = col * a + acc.rgb * (1.0 - a);
            acc.a = a + acc.a * (1.0 - a);
        }
    }
    return acc;
}
)HLSL";

bool compileShader(const char *entry, const char *target, bool withDepth, std::vector<uint8_t> &out) {
    ID3DBlob *code = nullptr, *errs = nullptr;
    const D3D_SHADER_MACRO macros[] = {{"USE_DEPTH", "1"}, {nullptr, nullptr}};
    HRESULT hr = D3DCompile(kShaderSource, std::strlen(kShaderSource), "realfluid_overlay", withDepth ? macros : nullptr, nullptr, entry,
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

void dropDepthView() {  // caller holds g_viewMutex
    if (g_depthView.handle) g_viewGrave.emplace_back(g_depthView, g_frameNo + 6);
    g_depthView = {0};
    g_viewRes = {0};
}

void flushViewGrave(bool all) {  // caller holds g_viewMutex
    if (!g_viewDevice) {
        g_viewGrave.clear();
        return;
    }
    for (size_t i = 0; i < g_viewGrave.size();) {
        if (all || g_viewGrave[i].second <= g_frameNo) {
            g_viewDevice->destroy_resource_view(g_viewGrave[i].first);
            g_viewGrave.erase(g_viewGrave.begin() + static_cast<long>(i));
        } else {
            ++i;
        }
    }
}

void destroyOverlayPipeline() {
    if (!g_overlayDevice) return;
    {
        std::lock_guard<std::mutex> lock(g_viewMutex);
        dropDepthView();
        flushViewGrave(true);
        g_viewDevice = nullptr;
    }
    if (g_pipeline.handle) g_overlayDevice->destroy_pipeline(g_pipeline);
    if (g_layout.handle) g_overlayDevice->destroy_pipeline_layout(g_layout);
    if (g_pipelineD.handle) g_overlayDevice->destroy_pipeline(g_pipelineD);
    if (g_layoutD.handle) g_overlayDevice->destroy_pipeline_layout(g_layoutD);
    g_pipeline = {0};
    g_layout = {0};
    g_pipelineD = {0};
    g_layoutD = {0};
    g_overlayDevice = nullptr;
    g_pipelineFormat = reshade::api::format::unknown;
}

void destroyOverlayIfDevice(reshade::api::device *device) {
    if (g_overlayDevice == device) destroyOverlayPipeline();
}

bool buildOverlayPipeline(reshade::api::device *dev, reshade::api::format fmt, bool withDepth,
                          reshade::api::pipeline_layout &layout, reshade::api::pipeline &pipe) {
    using namespace reshade::api;
    std::vector<uint8_t> vs, ps;
    if (!compileShader("VSMain", "vs_5_0", withDepth, vs) || !compileShader("PSMain", "ps_5_0", withDepth, ps)) return false;

    pipeline_layout_param params[2];
    params[0].type = pipeline_layout_param_type::push_constants;
    params[0].push_constants.binding = 0;
    params[0].push_constants.dx_register_index = 0;
    params[0].push_constants.dx_register_space = 0;
    params[0].push_constants.count = kConstCount;
    params[0].push_constants.visibility = shader_stage::pixel;
    uint32_t paramCount = 1;
    if (withDepth) {
        params[1].type = pipeline_layout_param_type::push_descriptors;
        params[1].push_descriptors.binding = 0;
        params[1].push_descriptors.dx_register_index = 0;  // t0
        params[1].push_descriptors.dx_register_space = 0;
        params[1].push_descriptors.count = 1;
        params[1].push_descriptors.visibility = shader_stage::pixel;
        params[1].push_descriptors.type = descriptor_type::shader_resource_view;
        paramCount = 2;
    }
    if (!dev->create_pipeline_layout(paramCount, params, &layout)) {
        logf("overlay: create_pipeline_layout failed (depth variant=%d)", withDepth ? 1 : 0);
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
    blend.source_color_blend_factor[0] = blend_factor::one;
    blend.dest_color_blend_factor[0] = blend_factor::one_minus_source_alpha;
    blend.color_blend_op[0] = blend_op::add;
    blend.source_alpha_blend_factor[0] = blend_factor::one;
    blend.dest_alpha_blend_factor[0] = blend_factor::one_minus_source_alpha;
    blend.alpha_blend_op[0] = blend_op::add;
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
    if (!dev->create_pipeline(layout, static_cast<uint32_t>(sizeof(subs) / sizeof(subs[0])), subs, &pipe)) {
        logf("overlay: create_pipeline failed (render target format %u, depth variant=%d)", static_cast<unsigned>(fmt),
             withDepth ? 1 : 0);
        dev->destroy_pipeline_layout(layout);
        layout = {0};
        return false;
    }
    return true;
}

bool createOverlayPipeline(reshade::api::device *dev, reshade::api::format fmt) {
    destroyOverlayPipeline();
    g_pipelineFailed = true;  // stays true unless the plain pipeline below succeeds

    if (!buildOverlayPipeline(dev, fmt, false, g_layout, g_pipeline)) return false;
    g_overlayDevice = dev;
    g_pipelineFormat = fmt;
    g_pipelineFailed = false;
    logf("overlay: pipeline created OK (render target format %u)", static_cast<unsigned>(fmt));

    if (buildOverlayPipeline(dev, fmt, true, g_layoutD, g_pipelineD))
        logf("overlay: depth-aware pipeline created OK");
    else
        logf("overlay: depth-aware pipeline FAILED - markers will not be hidden behind objects");
    {
        std::lock_guard<std::mutex> lock(g_viewMutex);
        g_viewDevice = dev;
    }
    return true;
}

void on_destroy_resource(reshade::api::device *, reshade::api::resource res) {
    for (auto &c : g_cands) {
        uint64_t expected = res.handle;
        c.res.compare_exchange_strong(expected, 0);
    }
    ++g_resGen;
    std::lock_guard<std::mutex> lock(g_viewMutex);
    if (g_viewRes.handle == res.handle) dropDepthView();
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
        cb[4 + 4 * kMaxMarkers + count] = p.depth;  // view depth in metres
        ++count;
    }
    g_lastMarkers = count;
    // pick the scene depth texture (most draws this frame among screen-sized candidates)
    ++g_frameNo;
    DepthCand *best = nullptr;
    uint64_t bestDraws = 0;
    for (auto &c : g_cands) {
        if (!c.res.load()) continue;
        const uint64_t dr = c.draws.exchange(0);
        if (c.w.load() == rd.texture.width && c.h.load() == rd.texture.height && dr > bestDraws) {
            best = &c;
            bestDraws = dr;
        }
    }
    resource depthRes{0};
    resource_view depthView{0};
    uint32_t depthState = 0;
    if (g_cfgDepthTest && g_pipelineD.handle) {
        std::lock_guard<std::mutex> lock(g_viewMutex);
        flushViewGrave(false);
        if (best && bestDraws >= 20 && best->res.load() != g_viewRes.handle && best->res.load() != g_viewFailedRes) {
            dropDepthView();
            const format dfmt = static_cast<format>(best->fmt.load());
            const format vfmt = dfmt == format::d32_float ? format::r32_float : format::r24_unorm_x8_uint;
            resource_view v{0};
            const resource r{best->res.load()};
            if (dev->create_resource_view(r, resource_usage::shader_resource,
                                          resource_view_desc(resource_view_type::texture_2d, vfmt, 0, 1, 0, 1), &v)) {
                g_viewRes = r;
                g_depthView = v;
                logf("depth: using scene depth %ux%u format=%u draws=%llu (view created OK)", best->w.load(), best->h.load(),
                     static_cast<unsigned>(dfmt), (unsigned long long)bestDraws);
            } else {
                g_viewFailedRes = r.handle;
                logf("depth: create_resource_view FAILED for scene depth %ux%u format=%u - depth test off", best->w.load(),
                     best->h.load(), static_cast<unsigned>(dfmt));
            }
        }
        depthRes = g_viewRes;
        depthView = g_depthView;
        DepthCand *vc = findCand(depthRes.handle);
        if (vc) depthState = vc->state.load();
    }
    if (count == 0) return;
    const bool useDepth = depthRes.handle != 0 && depthView.handle != 0;
    cb[0] = static_cast<float>(count);
    cb[1] = useDepth ? 1.0f : 0.0f;
    cb[2] = g_cfgDepthTolCm / 100.0f;
    g_depthActive = useDepth;

    const pipeline_layout lay = useDepth ? g_layoutD : g_layout;
    cmd_list->bind_pipeline(pipeline_stage::all_graphics, useDepth ? g_pipelineD : g_pipeline);
    cmd_list->bind_render_targets_and_depth_stencil(1, &rtv);
    const viewport vp = {0.0f, 0.0f, W, H, 0.0f, 1.0f};
    cmd_list->bind_viewports(0, 1, &vp);
    const rect sc = {0, 0, static_cast<int32_t>(rd.texture.width), static_cast<int32_t>(rd.texture.height)};
    cmd_list->bind_scissor_rects(0, 1, &sc);
    const resource_usage st = depthState ? static_cast<resource_usage>(depthState) : resource_usage::depth_stencil_write;
    bool barrierDone = false;
    if (useDepth) {
        if ((st & resource_usage::shader_resource_pixel) == 0) {
            cmd_list->barrier(depthRes, st, resource_usage::shader_resource_pixel);
            barrierDone = true;
        }
        descriptor_table_update upd{};
        upd.binding = 0;
        upd.array_offset = 0;
        upd.count = 1;
        upd.type = descriptor_type::shader_resource_view;
        upd.descriptors = &depthView;
        cmd_list->push_descriptors(shader_stage::pixel, lay, 1, upd);
    }
    cmd_list->push_constants(shader_stage::pixel, lay, 0, 0, kConstCount, cb);
    cmd_list->draw(3, 1, 0, 0);
    if (barrierDone) cmd_list->barrier(depthRes, resource_usage::shader_resource_pixel, st);
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
        g_bbW = lastW;
        g_bbH = lastH;
        ++g_resGen;
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
    logf("fps=%.0f  packets=%llu bad=%llu bytes=%llu  seq=%u age=%llums  cam=(%.1f,%.1f,%.1f) fov=%.0f  wheels=%zu maxSlip=%.2f maxSide=%.2f  overlay: draws=%llu markers=%d pipeline=%s depth=%s",
         fps, (unsigned long long)g_packets.load(), (unsigned long long)g_bad.load(), (unsigned long long)g_bytes.load(),
         snap.seq, (unsigned long long)(now - recvTick), snap.camPos[0], snap.camPos[1], snap.camPos[2], snap.fovDeg,
         snap.wheels.size(), maxSlip, maxSide, (unsigned long long)g_overlayDraws.load(), g_lastMarkers.load(),
         g_pipelineFailed ? "FAILED" : (g_pipeline.handle ? "ok" : "not-created"), g_depthActive.load() ? "on" : "off");
    static int dbgTick = 0;
    if ((dbgTick++ % 10) != 0) return;
    // DEBUG: where each wheel lands under both FOV readings (assumes 1920x1080).
    const int dbgFovVert = g_cfgFovVertical.load();
    logf("  DBG camRot=(%.3f,%.3f,%.3f,%.3f) fovVertical=%d", snap.camRot[0], snap.camRot[1], snap.camRot[2], snap.camRot[3], dbgFovVert);
    for (const auto &w : snap.wheels) {
        rf::Vec3 c{w.pos[0], w.pos[1], w.pos[2] - w.radius};
        rf::Projected pv = rf::project(snap.camPos, snap.camRot, snap.fovDeg, true, 1920.f, 1080.f, c);
        rf::Projected ph = rf::project(snap.camPos, snap.camRot, snap.fovDeg, false, 1920.f, 1080.f, c);
        logf("  DBG veh=%d wheel=%d pos=(%.2f,%.2f,%.2f) r=%.2f  vertFov->(%.0f,%.0f) ok=%d  horFov->(%.0f,%.0f) ok=%d  depth=%.2f",
             w.vehId, w.wheelIdx, w.pos[0], w.pos[1], w.pos[2], w.radius, pv.px, pv.py, pv.ok ? 1 : 0, ph.px, ph.py, ph.ok ? 1 : 0, pv.depth);
    }
}

}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD fdwReason, LPVOID) {
    switch (fdwReason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(hModule)) return FALSE;
            openLog(hModule);
            logf("RealFluid add-on milestone 2b-step2 loaded");
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
            reshade::register_event<reshade::addon_event::reshade_present>(on_reshade_present);
            reshade::register_event<reshade::addon_event::reshade_finish_effects>(on_finish_effects);
            reshade::register_event<reshade::addon_event::bind_render_targets_and_depth_stencil>(
                on_bind_render_targets_and_depth_stencil);
            reshade::register_event<reshade::addon_event::create_resource>(on_create_resource);
            reshade::register_event<reshade::addon_event::destroy_resource>(on_destroy_resource);
            reshade::register_event<reshade::addon_event::draw>(on_draw);
            reshade::register_event<reshade::addon_event::draw_indexed>(on_draw_indexed);
            reshade::register_event<reshade::addon_event::barrier>(on_barrier<>);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(hModule);
            break;
    }
    return TRUE;
}
