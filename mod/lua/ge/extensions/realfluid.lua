-- RealFluid (game-engine side), milestone 1.
-- Collects per-wheel data sent by lua/vehicle/extensions/realfluidVehicle.lua, adds the
-- camera, and forwards one UDP datagram per frame to the RealFluid ReShade add-on on
-- 127.0.0.1:43871. Format: see addon/src/protocol.hpp. Does not alter the game.
local M = {}
local logTag = 'realfluid'

local PORT = 43871
local SEND_INTERVAL_MS = 16      -- ~60 packets/s while wheels are reporting
local IDLE_INTERVAL_MS = 33      -- camera-only packets otherwise
local STALE_MS = 300             -- forget a vehicle that stopped reporting
local MAX_WHEELS = 12            -- keeps the datagram well under the MTU
local MAX_DIST = 250             -- metres from the camera
local STATUS_INTERVAL_MS = 10000

local udp = nil
local seq = 0
local lastSendMs, lastStatusMs, lastOpenTryMs = 0, 0, -100000
local sentPackets, sendErrors, lastWheelCount = 0, 0, 0
local vehicles = {}              -- [vehId] = {t, px, py, pz, vel, records}

local function nowMs() return Engine.Platform.getSystemTimeMS() end
local function fin(v)
  if v == nil or v ~= v or v == math.huge or v == -math.huge then return 0 end
  return v
end

local function openSocket()
  if udp then return true end
  if not socket then
    log('W', logTag, "socket library not available in the game-engine Lua state")
    return false
  end
  local s = socket.udp()
  s:settimeout(0)
  local ok, err = s:setpeername('127.0.0.1', PORT)
  if not ok then
    log('W', logTag, "setpeername failed: " .. tostring(err))
    s:close()
    return false
  end
  udp = s
  log('I', logTag, "UDP sender ready -> 127.0.0.1:" .. PORT)
  return true
end

-- Called from realfluidVehicle.lua through obj:queueGameEngineLua.
local function onWheels(vehId, px, py, pz, vx, vy, vz, records)
  vehicles[vehId] = {
    t = nowMs(), px = px, py = py, pz = pz,
    vel = string.format("%.2f %.2f %.2f", fin(vx), fin(vy), fin(vz)),
    records = records,
  }
end

local function onUpdate()
  local t = nowMs()
  if not udp then
    if t - lastOpenTryMs < 5000 then return end
    lastOpenTryMs = t
    if not openSocket() then return end
  end
  local interval = lastWheelCount > 0 and SEND_INTERVAL_MS or IDLE_INTERVAL_MS
  if t - lastSendMs < interval then return end
  lastSendMs = t

  if not core_camera then return end
  local cx, cy, cz = core_camera.getPositionXYZ()
  local q = core_camera.getQuat()
  local fov = core_camera.getFovDeg()
  if not cx or not q then return end

  local list = {}
  for id, v in pairs(vehicles) do
    if t - v.t > STALE_MS then
      vehicles[id] = nil
    else
      local dx, dy, dz = v.px - cx, v.py - cy, v.pz - cz
      local d2 = dx * dx + dy * dy + dz * dz
      if d2 <= MAX_DIST * MAX_DIST then list[#list + 1] = {id = id, v = v, d2 = d2} end
    end
  end
  table.sort(list, function(a, b) return a.d2 < b.d2 end)

  local lines, n = {}, 0
  for _, e in ipairs(list) do
    for rec in e.v.records:gmatch("[^;]+") do
      if n >= MAX_WHEELS then break end
      local idx, x, y, z, rest = rec:match("^(%S+) (%S+) (%S+) (%S+) (.+)$")
      if idx then
        n = n + 1
        lines[n] = "W " .. e.id .. " " .. idx .. " " .. x .. " " .. y .. " " .. z .. " " .. e.v.vel .. " " .. rest
      end
    end
  end
  lastWheelCount = n

  seq = (seq + 1) % 4000000000
  local packet = string.format("RF1 %.0f %.0f %.3f %.3f %.3f %.5f %.5f %.5f %.5f %.2f %d",
    seq, t, fin(cx), fin(cy), fin(cz), fin(q.x), fin(q.y), fin(q.z), fin(q.w), fin(fov), n)
  if n > 0 then packet = packet .. "\n" .. table.concat(lines, "\n") end

  local ok = udp:send(packet)
  if ok then sentPackets = sentPackets + 1 else sendErrors = sendErrors + 1 end

  if t - lastStatusMs > STATUS_INTERVAL_MS then
    lastStatusMs = t
    log('I', logTag, string.format("sent=%d errors=%d wheels_in_last_packet=%d", sentPackets, sendErrors, n))
  end
end

local function loadVehicleExtension(veh)
  if veh then veh:queueLuaCommand("extensions.load('realfluidVehicle')") end
end

local function onVehicleSpawned(vehId)
  local ok, err = pcall(function() loadVehicleExtension(be:getObjectByID(vehId)) end)
  if not ok then log('W', logTag, "could not attach to vehicle " .. tostring(vehId) .. ": " .. tostring(err)) end
end

-- Console helper: extensions.realfluid.reloadVehicles()
local function reloadVehicles()
  local ok, err = pcall(function()
    for _, veh in ipairs(getAllVehicles()) do loadVehicleExtension(veh) end
  end)
  if not ok then log('W', logTag, "reloadVehicles failed: " .. tostring(err)) end
end

local function onExtensionLoaded()
  log('I', logTag, "game-engine extension loaded (milestone 1)")
  openSocket()
  reloadVehicles()
end

local function onExtensionUnloaded()
  if udp then udp:close() udp = nil end
end

M.onWheels = onWheels
M.onUpdate = onUpdate
M.onVehicleSpawned = onVehicleSpawned
M.reloadVehicles = reloadVehicles
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded

return M
