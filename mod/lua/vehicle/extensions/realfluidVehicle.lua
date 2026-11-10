-- RealFluid (vehicle side), milestone 1.
-- Reads wheel data every GFX step, throttled to 30 Hz, and hands it to the game-engine
-- extension 'realfluid' (obj:queueGameEngineLua). Read-only: never touches physics.
local M = {}
local logTag = 'realfluid'
local abs, max, floor, fmt = math.abs, math.max, math.floor, string.format

-- Milestone 1: send even when parked so the whole chain can be verified without
-- driving. Set to false once it works (then only sends while tires are slipping).
local ALWAYS_SEND = true
local SEND_INTERVAL = 1 / 30
local SLIP_THRESHOLD = 0.2   -- m/s; only used when ALWAYS_SEND is false
local HOLD_TIME = 0.5        -- keep sending this long after slip stops

local timer, hold = 0, 0
local disabled = false
local sanityLogged = false

local function fin(v)
  if v == nil or v ~= v or v == math.huge or v == -math.huge then return 0 end
  return v
end

local function collect()
  local px, py, pz = obj:getPositionXYZ()
  local vx, vy, vz = obj:getVelocityXYZ()
  local records, n, maxSlip = {}, 0, 0
  local firstOffset
  for i, wd in pairs(wheels.wheels) do
    if not wd.isBroken then
      local p1 = obj:getNodePosition(wd.node1)
      local p2 = obj:getNodePosition(wd.node2)
      local ox, oy, oz = (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5
      local slip, side = fin(wd.lastSlip), fin(wd.lastSideSlip)
      maxSlip = max(maxSlip, abs(slip), abs(side))
      if not firstOffset then firstOffset = math.sqrt(ox * ox + oy * oy + oz * oz) end
      n = n + 1
      records[n] = fmt("%d %.3f %.3f %.3f %.3f %.3f %.3f %.1f %.1f %d %d",
        floor(i), px + ox, py + oy, pz + oz,
        fin(wd.dynamicRadius or wd.radius or 0.3), slip, side,
        fin(wd.slipEnergy), fin(wd.downForce),
        floor(fin(wd.contactMaterialID1)), floor(fin(wd.contactMaterialID2)))
    end
  end
  return {px = px, py = py, pz = pz, vx = vx, vy = vy, vz = vz,
          payload = table.concat(records, ";"), n = n, maxSlip = maxSlip, firstOffset = firstOffset}
end

local function step()
  local c = collect()
  if c.n == 0 then return end
  if not sanityLogged then
    sanityLogged = true
    -- If wheel centres are not within a few metres of the vehicle origin, the
    -- assumption "obj:getNodePosition is relative to obj:getPosition" is wrong.
    log('I', logTag, fmt("vehicle %s: %d wheels, wheel0 is %.2f m from vehicle origin", tostring(objectId), c.n, c.firstOffset or -1))
    if (c.firstOffset or 0) > 6 then
      log('W', logTag, "wheel offset looks too large - getNodePosition may not be relative; tell the developer")
    end
  end
  if c.maxSlip > SLIP_THRESHOLD then hold = HOLD_TIME else hold = hold - SEND_INTERVAL end
  if ALWAYS_SEND or hold > 0 then
    obj:queueGameEngineLua(fmt("realfluid.onWheels(%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,'%s')",
      objectId, c.px, c.py, c.pz, fin(c.vx), fin(c.vy), fin(c.vz), c.payload))
  end
end

local function updateGFX(dt)
  if disabled then return end
  timer = timer + dt
  if timer < SEND_INTERVAL then return end
  timer = 0
  local ok, err = pcall(step)
  if not ok then
    disabled = true
    log('E', logTag, "vehicle sender disabled after error: " .. tostring(err))
  end
end

local function onExtensionLoaded()
  log('I', logTag, "vehicle extension loaded for vehicle " .. tostring(objectId))
end

M.updateGFX = updateGFX
M.onExtensionLoaded = onExtensionLoaded

return M
