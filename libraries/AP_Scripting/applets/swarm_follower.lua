-- swarm_follower.lua
--
-- Leader-follower formation flight driven by AP_SwarmMesh peer state.
--
-- Each follower reads the designated leader's global position out of its local
-- SwarmMesh peer table (populated over the mesh by heartbeats + GLOBAL_POSITION_INT),
-- applies a fixed North/East offset at a fixed formation altitude, and commands
-- that as a GUIDED target with vehicle:set_target_location(). As the leader moves,
-- the mesh propagates its new position and every follower recomputes its slot,
-- holding formation.
--
-- Readiness is derived locally rather than signalled over the mesh for now (TODO)
-- a follower only starts commanding targets once it has heard from the expected
-- number of peers AND has a fresh position fix for the leader. This stands in for
-- an explicit ready flag until the coordination fields (role/formation_slot) are
-- added to the TX path.
--
-- An explicit "engage" gate (SCR_USER6) keeps the script from fighting an
-- externally commanded takeoff: it holds off until the user/GCS has the
-- vehicle safely at altitude and sets the flag.
--
-- Configuration (standard SCR_USER parameters, so the user can set a
-- different slot per instance without having to edit the script):
--   SCR_USER1 : leader SwarmMesh sysid (P2P_SYSID of the leader)
--   SCR_USER2 : this follower's offset North of the leader, metres
--   SCR_USER3 : this follower's offset East of the leader, metres
--   SCR_USER4 : formation altitude above home, metres
--   SCR_USER5 : expected peer count for the swarm to be "ready"
--   SCR_USER6 : engage flag (0 = hold, >=1 = keep formation). Set by the user.
--
-- NOTE: Copter only. Assumes the vehicle is armed, at altitude, and in GUIDED
-- before SCR_USER6 is set.

local COPTER_GUIDED_MODE = 4
local ALT_FRAME_ABOVE_HOME = 1

local UPDATE_MS = 200          -- 5 Hz formation update
local RUN_NAME = "swarm_follower"

-- read config once at startup
local leader_sysid   = math.floor(param:get('SCR_USER1') or 0)
local offset_north_m = param:get('SCR_USER2') or 0
local offset_east_m  = param:get('SCR_USER3') or 0
local formation_alt_m = param:get('SCR_USER4') or 15
local expected_peers = math.floor(param:get('SCR_USER5') or 0)

local announced_ready = false

-- returns true once we've heard from enough peers and have a fresh leader fix
local function swarm_ready()
  if leader_sysid <= 0 then
    return false
  end
  if expected_peers > 0 and swarm:count() < expected_peers then
    return false
  end
  -- wait until leader has fresh location
  return swarm:get_peer_location(leader_sysid) ~= nil
end

function update()
  -- do nothing gate param is set
  local engaged = (param:get('SCR_USER6') or 0) >= 1
  if not arming:is_armed() or not engaged then
    announced_ready = false
    return update, UPDATE_MS
  end

  if not swarm_ready() then
    return update, UPDATE_MS
  end

  if not announced_ready then
    announced_ready = true
    gcs:send_text(6, string.format("%s: leader %d acquired, holding formation", RUN_NAME, leader_sysid))
  end

  -- only steer while in GUIDED
  if vehicle:get_mode() ~= COPTER_GUIDED_MODE then
    return update, UPDATE_MS
  end

  local leader_loc = swarm:get_peer_location(leader_sysid)
  if leader_loc then
    local target = leader_loc:copy()
    target:offset(offset_north_m, offset_east_m)              -- shift horizontally in the NE plane
    target:set_alt_m(formation_alt_m, ALT_FRAME_ABOVE_HOME)  -- hold a fixed formation altitude

    -- Velocity FF, command the slot position and the leader's velocity. Falls back to a position target if the EKF origin or the leader's velocity isn't available.
    local neu = target:get_vector_from_origin_NEU_m()        -- slot position as NEU (m) from origin
    local vel = swarm:get_peer_velocity_NED(leader_sysid)    -- leader velocity, NED m/s (nil if stale)
    if neu and vel then
      local pos_ned = Vector3f()
      pos_ned:x(neu:x())        -- North
      pos_ned:y(neu:y())        -- East
      pos_ned:z(-neu:z())       -- Down = -Up
      vehicle:set_target_posvel_NED(pos_ned, vel)
    else
      vehicle:set_target_location(target)
    end
  end

  return update, UPDATE_MS
end

gcs:send_text(6, string.format("%s loaded: leader=%d offset N=%.1f E=%.1f alt=%.1f", RUN_NAME, leader_sysid, offset_north_m, offset_east_m, formation_alt_m))

return update, UPDATE_MS
