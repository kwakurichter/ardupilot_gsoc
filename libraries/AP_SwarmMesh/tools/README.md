# AP_SwarmMesh Leader-Follower Formation Demo (SITL)

A leader + N follower ArduCopter SITL instances connect over the AP_SwarmMesh multicast mesh. Each follower reads the leaders position out of its own peer table (**populated over the decentralized mesh, not from the GCS**) and holds a fixed NED offset in `GUIDED` mode as the leader flies in a set trajectory.

## Pieces

| File | Role |
|------|------|
| `../../AP_Scripting/applets/swarm_follower.lua` | Onboard follower logic: reads `swarm:get_peer_location(leader_sysid)`, commands a fixed NED offset target with the leaders velocity FF. |
| `swarm_formation_test.py` | launches the formation, arms/takeoff, engages followers, flies the leader, logs tracks to CSV. |
| `swarm_formation_plot.py` | Static matplotlib plot of the track CSV (overhead paths + formation error). |
| `swarm_formation_viz.py` | Animated HTML replay of the CSV. |

The follower logic depends on the `swarm:` Lua binding
(`swarm:count()`, `swarm:get_peer_location(sysid)`, `swarm:get_peer_velocity_NED(sysid)`),
added in `libraries/AP_Scripting/generator/description/bindings.desc` and backed
by `AP_SwarmMesh::get_peer_location()` / `get_peer_velocity_NED()`.

## Follower parameters (SCR_USER*)

The script sets the following parameters per follower:

| Param | Meaning |
|-------|---------|
| `SCR_USER1` | Leader SwarmMesh sysid (`P2P_SYSID` of the leader) |
| `SCR_USER2` | Offset North of the leader (m) |
| `SCR_USER3` | Offset East of the leader (m) |
| `SCR_USER4` | Formation altitude above home (m) |
| `SCR_USER5` | Expected peer count for "ready" (leader + other followers) |
| `SCR_USER6` | Engage flag: 0 = hold, ≥1 = keep formation (set after takeoff so the script doesn't fight the climb) |

Readiness is determined locally (peer count + a fresh leader fix) rather than signalled over the mesh for now (TODO).

## Requirements

Build the SITL copter binary (`./waf copter`) with scripting enabled, and make sure:

```bash
pip install pymavlink matplotlib
```

## Run

```bash
# from repo root, after building copter SITL
# 4 drone diamond formation flying a box trajectory
python3 libraries/AP_SwarmMesh/tools/swarm_formation_test.py \
    --followers 4 --radius 10 --alt 15 --speedup 2 \
    --leader-path box --leader-speed 1.5 --move-time 40 --converge-wait 15 \
    --work-dir ./formation_run --csv formation_track.csv

# generate animated HTML replay (open 'formation.html' in a browsewr)
python3 libraries/AP_SwarmMesh/tools/swarm_formation_viz.py \
    formation_track.csv -o formation.html

# or a static PNG
python3 libraries/AP_SwarmMesh/tools/swarm_formation_plot.py \
    formation_track.csv -o formation.png
```

Key options: `--followers N`, `--radius` (ring radius, m), `--alt`, `--leader-path {north,L,box}`, `--leader-speed` (m/s), `--move-time` (s total, split across path legs), `--speedup` (SITL time multiplier. Note wall clock waits still apply, so the leader covers speedup × speed × time).

## Results

With velocity FF, a 4 drone diamond holds its formation to within ~0.2 m (lag bias ~0.17 m, jitter rms ~0.12 m) while the leader manoeuvres.

## Notes / next steps

- Velocity FF (`swarm:get_peer_velocity_NED`) is on by default. It falls back to a position target if the EKF origin or leader velocity is unavailable. Could add accel FF.
- Altitude is held fixed (ABOVE_HOME). Should extend to full 3D by tracking the leades altitude instead.
- Coordination fields (`role`, `formation_slot`) are not yet on the TX path. Leader identity and readiness are currently determined locally. The sender should set a readiness variable itself for robustness.
