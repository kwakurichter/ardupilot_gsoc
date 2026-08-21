# AP_SwarmMesh SITL replays

Interactive replays of the AP_SwarmMesh SITL experiments (GSoC 2026).

This is an **orphan branch**: it shares no history with `master` or `pr/AP_SwarmMesh` and contains
nothing but these replay pages. It exists so the library documentation can link to a live,
interactive replay without committing megabytes of assets to the upstream pull request.

| File | Experiment |
|---|---|
| `gsoc-swarm-replay.html` | 1 leader + 56 spellers forming `GSoC` |
| `gsoc-to-cosg-replay.html` | The same fleet morphing `GSoC` → `CoSG` |
| `formation-40-followers.html` | Leader–follower formation, 40 followers |
| `formation-253-followers.html` | Leader–follower formation, the full sysid range |

Every page is a complete, self-contained HTML document — no external scripts, styles, fonts, or
network requests — so each works served over HTTP or opened directly from disk.

## Viewing

Served by GitHub Pages, if enabled for this repository:

```
https://kwakurichter.github.io/ardupilot_gsoc/
```

Otherwise any raw-HTML proxy renders them without Pages:

```
https://raw.githack.com/kwakurichter/ardupilot_gsoc/gh-pages/gsoc-swarm-replay.html
```

Opening a file locally works too — `open gsoc-swarm-replay.html`.

The glyph replays initialise a large dataset before painting their first frame, so allow a second
or two on load before the canvas fills in.

## Results

**`GSoC` glyph formation.** 57/57 airborne, 56/56 cells occupied, 0.02 m median cell error (0.05 m
worst), 944 s to form, 2.03 m minimum sampled separation, zero overlaps below 1 m across 15,028,995
pair samples.

**`GSoC` → `CoSG` morph.** All 56 cells reoccupied 668 s after the leader changed the active task,
with no reassignment traffic and no central planner — every vehicle independently recomputes which
cell it owns.

**Leader–follower formation.** 40 followers: 100% airborne and tracking, 0.36 m median formation
error. 253 followers: 253/254 airborne, 236/252 tracking (94%), 0.87 m median error, 5.3 m p90.

Method, control barrier function derivation, and limitations for the glyph experiments are in
[`tools/Glyph_Formation_Experiment.md`](https://github.com/kwakurichter/ardupilot_gsoc/blob/pr/AP_SwarmMesh/libraries/AP_SwarmMesh/tools/Glyph_Formation_Experiment.md);
the scaling work behind the 253-follower run is in
[`tools/SCALING.md`](https://github.com/kwakurichter/ardupilot_gsoc/blob/pr/AP_SwarmMesh/libraries/AP_SwarmMesh/tools/SCALING.md).

## Related

- Library source and docs: [`libraries/AP_SwarmMesh`](https://github.com/kwakurichter/ardupilot_gsoc/tree/pr/AP_SwarmMesh/libraries/AP_SwarmMesh)
- Upstream pull request: [ArduPilot#33881](https://github.com/ArduPilot/ardupilot/pull/33881)
