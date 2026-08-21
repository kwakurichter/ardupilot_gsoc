# AP_SwarmMesh SITL replays

Interactive replays of the AP_SwarmMesh decentralized glyph-formation experiments (GSoC 2026).

This is an **orphan branch**: it shares no history with `master` or `pr/AP_SwarmMesh` and contains
nothing but these replay pages. It exists so the library documentation can link to a live,
interactive replay without committing megabytes of assets to the upstream pull request.

| File | Experiment |
|---|---|
| `gsoc-swarm-replay.html` | 1 leader + 56 spellers forming `GSoC` |
| `gsoc-to-cosg-replay.html` | The same fleet morphing `GSoC` → `CoSG` |
| `*-standalone.html` | Same replays wrapped in a sandboxed iframe with a strict CSP |

Every page is fully self-contained — no external scripts, styles, fonts, or network requests — so
they work opened directly from disk or served over HTTP.

## Viewing

Served by GitHub Pages, if enabled for this repository:

```
https://kwakurichter.github.io/ardupilot_gsoc/
```

Otherwise any raw-HTML proxy renders them without Pages:

```
https://raw.githack.com/kwakurichter/ardupilot_gsoc/gh-pages/gsoc-swarm-replay.html
```

Opening the file locally works too — `open gsoc-swarm-replay.html`.

## Results

**`GSoC` formation:** 57/57 airborne, 56/56 cells occupied, 0.02 m median cell error (0.05 m worst),
944 s to form, 2.03 m minimum sampled separation, zero overlaps below 1 m across 15,028,995 pair
samples.

**`GSoC` → `CoSG` morph:** all 56 cells reoccupied 668 s after the leader changed the active task.

Method, control barrier function derivation, and limitations are documented in
[`tools/Glyph_Formation_Experiment.md`](https://github.com/kwakurichter/ardupilot_gsoc/blob/pr/AP_SwarmMesh/libraries/AP_SwarmMesh/tools/Glyph_Formation_Experiment.md)
on the `pr/AP_SwarmMesh` branch.

## Related

- Library source and docs: [`libraries/AP_SwarmMesh`](https://github.com/kwakurichter/ardupilot_gsoc/tree/pr/AP_SwarmMesh/libraries/AP_SwarmMesh)
- Upstream pull request: [ArduPilot#33881](https://github.com/ArduPilot/ardupilot/pull/33881)
