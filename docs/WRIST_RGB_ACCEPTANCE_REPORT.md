# Wrist RGB Acceptance — Outside-palm remount

| Field | Value |
|---|---|
| Status | RGB gate **4/4** on frozen `H_knuckle_z05`; portfolio launch default **on** |
| Date | 2026-08-14 |
| Previous freeze | `B_look_fingers` `pos="0 0 -0.02"` — RGB 0/4 (inside `hand_0`) |
| New freeze | `H_knuckle_z05` `pos="0 0 0.05"` look +Z_hand |
| Physical | `NOT_RUN/UNAVAILABLE` |
| Hand-eye | **not** done |
| Live ROS | idle RGB bag round **2** (`evidence/live_rgb_bag_20260814T062618Z/`); no teleop trajectory |
| Models merged | **false** |

## A. Why remount

GT projection 4/4 on B did not mean pixels could see the cube. RGB was a grey
disk: the camera sat inside the palm mesh. Behind-palm offsets (E/F/G) still
looked *through* `hand_0` and stayed at 0 red pixels.

## B. RGB selection (not GT-only)

Candidates scored on 4 static postures with the same `VirtualCamera` as
`camera_bridge`. Metric: `rgb_visible`, then `red_pixel_sum`, then GT.

| id | rgb | gt | red sum |
|---|---|---|---|
| **H_knuckle_z05** | **4** | 4 | 1237 |
| I_dorsal_knuckle | 2 | 4 | 806 |
| B_look_fingers | 0 | 4 | 0 |
| E/F/G behind palm | 0 | 4 | 0 |

Frozen XML:

```xml
<camera name="wrist_camera" pos="0.0 0.0 0.05" xyaxes="1 0 0 0 -1 0" fovy="70.0" />
```

Pose class: **`DESIGN_NOMINAL`**. Not `PHYSICAL_CALIBRATED`.

## C. Acceptance after freeze

`python3 -m teleop_diagnostics.wrist_rgb_cli --out-dir evidence/wrist_rgb_acceptance`

| Check | Result |
|---|---|
| GT projection 4/4 | true |
| RGB red-pixels 4/4 | **true** |
| Stage 3C `T_hand_camera` stable | true |
| `enable_wrist_camera` launch default | **true** (portfolio) |

SmolVLA / batch-preflight scripts that pass `enable_wrist_camera:=false` are
unchanged (no new VLA collection).

## D. Caveats

- The four postures are approximate joint poses, not IK onto the red cube.
  The cube is in frame; the green sphere can sit nearer the image center.
- Wrist XML freeze is `H_knuckle_z05` (`DESIGN_NOMINAL`). Do not remount for
  scene-visual work.
- Non-target poses and lighting diffuse now copy over `/sim/scene_visual`
  (two MjModels remain). Mass/friction is still not copied.
- Live ROS (2026-08-14) **round 2**: `publish_depth:=false`.
  `evidence/live_rgb_bag_20260814T062618Z/bag/rgb` — 15.4 s, scene 153, wrist 153,
  `/sim/object_pose` 1438, `/sim/scene_visual` 1438. PNGs from the bag:
  `png/scene.png`, `png/wrist.png` (wrist red_pixels=289). Physics log:
  `Loaded MuJoCo model` / `mujoco_sim up (MuJoCo)`. Non-target example:
  blue cylinder `y≈0.214` vs XML rest `0.10`; `lights.top.diffuse≈0.729`.
- Round 1 (valid but shorter): `evidence/live_rgb_bag_20260814T061324Z/` (9 s, 88/86 RGB).
- Fallback attempt `evidence/live_rgb_bag_20260814T060945Z/` remains **superseded**.
- Recorder 0-frame root cause: `_on_frame` dropped when `/teleop/cmd_pose`
  and `/teleop/gripper_cmd` were unseen (`start_teleop:=false`). Teleop/portfolio
  now hold-fills from `/ee_pose` + `/gripper/state` and tags `command_missing` /
  `action_fill=hold_from_ee` (**not** expert command). Live verify:
  `evidence/live_rgb_episode_20260814T063527Z/` **76 frames**. `batch_generator`
  still drops command-missing frames.
- Idle ready pose only; not a teleop grasp trajectory.

## E. Regenerate

```bash
export PYTHONPATH=src/mujoco_sim:src/camera_bridge:src/teleop_diagnostics:$PYTHONPATH
python3 -m teleop_diagnostics.wrist_rgb_cli --tune-outside-palm --out-dir evidence/wrist_rgb_tune_outside_palm
python3 -m teleop_diagnostics.wrist_rgb_cli --out-dir evidence/wrist_rgb_acceptance
```
