# rammp-calibration

One checkerboard session → all three camera calibrations:

1. **Intrinsics verification** — factory K/D from `camera_info`, checked by
   reprojection RMS (keep factory unless it's bad).
2. **Wrist → gripper** (eye-in-hand): `cv2.calibrateHandEye` over
   (FK pose, board-in-camera) pairs → `end_effector_link → wrist camera`.
3. **Scene → base** (eye-to-hand): the board's base-frame pose chained through
   (2), combined with the scene camera's view → `base_link → scene camera`.

The output is a `mounts.yaml` — the static-transform config that lands in the
embodiment's directory in rammp-deployments (rammp-deployments#76).

## The board

Preferred: **ChArUco** (partial views work — the wrist camera will clip a big
board at close range — and there is no orientation ambiguity):

```bash
uv run gen_board.py --out board_a2.png   # A2: 10x14 squares @40mm, 4X4_100
```

Print at 100% scale on matte paper, mount dead flat (Dibond / 6mm+ MDF —
flatness is the spec that matters most), then **caliper-measure** a run of ~10
squares and divide: the measured square and marker sizes are what feed the
solver, printers lie by a percent.

Plain checkerboard also works (`--board checker`, the default): use **odd ×
even inner corners** (e.g. 8×5) or the 180° ambiguity poisons the solve, and
every view must contain the whole grid.

Either way: clamp it. It must not move for the entire session, and the scene
camera must see it where it's clamped.

## Capture (on the cell)

Needs the `full` profile up: arm driver + planner + `arm_state_publisher`
(TF!) + both cameras.

```bash
uv run ~/.claude/skills/hardware-loop/scripts/hil.py sync
# manual: position the arm by teleop/freedrive, Enter captures each station
uv run ~/.claude/skills/hardware-loop/scripts/hil.py exec -- \
    dojoctl run --profile full -- python3 capture_calibration.py \
    --out data/session_01 --manual
# scripted: replay a station list through go_to_joint_config
...  -- python3 capture_calibration.py --out data/session_01 --poses poses.yaml
uv run ~/.claude/skills/hardware-loop/scripts/hil.py pull runs/<run-id>
```

15–20 stations, board filling 30–70% of the wrist frame, **rotation-diverse**
(30°+ apart about different axes). `poses.example.yaml` has the checklist.

## Solve (offline, anywhere)

```bash
uv run solve_calibration.py data/session_01 \
    --cols 8 --rows 5 --square-size 0.030 --out results/
```

Read the report before trusting the YAML:

- `[intrinsics] … worst < 0.8 px` — factory intrinsics fine. Higher: wrong
  resolution vs `camera_info`, mismeasured squares, or a camera worth
  recalibrating.
- `[hand-eye] park vs daniilidis` — the two solvers should agree to ~1 mm.
- `board-in-base consistency` — the real quality number: a perfect calibration
  chains every station to the identical board pose. A few mm translation std
  and sub-degree rotation is a good calibration; the arm's FK accuracy is the
  usual floor.

`results/mounts.yaml` holds `end_effector_link → wrist_camera_link` and
`base_link → scene_camera_link` (composed to the drivers' root frames when the
capture could see their TF; otherwise the optical frame, and it says so).

## Tests

```bash
uv run pytest
```

Synthetic ground-truth round trips, including an end-to-end run on rendered
checkerboard images. The capture script's ROS half can only be proven on the
cell; its pure helpers are covered.
