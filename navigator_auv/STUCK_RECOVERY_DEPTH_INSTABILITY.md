# World A Pinch-Point Recovery — Status and Known Issue

**Status**: PARTIALLY RESOLVED. The vertical-escape self-cancellation bug is
fixed and confirmed. The frontier-scan stuck-recovery added on top of it gets
the vehicle further than before but has a confirmed, unresolved depth
instability. Do not treat World A as reliably completing end-to-end.

## What was fixed (confirmed via headless log capture, not just code review)

### 1. Vertical escape self-cancellation (`only_gap.py`, `_process_data_paper`)

`vertical_escape_active`'s exit check compared current depth to
`target_z` (the current waypoint's depth) with no minimum-duration guard.
Since a horizontal pinch blocks the vehicle at the *same* depth it's already
holding, that check was satisfied on the very first tick after "beginning
escape", so the climb produced ~0m of net elevation change before cancelling
itself and falling through to `gap_follow` on stale SCG defaults, driving
straight back into the same obstacle.

Fix: gated the exit check on `vertical_escape_min_duration` /
`vertical_escape_min_planar_distance` (already-declared parameters that were
never actually read). Confirmed via log: escape now sustains real climbs
(observed `current_z` moving multiple meters, not the previous ~0).

### 2. Fully-boxed deadlock (new `_update_stuck_recovery`)

At a sufficiently tight obstacle cluster, the horizontal scan can report
`free_beams=0` on every bearing at once. `velocity_cbf.py`'s min-norm CBF-QP
then has an obstacle-derived constraint pushing back from every direction,
so whatever `only_gap.py` tries gets projected toward zero velocity — no 2D
or vertical-pivot decision resolves this because the corridor those
maneuvers look for may not exist from the vehicle's current position within
the current safety margin.

Fix: track real odometry displacement independent of decision state; after
`stuck_recovery_timeout` (45s) with under `stuck_recovery_distance_threshold`
(2.5m) of net movement, abandon the current maneuver and reverse straight
back — reversing is the one direction the CBF's constraint gradient
`2*(vehicle - obstacle)` always treats as safe, so it passes through
un-throttled. Confirmed via log: real clearance gains (nearest_obstacle
2.05m -> 3.9m -> 4.4m across attempts).

### 3. Backing out just re-approached the same corridor

Reversing alone doesn't help if `gap_follow`/`convexity_turn` just re-aims
at the goal afterward — confirmed: net position unchanged two minutes after
a successful back-away. Added a data-driven fix: while backing away, also
yaw and continuously sample the sonar's widest contiguous free run at each
heading (`_widest_free_run`); at the end of the back-away, turn onto the
best heading seen before resuming navigation, instead of guessing.

**Sub-bug found and fixed during this**: the very first post-trigger sample
(heading barely changed from the stuck heading) reported the full scan
width as free. This is not a real opening — backing away increases standoff
distance from the *same* wall, which alone can push a reading from
"blocked" to "reads free" at longer range. Turning back onto it just
re-approached the identical spot. Fixed by requiring
`yaw_delta >= 0.6rad` from the heading at trigger time before a candidate is
trusted (`stuck_recovery_start_yaw`).

Confirmed via log: with the guard, the vehicle got past y=68 in World A —
the exact position it had been permanently stuck at in every prior run.

## Known open issue: depth instability after clearing the pinch

In the same run that got past y=68, the vehicle became unstable shortly
after: y oscillated (68 -> 60 -> 55 -> 61 -> 68) and depth excursed far
below target — observed `current_z` reaching **-60.5** against a -50.0
target (10.5m off), well beyond the -50 to -55 range typical elsewhere in
the run. This is a new, more concerning failure mode than the pre-fix
"frozen in place" behavior — the vehicle is not stationary, but it is not
under good control either.

**Not yet root-caused.** Suspected: repeated `vertical_pivot`/
`vertical_escape` triggers further along the course (a second obstacle
cluster was encountered) interacting with the new stuck-recovery logic
across multiple cycles, possibly compounding depth error each cycle. Needs:
a full headless run with logs captured from spawn through the second
cluster, correlating `HOVER_CONTROL`/`VERTICAL_PIVOT`/`STUCK_RECOVERY` log
lines against the depth excursion to find the actual trigger sequence.

**Before extending this further**: reproduce headless with full logging,
find the exact cycle sequence that produces the dive, and consider whether
`vertical_escape_duration` (180s in World A) needs an absolute depth-limit
safety clamp independent of the target-depth-tracking logic in fix #1 above
— that fix removed the *only* thing that was stopping an escape from
running indefinitely once `reached_target_depth` stops being trivially
satisfiable, and the 180s duration alone is not a tight enough bound if the
climb rate is fast.

---
*Diagnosed 2026-08-16/17 across an extended debugging session. Reproduction
steps: `ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py
world_name:=world_a gui:=false start_navigator:=true start_cbf:=true`,
watch `/rexrov2/pose_gt` for depth excursions after the vehicle passes
y=67-68.*
