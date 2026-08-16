# World A Pinch-Point Recovery — Status and Known Issue

**Status**: PARTIALLY RESOLVED. The vertical-escape self-cancellation bug and
the depth-instability/near-surfacing bug (see fix #4 below) are both fixed
and confirmed. The pinch-clearing problem at World A's tightest obstacle
cluster (~y=67-68) is still open: the vehicle now handles it *safely*
(bounded depth, real motion, no runaway) but does not reliably get past it.
Do not treat World A as reliably completing end-to-end.

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

### 4. Depth instability / unbounded climb toward the surface (FIXED)

Fix #1 above (gating the depth-reached check on min duration/distance) fixed
the instant self-cancellation, but exposed a different bug: once gated, that
check compares `abs(current_z - target_z)`, and a real climb moves
`current_z` monotonically *away* from `target_z` (the original waypoint
depth) — so the check almost never re-satisfies during an actual climb, and
nothing else was bounding it except the 180s timer. Confirmed via headless
log capture: a single escape ran for the reproduced duration climbing
continuously from -50 all the way to **-14.7**, nearly reaching the surface,
with no sign of stopping before it was manually killed.

Root cause: the exit condition was checking the wrong thing. The paper's
actual intent (Sec III-C1 / Fig 8b) is "climb until you can see a way
through again," not "climb back to a specific depth" or "climb for a fixed
duration." Fixed by adding `_has_horizontal_gap` (reuses the same
gap-detection logic as normal `gap_follow`) and ending the escape as soon as
a real horizontal corridor reopens — this is both the correct success
signal and an inherent safety bound, since it stops climbing the moment
it's no longer needed rather than relying on a duration ceiling.

Confirmed via log after the fix: escapes now end in 4-6.5s, with `z`
staying within ~0.1m of the original target depth (`z=-49.92`, `z=-49.94`)
— nowhere near the earlier near-surface excursion.

## Known open issue: pinch at ~y=67-68 not reliably cleared

With fix #4 in place, re-running the same reproduction no longer produces
any dangerous depth excursion, but the vehicle still does not reliably get
past World A's tightest obstacle cluster. Observed behavior post-fix: real,
healthy motion the whole time (`measured_planar` 0.4-0.5 m/s, `h` staying
high/safe, `nearest_obstacle` 4-5m -- not boxed in or frozen), vertical
escapes ending quickly and safely, stuck-recovery finding and turning onto
wide corridors (`run=103` beams) -- but net position keeps returning to the
same ~1-2m pocket around (28-29, 67-68) rather than continuing toward
y=97+.

This is a materially different, *safer* failure mode than either the
original freeze or the depth runaway: the vehicle is never stuck motionless
and never leaves a safe depth band, it just hasn't found the way through
this specific cluster yet. Worth investigating next: whether the "widest
corridor" headings the frontier scan keeps finding are genuinely being
followed through to escape, or whether something downstream (spatial
memory re-accumulating around the vehicle as it re-approaches, or the
corridor closing again once it's no longer viewed edge-on) is pulling it
back before it clears the cluster.

---
*Diagnosed 2026-08-16/17 across an extended debugging session. Reproduction
steps: `ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py
world_name:=world_a gui:=false start_navigator:=true start_cbf:=true`,
watch `/rexrov2/pose_gt` after the vehicle passes y=67-68.*
