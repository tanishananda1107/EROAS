# World A Pinch-Point Recovery — Status and Known Issue

**Status**: PARTIALLY RESOLVED. The vertical-escape self-cancellation bug,
the depth-instability/near-surfacing bug (fix #4), and the
closing-corridor/dead-end trap (fix #5) are all fixed and confirmed. Fix #6
(bad-heading memory) is a clear, confirmed improvement over having no
memory at all. Its sub-refinement (gating memory-clearing on goal distance
rather than raw local displacement) is implemented but **not confirmed to
help** -- one verification run under it looked worse than fix #6 alone. The
vehicle now handles World A's obstacle clusters *safely* (bounded depth,
real motion throughout, no runaway, traps caught in ~6s instead of driving
fully into them) but a winding multi-pocket area (~y=60-68) still is not
reliably cleared. Do not treat World A as reliably completing end-to-end.

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

### 5. Frontier scan turning onto pockets that funnel shut (FIXED)

With fix #3's guard in place, the vehicle reliably found *a* wide-looking
heading and turned onto it -- but traced through a full headless log, that
heading led straight into a dead end: `gap_follow`'s own `free_beams` count
shrank cleanly and continuously as it drove forward (103 -> 88 -> 57 -> 9 ->
0 over ~15s) until it was fully boxed in again. The frontier scan measures
how open a heading looks *from the current distance*, not whether it stays
open as the vehicle gets closer -- a wide-mouthed pocket that narrows to a
dead end looks identical to a real corridor from outside. The existing
no-progress timeout only catches this *after* it's fully boxed in again,
paying the full 45s again for a trap that was visible 10+ seconds earlier.

Fixed by watching for the same signal directly during normal navigation:
track the widest free-beam run seen in the last `narrowing_trap_window`
(6s); if the current run has collapsed to a small fraction
(`narrowing_trap_ratio`, 0.35) of that recent peak while the peak itself was
large enough to look like a real opening (`narrowing_trap_min_width`, 60
beams), treat it as a closing trap and back away immediately rather than
finishing the drive into the dead end. Confirmed via log: traps now caught
in ~6s (`free_beams 89->28`, `84->18`, etc.) instead of driving fully in and
then waiting out the 45s no-progress timer.

### 6. Frontier scan deterministically re-picking the same dead end (PARTIALLY FIXED)

Even with fix #5 catching each closing trap quickly, the vehicle kept
re-triggering in the same local area (9 attempts in one run without net
progress past y=64). Root cause: "widest free run" is a poor proxy for
"real through-passage" -- a large dead-end bay is more open volume than a
narrow-but-genuine corridor, so it will *always* out-score the real path on
raw width. Picking the single global max is deterministic, so no amount of
re-sweeping or more attempts fixes this on its own; it just re-derives the
same wrong answer every time (confirmed: repeated attempts kept
re-selecting a `run=103` heading that traced, via fix #5's detector, into a
dead end every time).

Fixed by adding memory: `known_bad_headings` records the world-frame yaw of
any heading confirmed (via a subsequent trap) to be a dead end, and the
frontier scan excludes candidates within `bad_heading_tolerance` (~26 deg)
of any of them, forcing later attempts onto a different, unproven heading.
Confirmed via log: this is a real improvement over no memory -- attempts
now regularly end in "clearing N bad heading(s), real progress made"
instead of endlessly cycling with zero forward motion.

**Sub-refinement, not yet confirmed to help**: the original clear condition
(any 2.5m of raw local displacement) turned out to also be satisfiable by
wandering sideways along the same wall face without net advancing,
prematurely forgetting headings that were still valid dead ends nearby and
letting the vehicle re-discover them. Changed to require distance-to-goal
to shrink by `bad_heading_clear_progress` (3.0m) before clearing. One
verification run under this change did *not* clearly improve things: 6
attempts, zero clears, and it kept discovering additional same-width
(`run=103`) dead-end pockets rather than converging -- possibly because
requiring more sustained goal-ward progress before forgetting means bad
headings accumulate faster than the vehicle can find genuinely new
directions to try in a sonar-FOV-limited sweep. This needs its own
headless-log verification pass before being trusted; if it doesn't help on
a re-test, reverting the clear condition back to raw displacement (fix #6
without the sub-refinement) is a safe fallback -- that version was
confirmed to make real, if slow, progress.

## Known open issue: multi-pocket terrain near y=60-68 not reliably cleared

Even with fixes #4, #5, and #6, the vehicle has not been observed to get
through World A's obstacle cluster in this region and reach later waypoints
(y=97+) in any verification run so far. This looks like a winding,
concave-walled area (visually, an S-curved wall of blocks) with several
similar-width false openings close together. The mechanisms now in place
(closing-corridor detection, bad-heading exclusion) are demonstrably
working *individually* -- traps are caught fast and safely, and the vehicle
does explore genuinely different headings across attempts -- but the
combination hasn't yet been observed to fully clear this specific terrain
within the time available to verify it.

This is a materially different, *safer* failure mode than either the
original freeze or the depth runaway: the vehicle is never stuck motionless
and never leaves a safe depth band. Worth investigating next: whether the
goal-progress-gated clear condition (fix #6's sub-refinement) is actually
counterproductive here versus the simpler raw-displacement version, and
whether `bad_heading_tolerance` needs to be wider given how many
similar-width dead ends this specific area apparently has (6+ distinct
`run=103` pockets observed in one run without exhausting them).

---
*Diagnosed 2026-08-16/17 across an extended debugging session. Reproduction
steps: `ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py
world_name:=world_a gui:=false start_navigator:=true start_cbf:=true`,
watch `/rexrov2/pose_gt` after the vehicle passes y=67-68.*
