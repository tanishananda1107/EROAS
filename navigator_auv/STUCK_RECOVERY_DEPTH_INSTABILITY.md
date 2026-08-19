# World A Pinch-Point Recovery — Status and Known Issue

**Status**: PARTIALLY RESOLVED, one new open issue found. The vertical-escape
self-cancellation bug, the depth-instability/near-surfacing bug (fix #4),
and the closing-corridor/dead-end trap (fix #5) are all fixed and
confirmed. Fix #6 (bad-heading memory) is a clear, confirmed improvement
over having no memory at all; its goal-distance-gated clearing
sub-refinement is unverified (see fix #6). Fix #7 (excluding bad headings
from gap_follow itself, not just stuck-recovery's frontier scan) is
confirmed to break the vehicle out of the local multi-pocket cycling that
trapped it in every prior run -- but revealed a new problem: once loose in
open water, the same blacklist can prevent it turning back toward the goal
if that heading overlaps one blacklisted earlier, and the vehicle was
observed travelling 60+m away from the obstacle cluster without clearly
recovering toward the goal. Fix #8 gives `known_bad_headings` positional
scoping to address that specific mechanism and is now **confirmed via a
15-minute headless run**: bounded excursions (x within [0.6, 44.5], no
60+m runaway) and real net goal-ward progress (dist_to_goal 62.2m ->
33.8m at one point), reaching y=77.7 -- further than any prior run. The
vehicle now handles World A's obstacle clusters *safely* (bounded depth,
real motion throughout, no runaway) and makes real progress, but still
does not reliably reach the goal -- the same run was still cycling through
stuck-recovery, short of the first waypoint (y=97), when the 900s test
window ended. Do not treat World A as reliably completing end-to-end.

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

### 7. gap_follow itself re-targeting confirmed dead ends (FIXED, revealed a new issue)

The user visually traced the actual required route around this cluster: it
goes around the *outside* of the whole structure (a wide detour), not
through any gap within it. That explains why fix #6 alone wasn't enough:
`known_bad_headings` only stopped *stuck-recovery's* frontier scan from
re-selecting a dead end -- normal `gap_follow` (which always targets
whichever candidate is closest to the goal bearing) was never filtered, so
the instant the vehicle backed off even slightly from a dead end, the very
next cycle it would re-spot that same direction looking marginally open
again and cut straight back into it. The two mechanisms were fighting each
other, which is why bad headings kept accumulating (6+, all `run=103`)
without the vehicle ever committing to a detour long enough to get around.

Fixed by applying the same `known_bad_headings` exclusion to `gap_follow`'s
own candidate selection (filtering `mid_beams` by world-frame bearing
before picking the goal-nearest one), not just stuck-recovery's frontier
scan. **Confirmed via a 15-minute isolated log verification**: this broke
the vehicle out of the local y=56-64 cycling that trapped it in every prior
run -- it explored genuinely further (both laterally and in y) than any
previous attempt.

**But it surfaced a new problem, also confirmed in that same run**: once
loose of the immediate cluster, the vehicle continued diverging further
away (observed 60+m net displacement, x reaching -30 against a spawn x of
~29 and a goal x of ~29) without clearly turning back toward the goal. The
likely mechanism: `known_bad_headings` is a pure world-frame yaw blacklist
with no expiry and no positional scoping, so once the vehicle has travelled
far enough that the correct goal-ward heading now points through empty
water in roughly the same *direction* as a heading blacklisted earlier in
the same episode (different place, coincidentally similar bearing), the
filter still excludes it -- blocking the vehicle from turning back even
though that specific direction is no longer actually blocked by anything.

### 8. `known_bad_headings` blocking goal-ward turns far from where it was recorded (FIXED, not yet re-verified)

Implemented the fix predicted at the end of fix #7: `known_bad_headings`
entries are now recorded as `(yaw, x, y)` at the vehicle's position when the
dead end was confirmed, not just a bare yaw. `_is_known_bad_heading` takes
the vehicle's *current* position and only excludes a candidate if it's
within `bad_heading_position_radius` (new parameter, default 20.0m) of
where that heading was recorded, in addition to the existing yaw-tolerance
check. Both call sites (`_update_stuck_recovery`'s frontier scan and
`gap_follow`'s own `mid_beams` filter) now pass the vehicle's current `(x,
y)`. This directly targets the fix #7 regression: a heading recorded near
one obstacle cluster no longer suppresses a coincidentally-similar,
genuinely-clear bearing once the vehicle has moved on to different terrain.

**Confirmed via a 15-minute headless run** (2026-08-19, `world_a`, spawn
(29,33,-54), goal-ward waypoint at y=97):

- Real, sustained goal-ward progress, not just local wandering:
  dist_to_goal shrank 62.2m -> 58.1m -> ... -> 33.8m across the run (two
  `known_bad_headings` clears logged with those numbers). The vehicle
  reached y=77.7 (max), well past the y=56-68 cluster that trapped every
  prior run, and further than any previous attempt.
- No runaway divergence: x stayed within [0.6, 44.5] the whole run --
  bounded detouring around the obstacle cluster, not the unbounded 60+m
  drift (x reaching -30 against spawn x=29) that fix #7 alone produced.
  Depth stayed within ~0.3m of the -50 target throughout; no errors or
  crashes in 52k+ log lines.
- **Not fully resolved**: the vehicle was still in `stuck_recovery_backing_away`
  when the run's 900s timeout hit (attempt 10, 8 bad headings accumulated
  since the last clear at attempt 3). It did not reach y=97 or any later
  waypoint. Bad-heading accumulation resumed after attempt 3's clear and
  didn't clear again through attempt 10 -- consistent with the vehicle
  working through a harder stretch of the course later on, not yet
  understood. A longer run (and/or comparing against a fix-#7-only run
  under the same seed) would help isolate how much of the improvement is
  attributable to fix #8 specifically vs. run-to-run variance.

Net: fix #8 is a real, verified improvement over the fix #7 baseline
(bounded excursions, confirmed net goal-ward progress) but does not by
itself make World A complete reliably. Still open: whether fix #6's
goal-progress-gated clearing still helps now that positional scoping does
most of that job; whether `bad_heading_position_radius` (20.0m, a first
guess) needs tuning; and what's actually blocking progress past y~77-97.

## Known open issue: vehicle does not reliably reach the goal

With fixes #4-#7, the vehicle no longer gets permanently stuck or dives
unsafely, and it does break out of the local obstacle cluster that trapped
every prior run -- but it has not been observed reaching later waypoints
(y=97+) or the goal. Fix #8 (above) targets the specific mechanism behind
that but is unverified. Also still open: whether fix #6's
goal-progress-gated clearing helps or hurts (unverified), and whether
`bad_heading_tolerance` needs tuning for how many similar-width dead ends
this specific area has.

---
*Diagnosed 2026-08-16/17 across an extended debugging session. Reproduction
steps: `ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py
world_name:=world_a gui:=false start_navigator:=true start_cbf:=true`,
watch `/rexrov2/pose_gt` after the vehicle passes y=67-68.*
