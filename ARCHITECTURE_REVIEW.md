# EROAS1_ros22 Architecture Review

## 1. File Comparison

### velocity_cbf.py (1709 lines) — PRIMARY navigation node
**Status:** Running in production as `/obstacle_avoidance_node`

**Already implemented:**
- FLS/sonar point cloud processing (`pc_callback`, `_update_point_cloud_obstacles`)
- SCG integration — subscribes to all SCG topics:
  - `/rexrov2/scg/h` (barrier function)
  - `/rexrov2/scg/selected_gap_angle`
  - `/rexrov2/scg/selected_gap_width`
  - `/rexrov2/scg/obstacle_count`
  - `/rexrov2/scg/gap_count`
- Spatial memory with voxelization and pruning
- Analytical obstacle boxes for World A
- Full CBF safety filter with QP solver (active-set enumeration + iterative projection fallback)
- XY and XZ CBF constraint computation
- Recovery behavior:
  - Stall detection (near-zero velocity timeout)
  - Largest free gap heading computation (`_largest_free_gap_heading`)
  - Stable recovery heading with deadband/hysteresis
  - Safety escape commands when h < 0
  - Hover lock for squeeze/oscillation scenarios
- Depth hold (XZ plane control)
- Slew rate limiting
- Minimum forward speed enforcement
- Planner command integration (subscribes to `/rexrov2/cmd_vel_1`)
- Final velocity publisher to `/rexrov2/cmd_vel`
- Thruster input monitoring
- PID output monitoring
- Extensive debug logging with tags: `[CBF]`, `[RECOVERY]`, `[HOVER_LOCK]`, `[ESCAPE]`, `[SPATIAL_MEM]`

**Missing from EROAS pipeline:**
- ❌ SCG computation (relies on external `/sonar_heading_node` to publish SCG topics)
- ❌ Gap Finding analysis (done externally in `sonar_heading_node.py`)
- ❌ Boundedness analysis
- ❌ Convergence analysis
- ❌ SPD2C planner (relies on external planner command on `/rexrov2/cmd_vel_1`)
- ❌ Debug log tags: `[FLS_SENSOR]`, `[SCG]`, `[GAP_FINDING]`, `[BOUNDEDNESS]`, `[CONVERGENCE]`, `[SPD2C]`, `[PID]`, `[THRUSTER_COMMANDS]`, `[REXROV_DYNAMICS]`

---

### sonar_heading_node.py (142 lines) — SCG/gap-finding node
**Status:** Running as `/sonar_heading_node`

**Already implemented:**
- Subscribes to `/rexrov2/blueview_p900_point_cloud` (sonar point cloud)
- Subscribes to `/rexrov2/odom` (vehicle state)
- Gap finding from sonar data (angular binning, gap detection)
- Publishes SCG topics:
  - `/rexrov2/scg/h` — barrier function value
  - `/rexrov2/scg/obstacle_count`
  - `/rexrov2/scg/gap_count`
  - `/rexrov2/scg/selected_gap_angle`
  - `/rexrov2/scg/selected_gap_width`
- Publishes heading command to `/rexrov2/cmd_vel_1`
- Basic gap selection (largest gap center)

**Missing from EROAS pipeline:**
- ❌ Boundedness analysis
- ❌ Convergence analysis
- ❌ SPD2C (only publishes a simple heading command, not a full SPD2C planner)
- ❌ Multi-obstacle handling (1, 2, 3, 4+ obstacles with different strategies)
- ❌ Safety margin gap rejection

---

### eroas_navigation_node.py (424 lines) — INCOMPLETE new node
**Status:** NOT running, NOT launched, has syntax errors

**Implemented (but incomplete):**
- Constructor with publishers/subscribers
- Odometry callback
- Point cloud callback
- `spd2c()` method (basic)
- `cbf_safety_filter()` method (basic — much simpler than velocity_cbf.py's QP solver)
- `pid_controller()` method (basic)
- `_publish_thruster_cmd()` method
- `_check_recovery()` method
- `main()` function

**Broken/Missing:**
- ❌ `_sonar_cb()` — empty body, causes SyntaxError
- ❌ `_control_loop()` — calls undefined methods (`_fls_sensor_process`, `_scg_pipeline`, `_spd2c_planner`, `_cbf_safety_filter`, `_pid_controller`)
- ❌ No spatial memory
- ❌ No analytical obstacle boxes
- ❌ No QP-based CBF solver (only simple projection)
- ❌ No recovery heading computation
- ❌ No hover lock
- ❌ No slew rate limiting
- ❌ References undefined attributes (`self.pos`, `self.vel`, `self.recovery_mode`)

---

## 2. Architecture Recommendation

### ✅ RECOMMENDED: Option A — Extend velocity_cbf.py

**Rationale:**

| Criterion | velocity_cbf.py | eroas_navigation_node.py |
|-----------|----------------|-------------------------|
| Lines of working code | 1709 (production) | 424 (broken) |
| CBF solver | Full QP with active-set enumeration | Simple single-constraint projection |
| Recovery behavior | Complete (stall, hover lock, escape, gap heading) | Stub (references undefined methods) |
| Spatial memory | Voxelized with pruning and max size | None |
| Analytical obstacles | World A boxes defined | None |
| Currently running | Yes (`/obstacle_avoidance_node`) | No |
| Launch file integration | Yes | No |
| Topic wiring | Complete | Partial |

`velocity_cbf.py` has ~1700 lines of **tested, running** code. `eroas_navigation_node.py` has ~424 lines of **broken, incomplete** code that duplicates a fraction of what velocity_cbf.py already does, and does it worse (no QP solver, no spatial memory, no recovery).

Replacing velocity_cbf.py with eroas_navigation_node.py would mean rewriting 1300+ lines of working logic. Extending velocity_cbf.py means adding ~200-400 lines of new pipeline stages.

---

## 3. Migration Plan

### What already works (KEEP):
| Component | File | Status |
|-----------|------|--------|
| FLS point cloud processing | velocity_cbf.py | ✅ Working |
| Spatial memory | velocity_cbf.py | ✅ Working |
| Analytical obstacle boxes | velocity_cbf.py | ✅ Working |
| CBF QP solver | velocity_cbf.py | ✅ Working |
| Recovery (stall, hover lock, escape) | velocity_cbf.py | ✅ Working |
| Depth hold | velocity_cbf.py | ✅ Working |
| Slew rate limiting | velocity_cbf.py | ✅ Working |
| SCG topic publishing | sonar_heading_node.py | ✅ Working |
| Gap finding (basic) | sonar_heading_node.py | ✅ Working |
| Final cmd_vel publisher | velocity_cbf.py | ✅ Working |
| Thruster manager integration | velocity_cbf.py → thruster_allocator | ✅ Working |

### What is missing (ADD to velocity_cbf.py):
| Component | Description | Estimated Lines |
|-----------|-------------|----------------|
| Boundedness analysis | Check if selected gap is bounded on one/both sides | ~40-60 |
| Convergence analysis | Check if gap is closing as vehicle approaches | ~40-60 |
| SPD2C planner | Internal planner using SCG output to generate desired velocity/yaw | ~80-120 |
| Enhanced gap finding | Multi-obstacle strategies (1/2/3/4+ obstacles) | ~60-80 |
| EROAS debug log tags | `[FLS_SENSOR]`, `[SCG]`, `[GAP_FINDING]`, `[BOUNDEDNESS]`, `[CONVERGENCE]`, `[SPD2C]`, `[PID]`, `[THRUSTER_COMMANDS]`, `[REXROV_DYNAMICS]` | ~30-50 |
| Recovery enhancement | Rotate toward largest free sector when CBF outputs near-zero | Already partially done |

### What should be DELETED:
| File | Reason |
|------|--------|
| `eroas_navigation_node.py` | Incomplete, broken, duplicates velocity_cbf.py poorly. Delete after migration complete. |

### What should be MERGED:
| From | To | What |
|------|----|------|
| sonar_heading_node.py gap logic | velocity_cbf.py (or keep as separate node) | SCG gap finding can stay in sonar_heading_node.py since it already publishes to SCG topics that velocity_cbf.py subscribes to. Add boundedness/convergence either in sonar_heading_node.py or velocity_cbf.py. |

### Recommended merge strategy:
- **Keep sonar_heading_node.py** as the SCG front-end (it processes raw sonar → gaps → publishes SCG topics)
- **Enhance sonar_heading_node.py** with boundedness and convergence analysis (publish results on new topics)
- **Add SPD2C to velocity_cbf.py** as an internal planner that uses SCG data to generate desired velocity
- **Keep external planner command** (`/rexrov2/cmd_vel_1`) as optional fallback
- **Add EROAS debug log tags** to velocity_cbf.py's control loop

---

## 4. Topic & Publisher Verification

### cmd_vel publishers (CONFLICT CHECK):
| Node | Topic | Status |
|------|-------|--------|
| velocity_cbf.py (`/obstacle_avoidance_node`) | `/rexrov2/cmd_vel` | ✅ Active, primary |
| sonar_heading_node.py (`/sonar_heading_node`) | `/rexrov2/cmd_vel_1` | ✅ Active, planner reference |
| eroas_navigation_node.py | `/rexrov2/cmd_vel` | ❌ NOT running (would conflict!) |

**Result:** Currently NO conflict. If eroas_navigation_node.py were launched, it would create a SECOND publisher on `/rexrov2/cmd_vel` — this confirms it should NOT be launched alongside velocity_cbf.py.

### Navigation nodes (should be exactly 2):
| Node | Role |
|------|------|
| `/obstacle_avoidance_node` (velocity_cbf.py) | CBF filter + final velocity publisher |
| `/sonar_heading_node` (sonar_heading_node.py) | SCG + gap finding + planner reference |

### Launch file dependencies:
- `start_eroas_demo_world_a.launch.py` launches both `velocity_cbf.py` and `sonar_heading_node.py`
- No launch entry exists for `eroas_navigation_node.py`

### Topic dependency chain:
```
/rexrov2/blueview_p900_point_cloud  →  sonar_heading_node  →  /rexrov2/scg/* topics
                                                            →  /rexrov2/cmd_vel_1
/rexrov2/point_cloud                →  velocity_cbf.py     (point cloud processing)
/rexrov2/scg/*                      →  velocity_cbf.py     (SCG data consumption)
/rexrov2/cmd_vel_1                  →  velocity_cbf.py     (planner command)
/rexrov2/odom                       →  velocity_cbf.py     (vehicle state)
/rexrov2/sonar/moving               →  velocity_cbf.py     (sonar state)
velocity_cbf.py                     →  /rexrov2/cmd_vel    (final safe velocity)
/rexrov2/cmd_vel                    →  velocity_control     (PID)
velocity_control                    →  thruster_allocator   (thruster commands)
thruster_allocator                  →  thruster plugins     (physical thrusters)
```

---

## 5. Dependency Diagram: EROAS Pipeline → Actual Codebase

```
┌─────────────────────────────────────────────────────────────────────┐
│                        EROAS PIPELINE                               │
│                                                                     │
│  [FLS SENSOR]                                                       │
│      │  Sonar point cloud from Gazebo plugin                        │
│      │  Topic: /rexrov2/blueview_p900_point_cloud                   │
│      │  File: nps_uw_multibeam_sonar (Gazebo plugin)                │
│      ▼                                                              │
│  [SCG: Spatial Context Generator]                                   │
│      │  File: sonar_heading_node.py (142 lines)                     │
│      │  Node: /sonar_heading_node                                   │
│      │  Publishes: /rexrov2/scg/h, obstacle_count, gap_count,       │
│      │             selected_gap_angle, selected_gap_width            │
│      ▼                                                              │
│  [GAP FINDING]                                                      │
│      │  File: sonar_heading_node.py (inside sonar callback)         │
│      │  Status: ✅ Basic implementation exists                      │
│      │  Missing: Multi-obstacle strategies, safety margin rejection  │
│      ▼                                                              │
│  [BOUNDEDNESS ANALYSIS]  ← ❌ NOT IMPLEMENTED ANYWHERE              │
│      │                                                              │
│      ▼                                                              │
│  [CONVERGENCE ANALYSIS]  ← ❌ NOT IMPLEMENTED ANYWHERE              │
│      │                                                              │
│      ▼                                                              │
│  [SPD2C PLANNER]         ← ❌ NOT IMPLEMENTED                       │
│      │  Currently: sonar_heading_node publishes simple heading cmd   │
│      │  to /rexrov2/cmd_vel_1 (not a full SPD2C)                    │
│      │  Needed: velocity_cbf.py internal SPD2C using SCG data       │
│      ▼                                                              │
│  [CBF SAFETY FILTER]                                                │
│      │  File: velocity_cbf.py (1709 lines)                          │
│      │  Node: /obstacle_avoidance_node                              │
│      │  Status: ✅ Full QP solver, spatial memory, recovery         │
│      │  Publishes safe velocity                                     │
│      ▼                                                              │
│  [PID CONTROLLER]                                                   │
│      │  File: External — /rexrov2/velocity_control node             │
│      │  Subscribes: /rexrov2/cmd_vel                                │
│      │  Publishes: /rexrov2/thruster_manager/input                  │
│      │  Status: ✅ Already running                                  │
│      ▼                                                              │
│  [THRUSTER COMMANDS]                                                │
│      │  File: External — /rexrov2/thruster_allocator node           │
│      │  Subscribes: /rexrov2/thruster_manager/input                 │
│      │  Publishes: /rexrov2/thrusters/thruster_N/input              │
│      │  Status: ✅ Already running                                  │
│      ▼                                                              │
│  [REXROV DYNAMICS]                                                  │
│      │  Gazebo physics simulation                                   │
│      │  Publishes: /rexrov2/odom, /rexrov2/pose_gt                  │
│      │  Status: ✅ Running in Gazebo                                │
│      │                                                              │
│      └──── State feedback to SCG, SPD2C, CBF, PID ────┘            │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. Implementation Plan (After Review Approval)

### Phase 1: Enhance sonar_heading_node.py (~100 lines)
1. Add boundedness analysis after gap finding
2. Add convergence analysis after boundedness
3. Publish boundedness/convergence results on new topics
4. Improve multi-obstacle gap selection (1/2/3/4+ strategies)
5. Add safety margin gap rejection

### Phase 2: Add SPD2C to velocity_cbf.py (~150 lines)
1. Add `_spd2c_planner()` method that uses SCG data (h, gap angle, gap width, obstacle count)
2. Generate desired forward velocity and yaw rate toward selected gap
3. Keep external `/rexrov2/cmd_vel_1` as fallback when SCG data is stale
4. SPD2C becomes the primary decision-maker; CBF remains safety filter only

### Phase 3: Add EROAS debug logging to velocity_cbf.py (~50 lines)
1. Add `[FLS_SENSOR]`, `[SCG]`, `[GAP_FINDING]`, `[BOUNDEDNESS]`, `[CONVERGENCE]`, `[SPD2C]` log tags
2. Add `[PID]`, `[THRUSTER_COMMANDS]`, `[REXROV_DYNAMICS]` log tags
3. Log all required values each cycle

### Phase 4: Enhance recovery in velocity_cbf.py (~30 lines)
1. When CBF outputs near-zero velocity, rotate toward largest free sector
2. Zero velocity only for emergency collision distance

### Phase 5: Delete eroas_navigation_node.py
1. Remove the incomplete file
2. No launch file changes needed (it was never launched)

### Phase 6: Build, launch, verify
1. `colcon build --symlink-install`
2. Launch World A
3. Verify pipeline end-to-end

---

## Summary

**Decision: Option A — Extend velocity_cbf.py**  ✅ COMPLETED

The full production velocity_cbf.py (2494 lines) from EROAS1_ros2 has been
merged into EROAS1_ros22.  It includes:

- Full EROAS pipeline: FLS → SCG → Gap Finding → Boundedness → Convergence → SPD2C → ST-CBF → PID → Thrusters
- Internal local SCG with polar obstacle map, gap detection, and boundedness/convergence analysis
- SPD2C planner generating reference velocity from gap data
- QP-based CBF safety filter (active-set enumeration + iterative projection fallback)
- Spatial memory with voxelisation and time/distance pruning
- Analytical obstacle boxes for World A
- Full recovery: stall detection, barrier sliding, gap-seeking rotation, hover lock, free-space unstick
- Depth hold with slew limiting
- All EROAS debug log tags: [FLS_SENSOR], [SCG], [GAP_FINDING], [BOUNDEDNESS], [CONVERGENCE], [SPD2C], [CBF], [RECOVERY], [HOVER_CONTROL]

Deleted: `eroas_navigation_node.py.bak` (broken, incomplete, never launched).
