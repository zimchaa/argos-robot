# ARGOS — Roadmap and Forward Plan

## Current state (Session 3 complete)

Phase 1 is done. Hardware is partially verified. The safety layer is live.

| Item | Status |
|------|--------|
| Drivers (I2CMotor, GPIOMotor) | Done |
| Controllers (TrackedBase, RobotArm) | Done |
| SafetyMonitor (speed clamping + watchdog) | Done |
| Central config (MotorConfig, pin mappings) | Done |
| Tracks verified on hardware | Done |
| Elbow, wrist, gripper verified on hardware | Done |
| Shoulder — motor 4 fault | Under investigation |
| Track drift (right slower than left) | Known, deferred |

---

## Immediate hardware procurement

These are blocking or near-blocking items for the next build phase.

| Priority | Item | Purpose | Notes | Est. cost |
|----------|------|---------|-------|-----------|
| 1 | USB webcam (720p+) | Arm vision / ArUco detection | OpenCV `VideoCapture`. Ball-head mount for angle adjustment. 640×480 sufficient; 720p preferred. | ~£15 |
| 2 | Camera boom bracket | Mount camera lateral to arm | See design options below | £0–5 |
| 3 | ArUco marker sheet (printed) | Joint pose tracking | Print `DICT_4X4_50` markers on paper, laminate or use self-adhesive label stock. Sizes: 30/25/20/15 mm per link. | ~£0 |
| 4 | MPU-6050 IMU | Heading tracking + tilt correction for floor-plane | I2C at 0x68 — no conflict with 0x40. Plugs into Waveshare I2C expansion header (5-pin P?). Driver lives in `sensorium/imu.py`. | ~£3 |
| 5 | HC-SR04 ultrasonic sensor | Obstacle detection (safety before autonomous nav) | Voltage divider already fitted on MotorShield CN10. Plug straight in. TRIG=BOARD 29, ECHO=BOARD 31. | ~£2 |

---

## Camera boom — design options

The camera must sit ~30–40 cm to one side of the robot, at shoulder-to-wrist midpoint height
(~130–140 mm above the deck), pointing inward at the arm face-on.

See `docs/camera_mount.md` for FOV calculations and ArUco marker sizing.

### Option A — Aluminium angle bracket (recommended first prototype)

**Material:** 25×25 mm aluminium angle, ~350 mm long. Available from any hardware shop or eBay.

**Attachment to chassis:** M3 bolt + nut through the yellow track guard lip, or a hose clamp
around the side rail. No drilling required for the hose clamp approach.

**Camera mount:** 1/4"-20 threaded insert or nut epoxied or press-fitted into the bracket end.
Standard webcam tripod mount is 1/4"-20.

**Pros:** No printing. Can be built in under an hour with parts on hand.
**Cons:** Less elegant. Will need adjustment once FOV is verified.

**Key dimensions:**
```
chassis side rail
      │
      ├──── 25×25 mm angle, horizontal ───────────────┤
      │     ~340 mm projection                        📷
      │                               ↑ height: ~130 mm above deck
```

### Option B — 3D-printed clip-on boom (longer term)

**Design brief for CAD:**
- Clips onto the yellow track guard lip (snap-fit or M3 bolt). Measure guard lip width before printing.
- Horizontal arm: ~340 mm projection from chassis centreline to camera face.
- Camera height above clip: adjust so lens is at shoulder-to-wrist midpoint (~130 mm above deck).
- Camera attachment: 1/4"-20 brass insert, printed in.
- Material: PETG or ASA preferred (outdoors-capable, heat-resistant for Pi enclosure proximity).
- Weight budget: arm should not torque the clip noticeably. Keep arm tube hollow/ribbed.

**Recommended workflow:**
1. Build and verify Option A first — confirm FOV, angle, and height empirically.
2. Use those measurements to design the printed bracket with confidence.
3. Print and replace once FOV is validated.

### Verification steps (both options)

1. Mount camera, connect to Pi USB.
2. Run `python3 -c "import cv2; cap = cv2.VideoCapture(0); ret,f = cap.read(); print(ret, f.shape)"` — confirm capture.
3. Place a printed ArUco marker at arm shoulder distance. Confirm detection in OpenCV.
4. Move arm through range of motion. Confirm all 4 link markers remain in frame.
5. If any marker leaves frame: increase standoff distance or adjust camera angle.

---

## Software phases

### Phase 2a — Vision (next)

**Goal:** read current arm pose from camera.

1. `argos/vision/camera.py` — `Camera` class wrapping `cv2.VideoCapture`. Frame capture, JPEG encode.
2. `argos/vision/aruco.py` — `ArucoTracker` class. Detect markers, call `cv2.solvePnP` per marker, compute joint angles from adjacent marker relative poses.
3. `argos/vision/calibration/` — camera calibration routine (checkerboard), store intrinsic matrix + distortion as numpy `.npy` files.
4. Update `SafetyMonitor` to accept a pose-provider callback — plumbing for Phase 2c.

**Dependency to add:** `opencv-python` (or `opencv-python-headless` on the Pi).

**Test:** verify joint angle readout against manually measured angles on hardware.

---

### Phase 2b — Kinematics

**Goal:** `move_arm(x, y, z)` → joint angles → motor commands.

1. `argos/arm/kinematics.py`:
   - Constants: L2=9.0 cm, L3=11.3 cm, L4=10.5 cm, shoulder_height=9.0 cm.
   - `forward_kinematics(shoulder_deg, elbow_deg, wrist_deg)` → `(r, h)` end-effector.
   - `inverse_kinematics(r, h)` → list of `(shoulder, elbow, wrist)` candidates.
   - Algorithm: iterative shoulder sweep (0°→180°, 5° steps) + analytic elbow/wrist (Debenec).
   - See `docs/owi535_ik_reference.md` for the full algorithm.

2. Execution loop in `SafetyMonitor` or a new `ArmPlanner`:
   - Get current joint angles from `ArucoTracker`.
   - Run IK, select optimal candidate (minimise `max(Δθ)` from current pose).
   - Drive joints at configured speed, poll pose, stop when within tolerance.

**Test:** open-loop accuracy target ~1.2 cm (paper baseline). Visual feedback should improve this.

---

### Phase 2c — Planner (base + arm coordination)

**Goal:** `move_arm(x, y, z, grip)` achieves any reachable target in 3D space by
coordinating base movement and arm IK together, running entirely on the Pi.

#### The workspace insight

The arm is a 2D planar manipulator — it can only reach points in the vertical plane
aligned with the robot's forward axis. On its own this is a severe limitation.
With the tracked base underneath it, that plane can be aimed at any horizontal
direction and positioned at any standoff distance. The combined reachable workspace
becomes: **any point where height ≤ ~39 cm** (shoulder height + full arm reach ~30 cm).
The base's job is to get the target into the arm's plane at the right distance; the
arm's job is to reach it in 2D.

#### Coordinate system

All planner coordinates are **robot-relative**:

```
Y (up)
│
│    Z (forward — arm plane)
│   /
│  /
└──────── X (right)

Origin: robot centre at ground level.
Shoulder pivot: (0, shoulder_height, shoulder_forward_offset) in this frame.
```

The LLM calls `move_arm(x, y, z, grip)` with robot-relative coordinates.
If the camera identifies a target object in the scene, its position is expressed
in this frame using the ArUco reference frame.

#### Goal decomposition

```
Given target (x, y, z) in robot frame:

1. TURN — rotate base to face target
   heading_error = atan2(x, z)
   if |heading_error| > threshold:
       turn_base(heading_error)          ← IMU feedback for accuracy
   # After turn: target is now directly ahead (x ≈ 0, z = sqrt(x²+z²))

2. DRIVE — adjust standoff distance
   r_target       = sqrt(x² + z²)       ← horizontal distance to target
   r_arm          = r_target − shoulder_forward_offset
   [r_min, r_max] = IK feasible envelope (~3–30 cm from shoulder)
   if r_arm < r_min: move_base_forward(r_min − r_arm)   ← back up
   if r_arm > r_max: move_base_forward(r_arm − r_max)   ← drive in

3. SOLVE — 2D inverse kinematics
   (θ_shoulder, θ_elbow, θ_wrist) = ik.solve(r_arm, y)
   Select optimal candidate: minimise max(Δθ) from current pose

4. REACH — closed-loop arm execution
   loop until all joints within tolerance OR timeout:
       current = aruco.get_joint_angles()
       for each joint:
           error = target_angle − current_angle
           if |error| > tolerance:
               speed = clamp(error × Kp, min_speed, max_speed)
               safety.arm[joint].run(±speed)
           else:
               safety.arm[joint].stop()
   safety.arm.stop()

5. GRIP
   safety.arm.gripper.run(close_speed if grip else open_speed)
   sleep(gripper_duration)
   safety.arm.gripper.stop()
```

Steps 1–2 use dead reckoning (IMU for turns, time-based for distance).
Step 4 uses ArUco visual feedback — this corrects any residual positioning error
from steps 1–2. The arm's servo loop is the error corrector for the base's imprecision.

#### Fallback behaviour

- **ArUco markers not visible**: fall back to open-loop time-based execution,
  capped at a short duration. Log a warning.
- **IK has no solution**: return error to caller immediately, do not move.
- **Safety watchdog fires during execution**: the plan aborts. Report which motor
  timed out. Caller (MCP tool) returns an error string.
- **Tolerance not reached within timeout**: stop all joints, return partial success
  with the actual final pose from ArUco.

#### Module structure

```
argos/
  planner/
    __init__.py
    goal.py        # Goal dataclass: target (x,y,z), grip, tolerance, timeout
    decompose.py   # world goal → (base_steps, arm_target_angles)
    executor.py    # execute a decomposed plan; feedback loops for base + arm
```

The MCP tool becomes a thin wrapper:

```python
@mcp.tool()
def move_arm(x: float, y: float, z: float, grip: bool) -> str:
    goal   = Goal(position=(x, y, z), grip=grip)
    plan   = decompose(goal, shoulder_offset=SHOULDER_OFFSET)
    result = executor.run(plan, safety, sensorium)
    return result.status   # "ok" | "ik_no_solution" | "timeout" | "safety_stop"
```

#### Open values to calibrate on hardware

| Parameter | Description | How to measure |
|-----------|-------------|----------------|
| `shoulder_forward_offset` | Distance from robot centre to shoulder pivot along Z | Measure on hardware |
| `r_min` | Minimum IK-feasible arm reach | Test IK solver with link lengths |
| `Kp` | Proportional gain for joint servo loop | Tune empirically — start low (~0.3) |
| `tolerance_deg` | Joint angle error threshold to declare "reached" | Start at 3°, tighten if ArUco is stable |
| `base_velocity` | cm/s at a given track speed (for dead reckoning) | Characterise on a measured surface |

---

### Phase 2d — Safety upgrade (vision-based)

**Goal:** stop arm automatically if any joint approaches its mechanical limit.

- `SafetyMonitor` watchdog currently uses time only.
- Add a second watchdog thread that reads joint angles from `ArucoTracker` and calls `emergency_stop()` if any joint exceeds its limit.
- Limits: shoulder 180°, elbow 260°, wrist 100° (measured values from `docs/owi535_ik_reference.md`).
- The time-based watchdog stays as a fallback for when the camera can't see the arm (occlusion).

---

### Phase 2e — Sensorium (coarse-to-fine target acquisition)

**Goal:** give the planner a way to find a target in 3D space without needing the
LLM to supply exact coordinates — starting rough from camera bearing alone and
converging to a confident position estimate as the robot closes the distance.

#### The problem with monocular depth

A single camera cannot directly measure depth. The robot compensates by:

1. **Floor-plane homography** — the most useful technique for ground-level targets.
   Given the camera's known height, tilt, and intrinsic matrix, any pixel
   representing a floor point can be ray-cast to the ground plane `y=0`, giving
   an XZ position estimate in robot frame without stereo. The IMU roll/pitch
   corrects for robot pitch on uneven terrain. Works well at 0.5–2 m range.

2. **Sonar range + camera bearing** — once the HC-SR04 is in range (~1 m),
   combining its range reading with the camera's horizontal bearing to the target
   gives a good polar → cartesian position. No floor-plane assumption needed.

3. **ArUco marker on target** (optional, highest confidence) — if the target object
   carries an ArUco marker, `solvePnP` gives its full 6-DOF pose directly from a
   single frame. Sub-centimetre accurate, no estimation required. Useful when the
   environment is under control (lab / workshop setting).

4. **Parallax from motion** — fallback when the above fail. Drive a small lateral
   arc; observe how the target shifts in frame. Geometry gives depth. Slower but
   works on any visible target.

Neural monocular depth (MiDaS, Depth-Anything) is explicitly **not used** — too
slow on Pi 4 CPU (~1–2 fps) and unnecessary given sonar + floor-plane coverage.

#### Coarse-to-fine engagement model

Each sensor engages at the range where it is actually useful:

```
Distance to target   Active sensors            What the planner gets
──────────────────   ────────────────────────  ──────────────────────────────
> 1 m                Camera only               Bearing (horizontal angle).
                                               Floor-plane: rough XZ if target
                                               is on ground. Confidence: LOW.
                                               Action: turn to face, approach.

20 cm – 1 m          Camera + Sonar            Range + bearing → XZ position.
                                               IMU heading stabilises estimate.
                                               Confidence: MEDIUM.
                                               Action: refine approach, solve IK.

5 – 30 cm            Camera + Sonar + IR       IR confirms proximity. Sonar
                                               range now accurate at short dist.
                                               ArUco on target (if present):
                                               full 6-DOF, highest confidence.
                                               Confidence: HIGH.
                                               Action: hand off to planner.

< 30 cm (arm range)  ArUco on arm links        Closed-loop visual servo.
                                               Planner executes IK + servo loop.
```

The planner does not need a precise position upfront. It receives a `TargetEstimate`
with a confidence value and approaches iteratively until confidence is high enough
to commit to an IK solution:

```
approach_target(hint):
    estimate = sensorium.estimate_target(hint)
    while estimate.confidence < THRESHOLD_TO_PLAN:
        turn_to(estimate.bearing)          # keep target centred in frame
        move_base_forward(small_step)      # close distance
        estimate = sensorium.update()
    plan = decompose(estimate.position)    # now confident enough for IK
    executor.run(plan)
```

#### The Sensorium class

A single background object owns all sensors, runs a continuous update loop at
each sensor's natural rate, and exposes a fused state to everything above it.
All other modules read from it — nothing queries hardware directly.

```python
class Sensorium:
    """
    Owns all environment sensors. Runs continuous background update loops.
    Exposes a fused estimate of robot state and target position.

    Sensor update rates:
      Camera + ArUco : ~20 fps  (one thread, one core)
      IMU (MPU-6050) : 50 Hz    (I2C polling, negligible CPU)
      Sonar (HC-SR04): 10 Hz    (GPIO timing, negligible CPU)
      IR × 2         : 20 Hz    (GPIO read, negligible CPU)
    """

    def get_state(self) -> RobotState:
        # heading, arm_pose, obstacle_near, active sensor set

    def estimate_target(self, hint=None) -> TargetEstimate:
        # position (x,y,z), bearing, range, confidence, method used
```

Fusion priority (highest confidence wins):
1. ArUco on target → full 6-DOF via `solvePnP`
2. Sonar range + camera bearing → polar → cartesian
3. Floor-plane homography → XZ from camera + IMU tilt
4. Camera bearing only → direction, no depth

#### Pi 4 feasibility

| Task | CPU cost | Core assignment |
|------|----------|-----------------|
| ArUco detection @ 640×480 | ~50 ms/frame (20 fps) | Core 0 |
| Floor-plane estimation | < 2 ms (numpy ray-cast) | Core 0 (same frame) |
| IMU polling @ 50 Hz | negligible (I2C) | Core 1 |
| Sonar ranging @ 10 Hz | negligible (GPIO) | Core 1 |
| IR polling @ 20 Hz | negligible (GPIO) | Core 1 |
| Fusion update @ 10 Hz | < 1 ms | Core 1 |
| Safety watchdog | already running | Core 2 |
| Planner / MCP | on demand | Core 3 |

Entirely feasible. The Pi 4 has 4 cores; camera owns one, all lightweight sensors
share another, safety and planner take the rest.

#### Module structure

```
argos/
  sensorium/
    __init__.py
    fusion.py       # Sensorium — background update loop, fused state
    floor_plane.py  # Camera + IMU → XZ ground-plane position estimate
    imu.py          # MPU-6050 driver (I2C 0x68, Waveshare expansion header)
    sonar.py        # HC-SR04 driver (TRIG=BOARD 29, ECHO=BOARD 31, divider fitted)
    ir.py           # IR proximity drivers (CN8=BOARD 12, CN9=BOARD 7)
    target.py       # TargetEstimate dataclass + confidence model
```

`argos/vision/` handles camera capture and ArUco detection — `Sensorium` imports
from it rather than duplicating. `Sensorium` is the integration layer.

#### Calibration required (one-time, on hardware)

| Parameter | Purpose | Method |
|-----------|---------|--------|
| Camera intrinsics | All vision | Checkerboard + `cv2.calibrateCamera` |
| Camera height + tilt | Floor-plane homography | Measure physically, verify with known-distance floor target |
| IMU gyro bias | Heading drift correction | Average gyro reading at rest over 5 s |
| Sonar beam characterisation | Confidence at angle | Move known object laterally, note range falloff |
| IR trigger distance | Confidence threshold | Move known object toward sensor, note activation distance |

---

### Phase 3 — MCP server

**Goal:** expose the robot to a remote LLM via MCP over the network.

1. `argos/mcp/server.py` — tool definitions using Anthropic `mcp` package.
2. `argos/mcp/__main__.py` — entry point: `python -m argos.mcp`.
3. Transport: SSE (network), so the LLM client can be on a separate machine.

**Tools:**

| Tool | Args | Returns |
|------|------|---------|
| `get_camera_frame` | — | base64 JPEG |
| `get_arm_pose` | — | joint angles + end-effector xyz |
| `move_arm` | `x, y, z, grip` | success/error |
| `move_base_forward` | `cm: float` | success/error |
| `turn_base` | `degrees: float` | success/error |
| `stop_all` | — | — |

All motion tools route through `SafetyMonitor` — safety layer always has authority.

---

### Phase 4 — Shoulder fault investigation

The shoulder (motor 4, MotorShield terminal 4) produced no movement during Session 2.

**Checklist:**
- [ ] Confirm motor 4 terminal wiring: MotorShield terminal 4 → shoulder motor leads
- [ ] Check for loose connector at terminal block
- [ ] Test motor in isolation: apply 6V directly to motor leads, confirm it spins
- [ ] If motor OK: re-run `motor_jog.py gpio 4 60 1.0` and observe terminal block for voltage
- [ ] If no voltage at terminal: check L293DD IC 2 on MotorShield (motors 3+4 share one IC)
- [ ] As a last resort: swap shoulder motor to a spare terminal and update config.py accordingly

---

## Open questions

- Which chassis side gives the cleaner camera view? (Check wiring obstructions on both sides before building the boom.)
- USB cable length needed from Pi USB port to camera at boom end — estimate 600–800 mm.
- Does the gripper link need its own ArUco marker, or is the wrist marker sufficient for pose? (Gripper opens/closes — a marker on the gripper would move as it operates.)
- Camera calibration board: standard checkerboard print. How large? A4 is sufficient for this working distance.

---

## GitHub wiki recommendation

The following content is design-reference material that would be better served
by the GitHub wiki than by files in the repo:

| Current location | Suggested wiki page |
|------------------|---------------------|
| `docs/owi535_ik_reference.md` | `IK Reference — OWI-535 (Debenec 2015)` |
| `docs/camera_mount.md` | `Camera Mount Design` |
| `DEVLOG.md` | `Development Log` |
| This file | `Roadmap` |
| Hardware schematic notes in `CLAUDE.md` | `Hardware Reference` |

The wiki is a separate git repo at `<repo>.wiki.git`. Create it from the GitHub
repository → Wiki tab, then migrate pages there at a convenient point.
Keep `CLAUDE.md` in the repo as the lean AI context file — it's a runtime
dependency of Claude Code sessions, not documentation for humans.
