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
| 4 | MPU-6050 IMU | Gyro for turn angle / base odometry | I2C at 0x68 — no conflict with 0x40. Plugs into Waveshare I2C expansion header (5-pin P?). | ~£3 |
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

### Phase 2c — Safety upgrade (vision-based)

**Goal:** stop arm automatically if any joint approaches its mechanical limit.

- `SafetyMonitor` watchdog currently uses time only.
- Add a second watchdog thread that reads joint angles from `ArucoTracker` and calls `emergency_stop()` if any joint exceeds its limit.
- Limits: shoulder 180°, elbow 260°, wrist 100° (measured values from `docs/owi535_ik_reference.md`).
- The time-based watchdog stays as a fallback for when the camera can't see the arm (occlusion).

---

### Phase 2d — Odometry

**Goal:** `move_base_forward(cm)` and `turn_base(degrees)` with reasonable accuracy.

**Initial approach (no new hardware):** time-based dead reckoning.
- Characterise speed vs actual velocity on the real surface.
- `move_base_forward(cm)`: run at calibrated speed for `cm / velocity` seconds.
- `turn_base(degrees)`: run pivot at calibrated speed for `degrees / rate` seconds.
- Known limitation: right track drift — will need per-track speed calibration.

**With MPU-6050 (recommended upgrade):**
- I2C at 0x68, no conflict. Plug into Waveshare I2C expansion header.
- Library: `smbus2` (already a dependency) or `mpu6050-raspberrypi`.
- Gyro Z gives turn rate → integrate for heading → stop when target heading reached.
- Accelerometer X gives forward motion estimate (noisy, but better than nothing).
- `argos/base/odometry.py`: `Odometry` class wrapping the IMU.

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
