# ARGOS — Camera Mount Design

## Robot overview

New Bright PowerHorse tracked chassis with a 4-joint arm mounted centrally,
folding forward along the robot's long axis.

| ![Side view](robot_side.jpeg) | ![Top view](robot_top.jpeg) |
|---|---|
| Side view — arm at full extension | Top-down view — arm folded forward |

---

## Arm plane of motion

The arm (shoulder → elbow → wrist → gripper) rotates in a **vertical plane
aligned with the robot's forward axis**. The camera must sit to the **left or
right side** of the robot, facing inward, to see this plane face-on.

```
         TOP VIEW

   ┌─────────────────────┐
   │   [Pi + HATs]       │  ← rear
   │                     │
   │      [shoulder]     │
   │      [elbow]        │
   │      [wrist]        │  ← arm along centreline
   │      [gripper]→→→  │  ← front
   └─────────────────────┘

   📷  ← camera here, ~30–40 cm lateral offset, pointing inward
       at roughly shoulder-to-wrist mid-height
```

```
         SIDE VIEW (camera's-eye view)

              [wrist]──[gripper]
             /
       [elbow]
      /
[shoulder]
    |
   base
```

---

## Camera selection

**Chosen: USB webcam**

- No ribbon cable length constraint — USB cable runs along chassis rail to any Pi USB port
- OpenCV `VideoCapture` — simpler to iterate with than `picamera2`
- 640×480 or 720p @ 30 fps is sufficient for ArUco detection (markers need ~30+ px across)
- Ball-head or swivel mount allows angle adjustment during initial calibration, then locks

*Pi Camera Module (CSI) reserved for a possible forward-facing navigation camera later.*

---

## Mount design

### Attachment point
- Clamp or bolt to the left or right chassis side rail (yellow track guard lip)
- Zip-tie or small U-bolt through the guard rail as a temporary bracket during prototyping

### Arm geometry
- Short horizontal arm extending ~30–40 cm laterally from the chassis side
- Camera height: level with shoulder-to-wrist midpoint (TBD from measurements)
- Camera angle: pointing inward, slight downward tilt to centre the arm in frame

### Rough sketch
```
   chassis side rail
         │
         ├──── horizontal arm (rod / printed bracket) ────┤
                                                          📷
                                              ↑ approx 30–40 cm lateral offset
```

---

## Measurements needed

Fill these in once measured on hardware:

| Dimension | Value |
|-----------|-------|
| Chassis width (outer track to outer track) | ___ mm |
| Chassis length | ___ mm |
| Arm base (shoulder pivot) height above deck | ___ mm |
| Shoulder → elbow link length | ___ mm |
| Elbow → wrist link length | ___ mm |
| Wrist → gripper tip length | ___ mm |
| Total arm reach (shoulder to gripper, fully extended) | ___ mm |
| Arm link width (flat panel width for ArUco marker) | ___ mm |

---

## ArUco marker placement

One marker per arm link, affixed to the outward-facing flat panel on each segment.
The camera-side face of each link must be unobstructed in the neutral pose.

| Joint segment | Suggested marker size | Notes |
|---|---|---|
| Shoulder link (upper arm) | 30 × 30 mm | largest panel, easiest target |
| Elbow link (forearm) | 25 × 25 mm | confirm flat panel width |
| Wrist segment | 20 × 20 mm | smaller — check clearance |
| Gripper base | 15 × 15 mm | may be too small; TBD on hardware |

Marker dictionary: `cv2.aruco.DICT_4X4_50` (small, robust at short range).

---

## Field of view calculation

*To be completed once link lengths and standoff distance are measured.*

```
Required FOV height ≈ total arm reach + margin
Required FOV width  ≈ total arm reach (arm can sweep forward/up)

At standoff D, with webcam vertical FOV θ:
  frame_height_at_arm = 2 × D × tan(θ/2)
  frame_height_at_arm must be ≥ total arm reach + ~20% margin
```

Typical 720p webcam: ~43° vertical FOV.
At 35 cm standoff: frame height ≈ 27 cm — sufficient if total arm reach < ~22 cm.

---

## Open questions

- [ ] Measure all arm link lengths (table above)
- [ ] Confirm which chassis side gives cleaner view (wiring may obstruct one side)
- [ ] Prototype bracket material: 3D-printed, aluminium angle, or repurposed hardware
- [ ] Confirm USB cable routing and length needed to reach Pi USB port
- [ ] Check flat panel dimensions on each arm link for marker sizing
- [ ] Decide whether gripper segment needs a marker or if wrist marker is sufficient
