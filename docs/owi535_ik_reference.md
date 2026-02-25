# OWI-535 Robotic Arm — Inverse Kinematics Reference

Reference data extracted from:
> Primož Debenec, *"Inverzna kinematika robotske roke OWI-535"*, University of Ljubljana, 2015

---

## Robot Arm Physical Specifications

### Arm Type
- **Articulated (revolute) robot arm** — all joints are rotational
- **4 DOF** (degrees of freedom) — 4 rotational joints + 1 gripper
- **Spherical workspace** envelope
- Max payload: **100 g**
- Total reach (fully extended, joint 1 to gripper): **27 cm**

### Joint Specifications

| Joint | Name | Rotation (spec) | Rotation (measured) | Axis |
|-------|------|----------------|---------------------|------|
| 1 | Base (rotation) | 270° | 250° | Vertical (Y-axis, yaw) |
| 2 | Shoulder | 180° | 194° | Horizontal (pitch) |
| 3 | Elbow | 300° | 260° | Horizontal (pitch) |
| 4 | Wrist | 120° | 100° | Horizontal (pitch) |
| 5 | Gripper | — | — | Open/close, max 4.5 cm |

**Key note:** Joint 1 rotates around the vertical axis (controls X and Z position).
Joints 2–4 all rotate in the vertical plane (control reach distance and height).
This means joints 2–4 can be solved in a **2D plane** after solving joint 1.

### Link Lengths (original OWI-535, from paper)

| From | To | Distance (cm) |
|------|----|---------------|
| Ground | Joint 1 | 5.0 |
| Joint 1 | Joint 2 (shoulder pivot) | 2.0 |
| Joint 2 | Joint 3 (elbow pivot) | 9.0 |
| Joint 3 | Joint 4 (wrist pivot) | 11.3 |
| Joint 4 | Gripper tip | 6.7 |

**Ground to shoulder pivot (joint 2) in original OWI-535: 5.0 + 2.0 = 7.0 cm**

### Link lengths on ARGOS (measured 2026-02-25)

The arm is a cannibalised OWI-535 — mechanical structure retained, motors and
control board replaced with SB Components MotorShield + DC motors.

| Segment | From | To | ARGOS measured | Paper value | Notes |
|---------|------|----|----------------|-------------|-------|
| L_base | Ground | Shoulder pivot | ~9 cm | 7.0 cm | Base/mounting differs from original |
| L2 | Shoulder pivot | Elbow pivot | 9.0 cm | 9.0 cm | Matches |
| L3 | Elbow pivot | Wrist pivot | 11.3 cm | 11.3 cm | Matches |
| L4 | Wrist pivot | Gripper tip | 10.5 cm | 6.7 cm | **Discrepancy — see note below** |

**L4 discrepancy:** the paper's 6.7 cm may measure to the gripper joint centre
rather than the physical tip of the closed fingers. ARGOS measured to the tip.
Use 10.5 cm for FK/IK end-effector position; use 6.7 cm if the target is the
wrist/gripper joint itself.

---

## Coordinate System

- **X, Z** axes define the ground plane (horizontal)
- **Y** axis defines height (vertical)
- Origin is at the base of the robot arm
- The arm is placed at the centre of the coordinate system

---

## Inverse Kinematics Algorithm

### Overview

The IK is solved analytically (geometric approach), not via Jacobian. The process:

1. **Input:** Target position (X, Y, Z) in 3D space
2. **Calculate set of possible solutions** (many due to multiple joint configurations)
3. **Filter invalid solutions** (joint limits, collisions with base/ground)
4. **Select optimal solution** (minimise maximum joint movement from current position)
5. **Output:** Joint angles θ₁, θ₂, θ₃, θ₄

### Step 1: Solve Joint 1 (Base Rotation)

Joint 1 is independent of joints 2–4. It determines the horizontal direction
the arm faces.

```
D = sqrt(X² + Z²)          // horizontal distance from base to target
θ₁ = acos(X / D)           // base rotation angle
```

**Two solutions may exist** for joint 1:
- **Solution A:** Direct rotation to face the target
- **Solution B:** Rotate 180° from solution A, with joints 2–4 "flipped" (arm reaches backward)

Solution B exists when the target is in the region where both orientations of
joint 1 can reach (roughly 180° of the 270° range):

```
θ₁' = abs(θ₁ - 180°)
```

The 90° dead zone of joint 1 can still be reached because joints 2–4 have
sufficient range to reach behind the base.

### Step 2: Reduce to 2D Problem

After solving joint 1, the 3D problem reduces to a **2D problem** in the
vertical plane containing the arm:

```
X' = sqrt(X² + Z²)         // horizontal distance from base axis
Y' = Y                      // height (unchanged)
```

Now solve for joints 2, 3, 4 as a 3-link planar arm.

### Step 3: Solve Joint 2 (Shoulder — Iterative Sweep)

Joint 2 is **not solved analytically**. The algorithm sweeps through all valid
values in fixed increments (default step = 5°):

```
for θ₂ = 0° to 180°, step 5°:
    // for each θ₂, compute joints 3 and 4 analytically
```

This produces 180/5 = **36 candidate values** for θ₂. Step size can be reduced
for finer precision at higher computational cost.

### Step 4: Solve Joints 3 and 4 (Elbow and Wrist — Analytical)

For each candidate θ₂, compute the position of joint 2's endpoint:

```
X₁' = L2 * cos(θ₂)
Y₁' = L2 * sin(θ₂)
```

Set the target in the sub-frame originating at joint 2:

```
X₃'' = X₃' - X₁'
Y₃'' = Y₃' - Y₁'
```

Solve θ₄ (wrist) using the **law of cosines**:

```
θ₄ = acos( (X₃''² + Y₃''² - L₃² - L₄²) / (2 * L₃ * L₄) )
```

Solve θ₃ (elbow):

```
D₂ = sqrt(X₃''² + Y₃''²)
θ₃ = ( 2 * atan(Y₃'' / (X₃'' + D₂)) - θ₃' ) - θ₂
```

Where θ₃' is an intermediate angle from the two-link sub-problem.

### Step 5: Forward Kinematics Verification

Each solution is verified using FK:

```
X₃' = L2*cos(θ₂·π/180) + L3*cos(θ₃·π/180) + L4*cos(θ₄·π/180)
Y₃' = L2*sin(θ₂·π/180) + L3*sin(θ₃·π/180) + L4*sin(θ₄·π/180)
```

---

## Solution Selection

### Joint Limits (IK Calculation Frame, centred at 0°)

| Joint | Range | Valid Values |
|-------|-------|--------------|
| 1 (Base) | 270° | -135° to +135° |
| 2 (Shoulder) | 180° | -90° to +90° |
| 3 (Elbow) | 300° | -150° to +150° |
| 4 (Wrist) | 120° | -60° to +60° |

Solutions outside these ranges are discarded. Use the **measured** ranges
(250°, 194°, 260°, 100°) rather than spec values for tighter safety limits.

### Optimal Solution Criteria

For each valid solution, compute the movement delta for each joint:

```
Δθ₂ = abs(θ₂_current - θ₂_candidate)
Δθ₃ = abs(θ₃_current - θ₃_candidate)
Δθ₄ = abs(θ₄_current - θ₄_candidate)
C_Max = max(Δθ₂, Δθ₃, Δθ₄)
```

**The solution with the smallest C_Max wins** — minimises time for the
slowest-moving joint, giving the fastest overall move.

### Gripper Orientation (Optional)

When constraining the end-effector to point downward or horizontally (for
picking objects from a surface):

```
θ_Sum = Δθ₂ + Δθ₃ + Δθ₄
```

Only solutions where the gripper points toward the ground plane or parallel
to it are kept.

### Format Conversion for Motor Commands

IK angles (centred around 0°) must be converted to motor command range
(0 to θ_Max):

```
// If θ < 0:
R = (θ_Max / 2) + abs(θ)

// If θ >= 0:
R = (θ_Max / 2) - θ
```

---

## Error Conditions (No Solution)

The IK function returns an error when:
- Target point is **beyond reach** (> 27 cm from base)
- Target is **too close** to or inside the base/controller area
- Target is **below ground** (Y < 0)
- **No valid solution** found after filtering

---

## Experimental Accuracy Results (from paper)

| Metric | Value |
|--------|-------|
| Average error (X axis) | 0.58 cm |
| Average error (Y axis) | 0.75 cm |
| Average error (Z axis) | 0.44 cm |
| **Average Euclidean error (point-to-point)** | **1.21 cm** |
| Average error (moving to same target from different starts) | 0.88 cm |

### Error Sources
- **Systematic error** observed — likely from imprecise joint calibration
- Mechanical play and low-cost construction contribute to inaccuracy
- Y-axis (height) has the largest average error
- The IK calculation itself is accurate (verified by FK); errors are primarily mechanical

---

## Notes for ARGOS / Camera-Based IK

Adaptation notes for ARGOS, which uses ArUco marker pose tracking and has no
base rotation joint (the base is fixed):

1. **No joint 1 (base rotation)** — ARGOS arm is fixed-base. Skip the base
   rotation step entirely. The 2D planar IK (joints 2–4) is the full IK.

2. **Shoulder height offset** — ARGOS shoulder pivot is ~9 cm above ground
   (vs 7 cm in the paper). Use ARGOS measured value for the base offset constant.

3. **L4 value** — use 10.5 cm (measured to gripper tip) for end-effector
   positioning; use 6.7 cm if targeting the wrist joint centre.

4. **Camera calibration** is essential — map pixel coordinates to the arm's
   coordinate frame before applying IK.

5. **ArUco markers** on each joint provide real-time joint positions.
   The **2D simplification** (vertical plane) aligns well with a side-mounted
   camera view.

6. **Iterative sweep step of 5°** for joint 2 can be reduced for better
   precision; consider 1° for final approach moves.

7. The **1.21 cm average error** (paper, open-loop) is the baseline.
   Visual feedback with ArUco markers should correct systematic mechanical
   errors in real time via closed-loop control.

8. **Use measured joint ranges** (shoulder 194°, elbow 260°, wrist 100°)
   for safety limits rather than spec values.
