# Swing-Arm Lamp: How the System and Kinematics Work

This note explains the architect / Anglepoise-style lamp we modelled in Dekal: what the real mechanism is, why it is not a simple 2-link robot arm, and how the code implements the kinematics.

Working model: `swing_arm_lamp.dek`

---

## 1. What we are modelling

The lamp is a **balanced-arm desk lamp** (Luxo / Anglepoise / architect lamp).

It looks like two links and a shade, but each “link” is a **parallelogram** made of two parallel bars, not a single rod.

```
                    shade (tips down)
                      |
                   [head coupler]  E -------- F     stays vertical
                         /              /
                   L2  /              /  L2         upper pair (parallel)
                     /              /
              [elbow coupler]  B -------- C         stays vertical
                    /              /
              L1  /              /  L1              lower pair (parallel)
                /              /
         [base coupler]  A -------- D               fixed, vertical
                         |
                       post
                         |
                       base
```

Three user inputs:

| Name | What it is | Typical meaning |
|------|------------|-----------------|
| `theta1` | world angle of the **lower** bar pair | `0` = horizontal, `π/2` = straight up |
| `theta2` | world angle of the **upper** bar pair | `0` = horizontal |
| `head_tilt` | extra rotation of the shade | `0` = pointing straight down |

Springs on the real lamp **do not change the kinematics**. They only cancel gravity so the arm stays where you put it. This model is geometry + motion only.

---

## 2. Why not a simple 2-link arm?

A textbook 2-link (RR) arm is:

```
base -- rz(θ1) -- link1 -- rz(θ2) -- link2 -- tip
```

`θ2` is a **relative** elbow angle: it is applied in the already-rotated frame of link 1. So the tip orientation is `θ1 + θ2`. If you move the shoulder, the forearm and the tool rotate with it.

A parallelogram lamp is different on purpose:

- each stage has **two** bars of equal length
- the short couplers (base, elbow, head) stay **parallel**
- if the base coupler is vertical, the elbow coupler stays vertical, and the head coupler stays vertical
- the shade stays level while you move the arms, unless you turn `head_tilt`

That is the product feature: move the arms, keep the light pointing the same way.

---

## 3. Parallelogram geometry (one stage)

Take the lower stage alone. Four points:

| Point | Role |
|-------|------|
| **A** | lower base pivot |
| **D** | upper base pivot |
| **B** | lower elbow pivot |
| **C** | upper elbow pivot |

Bars:

- **AD** = short coupler, length `h`, **fixed and vertical**
- **AB** = long bar, length `L1`
- **DC** = long bar, length `L1` (same as AB)
- **BC** = short coupler, length `h` (same as AD)

Because opposite sides are equal, ABCD is a parallelogram for every `theta1`:

- AB ∥ DC
- AD ∥ BC

AD is fixed vertical, so **BC is always vertical**. The elbow plate does not rotate when the lower arm moves. That is the whole trick.

The upper stage repeats the same pattern on BC:

- BE ∥ CF, length `L2`
- BC ∥ EF, so **EF stays vertical too**
- the shade mount therefore stays level

Two parallelograms stacked = **double parallelogram**.

Degrees of freedom (planar, ignoring the shade knob):

- a generic 4-bar has 1 DOF
- here we **drive** each parallelogram with its own angle
- so the lamp has **2 DOF** for the arms, plus **1 DOF** for the shade

---

## 4. Coordinate system and default pose

The lamp moves in a **vertical plane**.

| Axis | Meaning in this model |
|------|------------------------|
| **X** | forward, across the desk |
| **Y** | sideways (out of the motion plane) |
| **Z** | up |

Arm rotations are `ry` (about Y) so the bars stay in the XZ plane.

A rotation `ry(θ)` then a translation `tx(L)` places the far pivot at:

\[
(L \cos \theta,\; 0,\; L \sin \theta)
\]

relative to the near pivot.

| `θ` | Bar direction |
|-----|----------------|
| `0` | along +X (horizontal) |
| `π/2` | along +Z (straight up) |
| negative | below horizontal |

Default pose in the file (`theta1 = 1.15`, `theta2 = 0`):

- lower arm about **66°** from the desk (up and out)
- upper arm **horizontal**
- shade hanging **down** and slightly tipped

That matches the usual architect-lamp photograph: rear lower arm raised, front arm level, head pointing at the desk.

---

## 5. Forward kinematics (what the code actually does)

This model is **forward kinematics only**. You choose the angles; the code computes every pose. There is no `!` and no `collapse`. Nothing is solved iteratively, which is why the motion stays on one configuration and does not jump.

A pose in Dekal is a rigid frame in SE(3): position + orientation. Each `ry` / `tx` / `tz` takes a pose in and returns a new pose.

Chain from the desk up:

### 5.1 Base and lower coupler

```dek
let A = tz post base      // top of the post, lower base pivot
let D = tz h A            // upper base pivot, coupler AD along +Z
```

`A` and `D` never rotate. AD is a fixed vertical segment of length `h`.

### 5.2 Lower parallelogram

```dek
let B = A |> ry theta1 |> tx L1
let C = D |> ry theta1 |> tx L1
```

Both long bars use the **same** `theta1`. Therefore:

- AB and DC have the same length and the same world angle
- they stay parallel
- C is exactly B moved by `(0, 0, h)`
- BC stays vertical, parallel to AD

No solver is required: the parallelogram is true **by construction**.

### 5.3 Upper parallelogram — the important minus sign

`B` still carries orientation `theta1` (we rotated there, then translated). If we wrote:

```dek
B |> ry theta2 |> tx L2     // WRONG for a lamp
```

then the upper bars would inherit `theta1`, and their world angle would be `theta1 + theta2`. Moving the lower arm would swing the upper arm. That is serial-robot behaviour, not lamp behaviour.

The elbow joint that keeps `theta2` a **world** angle is:

```dek
let E = B |> ry (theta2 - theta1) |> tx L2
let F = C |> ry (theta2 - theta1) |> tx L2
```

Why `theta2 - theta1`:

- `B` is already rotated by `theta1`
- a further `ry(theta2 - theta1)` makes the total rotation `theta1 + (theta2 - theta1) = theta2`
- the upper bars therefore point at world angle `theta2`, no matter what `theta1` is

The relative elbow angle is `theta2 - theta1`. The parallelogram is what makes that the correct physical joint.

The same angle is applied to both upper bars, so BE ∥ CF and EF stays vertical.

### 5.4 Coupler drawings and shade

```dek
B |> ry (0.0 - theta1) |> tz h     // draw vertical elbow coupler BC
E |> ry (0.0 - theta2) |> tz h     // draw vertical head coupler EF

let shade = E |> ry (head_tilt - theta2) |> tz (0.0 - shade_len)
```

`ry (-theta1)` undoes the lower-arm rotation so `tz h` is world-vertical, not along the bar.

`head_tilt - theta2` undoes the upper-arm rotation, then applies the shade knob. Result: the shade orientation depends only on `head_tilt`. Move the arms, the light direction stays put.

---

## 6. Pose formulas (if you need equations)

Let `p` be the post height. Pivot positions in world coordinates:

\[
\begin{aligned}
A &= (0,\,0,\,p) \\
D &= (0,\,0,\,p + h) \\
B &= A + L_1(\cos\theta_1,\,0,\,\sin\theta_1) \\
C &= D + L_1(\cos\theta_1,\,0,\,\sin\theta_1) \\
E &= B + L_2(\cos\theta_2,\,0,\,\sin\theta_2) \\
F &= C + L_2(\cos\theta_2,\,0,\,\sin\theta_2)
\end{aligned}
\]

Check the parallelogram identities:

\[
C - B = D - A = (0,0,h), \qquad F - E = C - B = (0,0,h)
\]

The shade origin (hanging down from a world-aligned head, then tipped by `head_tilt`):

\[
S = E + R_y(\text{head\_tilt})\,(0,\,0,\,-\ell_{\text{shade}})
\]

Jacobian / velocities (optional): differentiate with respect to time. With `θ1(t)` and `θ2(t)` independent,

\[
\dot B = L_1(-\sin\theta_1,\,0,\,\cos\theta_1)\,\dot\theta_1
\]

\[
\dot E = \dot B + L_2(-\sin\theta_2,\,0,\,\cos\theta_2)\,\dot\theta_2
\]

The upper-bar direction does not depend on `θ1`. That is the kinematic statement of “the head stays level.”

---

## 7. How this maps onto Dekal

Dekal represents every frame as an SE(3) pose. A mechanism is a **tree of transforms**.

| Idea | In this lamp |
|------|----------------|
| Root pose | `root` / `base` |
| Driven inputs | named `let`s: `theta1`, `theta2`, `head_tilt` |
| Revolute joint | `ry angle` |
| Rigid bar | `tx L` or `tz h` |
| Composition | `|>` (left to right, each step in the **local** frame) |
| Free unknown | `!` — **not used here** |
| Loop closure | `collapse` — **not used here** |

### Forward vs inverse (why we do not `collapse`)

| | Forward (this file) | Inverse |
|--|---------------------|---------|
| You set | `theta1`, `theta2` | a target for the shade |
| You get | all pivot poses | the angles |
| Free vars | none | `ry !` on the joints |
| `collapse` | not needed | yes: force chain tip = target |

`collapse a b` means: find the free variables so poses `a` and `b` are the same element of SE(3). That is loop closure / IK.

An open serial chain with known angles has nothing to close. A parallelogram **could** be written as a four-bar with `!` on the follower bar and `collapse` at C. That is physically valid, but the solver has two assemblies (open vs crossed) and can jump when you drag. Driving both bars with the same angle avoids that: one configuration, smooth motion.

### `!` vs `let` vs `?`

- **`let theta1 = 1.15`** — parameter you choose (slider)
- **`ry !`** — unknown the solver must find
- **`?`** — hidden unknown, used for reaction loads in `solve_load`

This lamp uses only `let`.

### Local vs world

`A |> ry theta1 |> tx L1` means:

1. start at `A`
2. rotate about **A’s local Y**
3. walk along the **new** local X by `L1`

After step 2, local X is already tilted in the world XZ plane, so the bar is diagonal. Order matters.

---

## 8. How to talk through the demo

1. **Identify the parts.** Short vertical couplers at base, elbow, and head. Two long bars per stage.
2. **Drag `theta1` only.** Lower bars rotate; elbow moves on a circle of radius `L1`; upper bars keep their direction; shade keeps pointing down.
3. **Drag `theta2` only.** Upper bars rotate about the elbow; lower bars stay put; shade still keeps its tilt.
4. **Drag `head_tilt`.** Only the shade rotates. That is the real lamp’s head knob.
5. **Point at `theta2 - theta1`.** That is the elbow’s relative angle. It is how a world-angle input is applied in a frame that already contains `theta1`.

If someone asks “where are the springs?”: they are a **statics** device (`F = kx` and torque balance), not a position constraint. Omit them for a kinematics explanation.

---

## 9. Line-by-line map of `swing_arm_lamp.dek`

| Lines | Role |
|-------|------|
| `h, L1, L2, post, shade_len` | geometry (metres) |
| `theta1, theta2, head_tilt` | driven angles (radians) |
| `A`, `D` | fixed base pivots |
| `B`, `C` | lower parallelogram, driven by `theta1` |
| `E`, `F` | upper parallelogram, driven by world `theta2` |
| `ry (0 - theta1)` / `ry (0 - theta2)` then `tz h` | draw the vertical couplers |
| `shade` | head, hanging down in world Z |
| `show root cam [...]` | draw the pose tree and expose sliders |
| `animate` | resubstitute angles vs time; same FK, no solve |

---

## 10. One-sentence summary

**The lamp is two parallelograms in a vertical plane: each pair of bars shares a world angle so the couplers stay vertical; the elbow uses `θ2 − θ1` so the upper pair does not inherit the lower pair; the shade uses `head_tilt − θ2` so the light direction is independent of the arms.**
