# Square Four-Bar in Dekal — Presentation Script

**Run:** https://demo.dekal.sh/ → paste `fourbar_2d_minimal.dek` → Shift+Enter  
**Drag:** `lean` to move the top points

---

## 1. What are we building?

A **planar four-bar** shaped like a square / parallelogram:

```
B -------- C     ← MOVING (top bar)
|          |
|          |
A -------- D     ← FIXED (ground point A)
```

| Joint | Role |
|-------|------|
| **A** | Fixed bottom-left |
| **D** | Fixed bottom-right |
| **B** | Moving top-left |
| **C** | Moving top-right |

Four rigid bars: ground AD, left AB, top BC, right DC.  
All side lengths = 1 → upright pose is a **square**. When it leans, it becomes a **parallelogram**.

**Why 1 DOF?** Gruebler for a planar mechanism:  
`M = 3(N−1) − 2J = 3(4−1) − 2·4 = 1`  
So we drive one input (`lean`); everything else is solved.

---

## 2. How kinematics works

### 2.1 Bodies, joints, DOF

| Body | Bar | Length |
|------|-----|--------|
| Ground | AD | 1 (fixed) |
| Side | AB | 1 |
| Coupler | BC | 1 |
| Side | DC | 1 |

Each planar rigid body has 3 DOF (x, y, rotation).  
Each revolute joint removes 2 DOF. With ground fixed:

`M = 3(4−1) − 2·4 = 1`

One independent input → we choose **lean** = angle of AB. All other angles/positions are **dependent**.

### 2.2 Loop-closure (position kinematics)

Put A at `(0,0)`, D at `(1,0)`. Let:

- `θ` = `lean` = angle of AB from the local +Y axis (as in the Dek code)
- `φ` = unknown angle of DC (solver finds this)

Then (with bars along local Y after `rz`):

- **B** is fixed once `θ` is known: it lies on a circle of radius 1 around A  
- **C** must satisfy **two** distance constraints at once:
  - `|C − B| = 1` (top bar BC)
  - `|C − D| = 1` (right bar DC)

In vector form (classic four-bar loop):

`A→B + B→C = A→D + D→C`

That single vector equation is **2 scalar equations** (x and y).  
Unknowns after fixing `θ`: mainly `φ` (and coupler orientation).  
2 equations + 1 input → configuration is determined (up to assembly mode).

**In words:** choose where B is → C must sit at an intersection of  
“circle around B (radius 1)” and “circle around D (radius 1)”.  
That intersection is the kinematic solution for C.

### 2.3 What motion you see

1. Drag `lean` → AB rotates about fixed **A**  
2. **B** moves on a circle around A  
3. Circle around B and circle around D update  
4. **C** jumps to their intersection (continuous branch while you drag)  
5. **A** and **D** never move  

Because AB = DC = BC = AD = 1, the figure stays a **parallelogram**:  
opposite sides stay equal and parallel. Top BC stays parallel to ground AD.

### 2.4 Forward vs inverse

- **Forward kinematics:** input = `lean` → outputs = poses of B and C (what the demo does)  
- **Inverse kinematics:** input = desired C (or B) → find `lean` (possible, but 1 DOF so only paths that the linkage can reach)

### 2.5 How this maps to Dekal

| Kinematics idea | In our code |
|-----------------|-------------|
| Fixed pivots A, D | `let A = root`, `let D = tx 1.0 root` |
| Input angle θ | `lean` + `A \|> rz (active … lean)` |
| Unknown angle φ | `D \|> rz !` |
| Rigid bar length 1 | `ty 1.0` inside `link` |
| Free hinge | `rz !` |
| Loop closure A→B→C = A→D→C | `collapse pop pop` |
| Solve unknowns | Dekal’s solver (implicit) |

Dekal does **not** make you write sin/cos by hand.  
`collapse` **is** the loop-closure equation; the solver finds `φ` and the free hinge angles.

### 2.6 Velocity (optional, if asked)

Differentiate the loop equation w.r.t. time:

`v_B + ω_BC × (C−B) = v_C` (and C also rotates about D)

With 1 input `θ̇ = d(lean)/dt`, all other angular rates are linear in `θ̇`.  
Same structure as positions: one free rate, rest follow from constraints.

---

## 3. How Dekal thinks

Dekal is an **implicit** constraint solver:

1. You describe **allowed motions** with transforms (`tx`, `ty`, `rz`)
2. `!` means “unknown — solver finds it”
3. `collapse` means “these two chains must meet at the same place” (loop closure)

You do **not** place parts then add constraints afterward. You write two open chains and close them.

---

## 4. Build steps

### Step 1 — Origin

```dek
let root = tx 0.0 origin
```

World frame for the mechanism.

### Step 2 — Two fixed points (ground)

```dek
let A = root
let D = tx 1.0 root
```

- **A** at the origin  
- **D** one unit to the right  
→ bottom bar AD is fixed. These two points **never move**.

### Step 3 — Input variable

```dek
let lean = 0.2
let reaction = ?
```

- `lean` = angle of the left leg (what you drag)  
- `reaction = ?` = placeholder for optional load/reaction later  

### Step 4 — One reusable link

```dek
let link = fun atPos ->
    ty 1.0 atPos |> rz !
end
```

From a joint:

1. `ty 1.0` — go length 1 along local Y (the rigid bar)  
2. `rz !` — free hinge at the far end (`!` = unknown angle)

Same helper builds AB, BC, and DC.

### Step 5 — Left and right pivots (rotation at A and D)

```dek
let left  = A |> rz (active reaction lean)   // driven hinge at A
let right = D |> rz !                        // free hinge at D
```

- At **A**: rotation is **driven** by `lean` (`active` → interactive slider)  
- At **D**: rotation is **free** (`!`) — solver picks the angle that closes the loop  

### Step 6 — Reach moving point B

```dek
let B = left |> link
```

From the rotated left hinge, add one bar of length 1 → **B** (top-left).  
When you change `lean`, **B** moves on a circle around **A**.

### Step 7 — Two paths to C, then close the loop

```dek
B |> link        // path 1: B --top bar--> C
right |> link    // path 2: D --right leg--> C
collapse pop pop
```

- Path 1: from **B**, another bar (top BC) toward **C**  
- Path 2: from **D**, the right leg toward **C**  
- `collapse pop pop` = force both paths to end at the **same** point **C**

That is the whole four-bar: loop closure.

### Step 8 — Show it

```dek
let cam = ty (-1.0) origin |> tz 2.5
show root cam [lean]
```

Draw the mechanism and expose **lean** as a draggable control.

---

## 5. What happens when you drag `lean`

1. Left hinge at A rotates  
2. **B** moves (circle around A)  
3. Top bar pushes/pulls  
4. Solver updates free angle at D  
5. **C** moves so bars BC and DC stay length 1  
6. **A** and **D** stay fixed  

So: move one top corner → the other follows because they are connected by the bars.
