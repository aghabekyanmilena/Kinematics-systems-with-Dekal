# Kinematics Study Guide: Dekal → ForgeCAD

Your main task is not “draw a square.” It is to understand **how motion is constrained**.

---

## 1. The mental model

Every good kinematic system answers four questions:

| Question | Four-bar answer | Tool job |
|----------|-----------------|----------|
| What is **fixed**? | A, D (and bar AD) | Anchors / `fixed: true` |
| What is **rigid**? | Lengths AB, BC, DC, AD | Distance constraints / edges |
| What is **free**? | Angles at hinges (except the driven one) | Unknowns / `!` / solver DOFs |
| What is the **input**? | `lean` (angle of AB) | Driven control |

If you skip any of these, you do not have kinematics — you have a drawing of one pose.

**Kinematics = geometry + constraints + one (or few) inputs → solver fills the rest.**

---

## 2. Why the square four-bar has good kinematics

```
B ----- BC ----- C      movable
|                |
AB               DC     movable
|                |
A ----- AD ----- D      FIXED
```

### Mobility (count DOF before coding)

- 4 rigid bodies in the plane → each free body has 3 DOF  
- Ground is fixed → remove ground’s 3 DOF:

\[
M = 3(N-1) - 2J_r = 3(4-1) - 2\cdot 4 = 1
\]

So: **exactly one input**. More inputs → overconstrained or fighting. Fewer → floppy.

### Loop closure

Two paths from A to C must agree:

\[
A \to B \to C \quad=\quad A \to D \to C
\]

\[
\vec{AB} + \vec{BC} = \vec{AD} + \vec{DC}
\]

That is **2 equations** (x and y in the plane).  
With lengths fixed and one angle driven, the remaining angles are determined.

### Geometric picture

1. Drive angle at A → **B** moves on a **circle** around A  
2. **C** must lie on a circle around **B** (radius BC)  
3. **C** must also lie on a circle around **D** (radius DC)  
4. **C** = intersection of those two circles  

That intersection **is** the kinematic solution.

### Forward kinematics (FK)

**Input:** driven joint (`lean`)  
**Output:** all other joint angles and point positions  

This is what Dekal does when you drag `lean`.

---

## 3. How Dekal encodes the same ideas

| Kinematic idea | Dekal |
|----------------|-------|
| Fixed point | `let A = root`, `let D = tx 1.0 root` |
| Rigid bar | `ty 1.0` (length) |
| Free hinge | `rz !` |
| Driven hinge | `rz (active reaction lean)` |
| Loop closure | `collapse pop pop` |
| Solve | automatic when you `show` / drag |

Dekal’s `collapse` **is** the loop-closure equation.  
You never place C by hand; the solver finds C.