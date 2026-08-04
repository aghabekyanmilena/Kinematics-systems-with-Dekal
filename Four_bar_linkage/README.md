# Square Four-Bar Linkage (Dekal)

Planar four-bar mechanism in [Dekal](https://demo.dekal.sh/): a square with a **fixed bottom** and a **movable top**.

## What it is

```
B ----- BC ----- C      movable top bar
|                |
AB               DC     movable side bars
|                |
A ----- AD ----- D      FIXED ground bar
```

| Part | Status | Role |
|------|--------|------|
| Points **A**, **D** | Fixed | Ground pivots |
| Bar **AD** | Fixed | Ground (bottom of the square) |
| Points **B**, **C** | Movable | Top corners |
| Bars **AB**, **BC**, **DC** | Movable | The three moving bars |

All four sides have length **1**. At `lean = 0` the figure is an upright **square**. When `lean` changes it becomes a **parallelogram** (opposite sides stay equal and parallel).

Working file: **`fourbar_2d_minimal.dek`**

---

## How to run

1. Open https://demo.dekal.sh/
2. Delete any code in the editor
3. Paste the contents of `fourbar_2d_minimal.dek`
4. Press **Shift+Enter**
5. Drag **lean** — A, D, and AD stay fixed; B, C and the three upper bars move

---

## Kinematics

### Degrees of freedom

A planar rigid body has 3 DOF (x, y, rotation).  
A revolute joint removes 2 DOF. With one body fixed as ground:

\[
M = 3(N - 1) - 2 J = 3(4 - 1) - 2 \cdot 4 = 1
\]

So the linkage has **one degree of freedom**.  
We choose that input as **`lean`** = rotation of bar AB about fixed point A.  
Every other angle and the positions of B and C are **dependent** — the solver computes them.

### Fixed vs free

- **A** at \((0, 0)\), **D** at \((1, 0)\) — never move  
- **AD** length 1 — fixed ground  
- **AB**, **BC**, **DC** length 1 — rigid but free to rotate at their hinges  

### Position kinematics

Let \(\theta\) = `lean` (angle of AB).  
Let \(\phi\) = unknown angle of DC (found by the solver).

Once \(\theta\) is set:

1. **B** is determined: it lies on a circle of radius 1 around **A**
2. **C** must satisfy two distance constraints at the same time:
   - \(|C - B| = 1\) (top bar BC)
   - \(|C - D| = 1\) (side bar DC)

So **C** is an intersection of two circles:

- circle centered at B with radius 1  
- circle centered at D with radius 1  

In vector form (classic four-bar loop-closure equation):

\[
\overrightarrow{AB} + \overrightarrow{BC} = \overrightarrow{AD} + \overrightarrow{DC}
\]

That is **two** scalar equations (x and y).  
With input \(\theta\) known, the unknowns (mainly \(\phi\) and the coupler orientation) are solved so both paths to C agree.

### Forward kinematics

| | |
|--|--|
| **Input** | `lean` (angle of AB at A) |
| **Outputs** | Positions of B and C; angle of DC at D |

Drag `lean` → AB rotates → B moves on its circle → C follows so BC and DC stay length 1.

### Why B and C move together

They are connected by bar **BC**.  
You cannot move B without changing the feasible set for C (intersection of the two circles).  
The three movable bars stay assembled; only the bottom stays put.

### Parallelogram property

Because all sides equal (`AB = BC = DC = AD = 1`), the mechanism is a **parallelogram linkage**:

- opposite sides remain parallel  
- top bar BC stays parallel to ground AD  
- upright pose (`lean = 0`) is a square  

### How kinematics maps to Dekal code

| Kinematics idea | In `fourbar_2d_minimal.dek` |
|-----------------|----------------------------|
| Fixed A, D and ground AD | `let A = root`, `let D = tx 1.0 root` |
| Input angle \(\theta\) | `lean` + `hinge_A = A \|> rz (active reaction lean)` |
| Unknown angle \(\phi\) at D | `hinge_D = D \|> rz !` |
| Rigid bar length 1 | `ty 1.0` inside `bar` |
| Free hinge at far end of a bar | `rz !` |
| Loop closure (two paths meet at C) | `B \|> bar`, `hinge_D \|> bar`, then `collapse pop pop` |
| Solve for unknowns marked `!` | Dekal constraint solver |

You do not write \(\sin\) / \(\cos\) by hand.  
`collapse` **is** the loop-closure constraint; Dekal finds every `!`.

---

## How the model is built

### 1. Origin

```dek
let root = tx 0.0 origin
```

### 2. Fixed bottom (points + bar)

```dek
let A = root
let D = tx 1.0 root
```

A and D are fixed; AD is the fixed ground bar of length 1.

### 3. Input

```dek
let lean = 0.0
let reaction = ?
```

- `lean = 0` → upright square  
- `reaction` is reserved for optional load analysis  

### 4. Movable bar helper

```dek
let bar = fun atPos ->
    ty 1.0 atPos |> rz !
end
```

- `ty 1.0` — rigid segment of length 1  
- `rz !` — free revolute joint at the far end  

Used for all three movable bars AB, BC, DC.

### 5. Hinges at the fixed points

```dek
let hinge_A = A |> rz (active reaction lean)   // driven
let hinge_D = D |> rz !                        // free
```

### 6. Movable point B (bar AB)

```dek
let B = hinge_A |> bar
```

### 7. Movable point C (bars BC and DC + loop closure)

```dek
B |> bar           // path A → B → C along AB then BC
hinge_D |> bar     // path D → C along DC
collapse pop pop   // both paths must end at the same C
```

### 8. Display

```dek
let cam = ty (-1.0) origin |> tz 2.5
show root cam [lean]
```

---

## Dek language cheat sheet

| Syntax | Meaning |
|--------|---------|
| `tx`, `ty`, `rz` | Translate / rotate |
| `\|>` | Pipe into next transform |
| `!` | Unknown DOF (solver fills it in) |
| `active reaction lean` | Interactive input + reaction channel |
| `collapse pop pop` | Force two open chains to meet (loop closure) |
| `show root cam [lean]` | Draw mechanism and expose `lean` |

Dekal is an **implicit** solver: you describe allowed motion subspaces, then `collapse` them until the pose is determined.

---

## Files

| File | Purpose |
|------|---------|
| `fourbar_2d_minimal.dek` | **Working model** — paste into Dekal |
| `PRESENTATION.md` | Short talk / demo script |
| `fourbar_2d.dek` | Longer earlier variant (prefer minimal) |
| `reference-examples/` | Official Dekal demos for comparison |

---