# Five-Bar Mechanism Mathematics — Simple Explanation

This document explains the mathematics used in `five_bar_cad_like.py`. It starts with the physical meaning and then introduces the formulas one at a time.

## 1. What the mechanism looks like

The mechanism has five rigid bars:

```text
          C -------- D -------- E
         /                      \
        /                        \
       A ------------------------ B

       A and B are fixed to the ground.
```

The closed chain is:

```text
A → C → D → E → B → A
```

The bar names and lengths are:

| Bar | Length in the Python example |
|---|---:|
| AC | 170 pixels |
| CD | 240 pixels |
| DE | 240 pixels |
| EB | 170 pixels |
| BA | 560 pixels |

`A` and `B` never move. The user rotates bar `AC`. The program must calculate the positions of `C`, `D`, and `E` without changing any bar length.

## 2. The basic distance formula

If two points are

\[
P=(P_x,P_y)
\]

and

\[
Q=(Q_x,Q_y),
\]

the distance between them is

\[
\operatorname{distance}(P,Q)
=
\sqrt{(Q_x-P_x)^2+(Q_y-P_y)^2}.
\]

This is only the Pythagorean theorem:

- horizontal distance: \(Q_x-P_x\);
- vertical distance: \(Q_y-P_y\);
- direct distance: the hypotenuse.

For a rigid bar with length \(L\), the condition is simply

\[
\operatorname{distance}(P,Q)=L.
\]

For example, bar `CD` is rigid when

\[
\operatorname{distance}(C,D)=240.
\]

## 3. Calculating point C from the input angle

Point `A` is fixed. Bar `AC` has length \(L_{AC}\), and its angle is \(\theta\).

The horizontal part of the bar is

\[
\Delta x=L_{AC}\cos(\theta).
\]

The vertical part is

\[
\Delta y=L_{AC}\sin(\theta).
\]

Therefore:

\[
C_x=A_x+L_{AC}\cos(\theta),
\]

\[
C_y=A_y+L_{AC}\sin(\theta).
\]

In plain language:

```text
new point C = fixed point A + rotated bar AC
```

### Numerical example

Assume:

```text
A = (170, 410)
length AC = 170
angle = 300°
```

We know:

\[
\cos(300^\circ)=0.5,
\]

\[
\sin(300^\circ)\approx-0.866.
\]

Then:

\[
C_x=170+170(0.5)=255,
\]

\[
C_y=410+170(-0.866)\approx262.8.
\]

So:

\[
C\approx(255,262.8).
\]

The minus sign moves the point upward because the Tkinter screen coordinate \(y\) increases downward.

## 4. Why one input angle is not enough for one unique answer

The complete mechanism has two degrees of freedom. Rotating `AC` specifies one of them. One free movement still remains.

After `C` is known, point `E` can still move around fixed point `B` while bar `EB` keeps the same length:

```text
                   E can move on this circle
                         ○ ○ ○
                     ○           ○
                   ○       B       ○
                     ○           ○
                         ○ ○ ○
```

This is why there are many correct mechanism positions for the same angle of `AC`.

The program does not ask the user for another angle. Instead, it chooses the valid position closest to the previous frame. This is the hidden selection rule that creates CAD-like continuous motion.

## 5. Calculating point E

Because bar `EB` is rigid, `E` must stay on a circle centered at `B`.

The program uses an internal angle \(\phi\):

\[
E_x=B_x+L_{EB}\cos(\phi),
\]

\[
E_y=B_y+L_{EB}\sin(\phi).
\]

The user does not enter \(\phi\). The program tests possible values automatically.

Every tested point `E` is exactly \(L_{EB}\) away from `B`. Therefore bar `EB` cannot stretch.

## 6. How point D is found

Point `D` must satisfy two rules at the same time:

```text
distance from C to D = length CD
distance from E to D = length DE
```

Geometrically:

- draw a circle centered at `C` with radius `CD`;
- draw a circle centered at `E` with radius `DE`;
- point `D` must be at an intersection of the circles.

```text
         circle around C       circle around E
              .----.             .----.
           .-'      '-.       .-'      '-.
          C            D₁     E
           '-.      .-'  '-.      .-'
              '----'   D₂  '----'
```

There can be:

- two intersections: two possible assembly positions;
- one intersection: the bars are exactly aligned;
- zero intersections: the mechanism cannot close.

## 7. When the two circles can intersect

First calculate the distance between `C` and `E`:

\[
d=\operatorname{distance}(C,E).
\]

Let:

```text
r₀ = length CD
r₁ = length DE
```

The circles intersect only when

\[
|r_0-r_1|\le d\le r_0+r_1.
\]

The right side is easy to understand:

\[
d\le r_0+r_1.
\]

If `C` and `E` are farther apart than the two bars added together, the bars cannot reach each other.

Example:

```text
CD = 240
DE = 240
maximum reachable distance C–E = 240 + 240 = 480
```

If the distance from `C` to `E` is 500, no solution exists.

The left side means that one circle must not be completely inside the other without touching:

\[
d\ge |r_0-r_1|.
\]

In this example both radii are 240, so the lower limit is zero.

## 8. Exact circle-intersection calculation

The code first calculates the vector from `C` to `E`:

\[
dx=E_x-C_x,
\]

\[
dy=E_y-C_y.
\]

Its length is

\[
d=\sqrt{dx^2+dy^2}.
\]

Next, it calculates how far to travel from `C` toward `E` to reach the middle of the intersection chord:

\[
a=\frac{r_0^2-r_1^2+d^2}{2d}.
\]

Meaning of each symbol:

| Symbol | Meaning |
|---|---|
| \(r_0\) | length `CD` |
| \(r_1\) | length `DE` |
| \(d\) | distance from `C` to `E` |
| \(a\) | distance from `C` to the middle point between the two intersections |

Then the perpendicular distance from that middle point to either intersection is

\[
h=\sqrt{r_0^2-a^2}.
\]

The middle point is

\[
P_x=C_x+a\frac{dx}{d},
\]

\[
P_y=C_y+a\frac{dy}{d}.
\]

Finally, rotate the normalized direction by 90 degrees and move by \(h\):

\[
D_{1x}=P_x-h\frac{dy}{d},
\]

\[
D_{1y}=P_y+h\frac{dx}{d},
\]

\[
D_{2x}=P_x+h\frac{dy}{d},
\]

\[
D_{2y}=P_y-h\frac{dx}{d}.
\]

These are the two exact possible positions of `D`.

You do not need to memorize these formulas. The important idea is:

```text
D is not guessed.
D is calculated as the exact intersection of two circles.
```

That is why `CD` and `DE` never stretch.

## 9. How the program chooses between D₁ and D₂

Both circle intersections are mathematically correct. To avoid randomly jumping between them, the program compares each candidate with the previous point `D`.

It selects the one with the smaller movement:

\[
\operatorname{distance}(D_{candidate},D_{old}).
\]

In plain language:

```text
choose the new D that is nearest to the previous D
```

This preserves the current assembly branch whenever possible.

## 10. How the program chooses the free E position

The program tries several values of the internal angle \(\phi\). For each value it:

1. calculates an exact `E` on the circle around `B`;
2. tries to calculate `D` from the two circle intersections;
3. rejects the candidate if the circles do not intersect;
4. measures how far `D` and `E` moved from the previous frame.

The movement score is

\[
J=
\operatorname{distance}(D,D_{old})^2+
\operatorname{distance}(E,E_{old})^2.
\]

This formula simply means:

```text
score = movement of D + movement of E
```

The program chooses the valid candidate with the smallest score.

This score does not stretch the rods. It only chooses one exact rigid configuration from many possible configurations.

## 11. Why some angles between 0° and 360° have no solution

The slider permits a full rotation, but the mechanism cannot physically close at every angle.

After `C` is calculated, the three bars `CD`, `DE`, and `EB` must connect `C` to fixed point `B`.

Their maximum total reach is

\[
L_{CD}+L_{DE}+L_{EB}.
\]

Using the example values:

\[
240+240+170=650.
\]

Therefore, if

\[
\operatorname{distance}(C,B)>650,
\]

the mechanism cannot close. Stretching would be required, but stretching is forbidden.

The program then:

- keeps the last valid mechanism position;
- does not modify any rod length;
- displays `ERROR: MECHANISM CANNOT CLOSE`.

## 12. Why all rod lengths remain exact

Each joint is constructed using rigid geometry:

| Bar | How its length is guaranteed |
|---|---|
| AC | `C` is calculated using `cos` and `sin` with radius `AC` |
| EB | `E` is calculated using `cos` and `sin` with radius `EB` |
| CD | `D` lies on the circle centered at `C` with radius `CD` |
| DE | `D` lies on the circle centered at `E` with radius `DE` |
| BA | `A` and `B` are fixed |

The program verifies the largest length error using

\[
\text{maximum error}
=
\max(
|AC-L_{AC}|,
|CD-L_{CD}|,
|DE-L_{DE}|,
|EB-L_{EB}|
).
\]

Here `AC`, `CD`, `DE`, and `EB` inside the absolute-value signs mean the measured distances between the corresponding points.

The displayed result is normally extremely close to zero. A value such as

```text
0.000000000001
```

is floating-point rounding, not physical stretching.

## 13. Complete solver sequence

For every new input angle:

```text
1. Save the last valid C, D, and E positions.
2. Calculate C from angle θ and rigid length AC.
3. Search possible E positions on the rigid circle around B.
4. For every E, calculate exact circle intersections for D.
5. Reject impossible candidates.
6. Select the valid candidate nearest to the previous pose.
7. If a candidate exists, show the new mechanism.
8. If none exists, restore the previous pose and show ERROR.
```

## 14. The central idea

The entire calculation can be understood with three simple rules:

1. A point connected to one fixed point by one rigid bar moves on a circle.
2. A point connected to two known points by two rigid bars is found by intersecting two circles.
3. If several exact solutions exist, choose the one nearest to the previous frame.

These three rules produce continuous CAD-like motion without requiring a second user-controlled parameter and without allowing any bar to stretch.
