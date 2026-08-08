import math
import tkinter as tk


W, H = 900, 600


class FiveBarDemo:
    """Five rods A-C-D-E-B-A. A and B are fixed; AC is driven."""

    def __init__(self, root):
        self.root = root
        root.title("5-bar mechanism — CAD-like continuation")

        self.canvas = tk.Canvas(root, width=W, height=H, bg="#f7f8fa")
        self.canvas.pack(fill="both", expand=True)

        panel = tk.Frame(root)
        panel.pack(fill="x", padx=10, pady=8)
        self.running = False
        self.direction = 1.0
        self.angle = 305.0
        self.valid = True

        tk.Button(panel, text="Play / Pause", command=self.toggle).pack(side="left")
        tk.Button(panel, text="Reset", command=self.reset).pack(side="left", padx=6)
        tk.Label(panel, text="Angle AC:").pack(side="left", padx=(15, 4))
        self.slider = tk.Scale(panel, from_=0, to=360, orient="horizontal",
                               length=380, resolution=0.5,
                               command=self.slider_changed)
        self.slider.set(self.angle)
        self.slider.pack(side="left")
        self.info = tk.Label(panel, text="")
        self.info.pack(side="right")

        self.A = [170.0, 410.0]
        self.B = [730.0, 410.0]
        self.L_AC = 170.0
        self.L_CD = 240.0
        self.L_DE = 240.0
        self.L_EB = 170.0
        self.C = [0.0, 0.0]
        self.D = [430.0, 180.0]
        self.E = [620.0, 270.0]
        self.set_driven_point()
        self.solve(400)
        self.draw()
        self.tick()

    def set_driven_point(self):
        a = math.radians(self.angle)
        self.C[0] = self.A[0] + self.L_AC * math.cos(a)
        self.C[1] = self.A[1] + self.L_AC * math.sin(a)

    def circle_intersections(self, p0, r0, p1, r1):
        """Return exact intersections of two circles (zero, one or two)."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        d = math.hypot(dx, dy)
        if d < 1e-12 or d > r0 + r1 or d < abs(r0 - r1):
            return []
        a = (r0*r0 - r1*r1 + d*d) / (2*d)
        h2 = max(0.0, r0*r0 - a*a)
        h = math.sqrt(h2)
        x = p0[0] + a * dx / d
        y = p0[1] + a * dy / d
        rx, ry = -dy * h / d, dx * h / d
        return [[x + rx, y + ry], [x - rx, y - ry]]

    def candidate(self, phi, old_d, old_e):
        """Place E exactly on EB and D exactly at the CD/DE intersection."""
        e = [self.B[0] + self.L_EB * math.cos(phi),
             self.B[1] + self.L_EB * math.sin(phi)]
        ds = self.circle_intersections(self.C, self.L_CD, e, self.L_DE)
        if not ds:
            return None
        d = min(ds, key=lambda p: math.dist(p, old_d))
        cost = math.dist(d, old_d)**2 + math.dist(e, old_e)**2
        return cost, d, e

    def solve(self, iterations=100):
        # The free DOF is E's angle around B. We do not ask the user for it:
        # select the exact valid pose closest to the previous frame.
        old_d, old_e = self.D[:], self.E[:]
        phi0 = math.atan2(old_e[1] - self.B[1], old_e[0] - self.B[0])

        best = None
        # Local search preserves continuity; global search handles large slider jumps.
        for span, samples in ((math.radians(25), 301), (math.pi, 721)):
            for i in range(samples):
                phi = phi0 - span + 2 * span * i / (samples - 1)
                result = self.candidate(phi, old_d, old_e)
                if result is not None and (best is None or result[0] < best[0]):
                    best = (*result, phi)
            if best is not None:
                break

        if best is None:
            return False

        # Refine the selected free angle without ever relaxing rod lengths.
        phi = best[3]
        step = math.radians(25) / 300
        for _ in range(18):
            options = [self.candidate(phi + k*step, old_d, old_e) for k in (-1, 0, 1)]
            valid = [(r, phi + k*step) for k, r in zip((-1, 0, 1), options) if r]
            r, phi = min(valid, key=lambda item: item[0][0])
            best = (*r, phi)
            step *= 0.5

        _, d, e, _ = best
        self.D[:], self.E[:] = d, e
        return True

    def residual(self):
        pairs = [(self.A, self.C, self.L_AC), (self.C, self.D, self.L_CD),
                 (self.D, self.E, self.L_DE), (self.E, self.B, self.L_EB)]
        return max(abs(math.dist(p, q) - length) for p, q, length in pairs)

    def slider_changed(self, value):
        self.angle = float(value)
        old_c = self.C[:]
        self.set_driven_point()
        self.valid = self.solve(160)
        if not self.valid:
            # A rigid mechanism cannot enter this pose. Keep the last valid pose.
            self.C[:] = old_c
        self.draw()

    def toggle(self):
        self.running = not self.running

    def reset(self):
        self.running = False
        self.angle = 305.0
        self.valid = True
        self.D[:] = [430.0, 180.0]
        self.E[:] = [620.0, 270.0]
        self.slider.set(self.angle)
        self.set_driven_point()
        self.solve(400)
        self.draw()

    def tick(self):
        if self.running:
            self.angle = (self.angle + 0.35) % 360.0
            self.slider.set(self.angle)  # callback solves and redraws
        self.root.after(16, self.tick)

    def draw(self):
        c = self.canvas
        c.delete("all")
        c.create_text(20, 18, anchor="nw",
                      text="A and B fixed • AC driven • D and E selected from previous pose",
                      font=("Arial", 13), fill="#333")

        points = [self.A, self.C, self.D, self.E, self.B, self.A]
        colors = ["#e45756", "#4c78a8", "#59a14f", "#f28e2b", "#777"]
        for i in range(5):
            p, q = points[i], points[i + 1]
            c.create_line(p[0], p[1], q[0], q[1], width=9,
                          fill=colors[i], capstyle="round")

        for name, p in zip("ACDEB", [self.A, self.C, self.D, self.E, self.B]):
            fixed = name in "AB"
            r = 11 if fixed else 9
            c.create_oval(p[0]-r, p[1]-r, p[0]+r, p[1]+r,
                          fill="#222" if fixed else "white", outline="#111", width=3)
            c.create_text(p[0], p[1]-25, text=name, font=("Arial", 12, "bold"))

        if self.valid:
            self.info.config(text=f"OK   |   max length error = {self.residual():.12f} px",
                             fg="#187a2f")
        else:
            self.info.config(text="ERROR: для этого угла решения нет",
                             fg="#c62828")
            c.create_text(W/2, 70, text="ERROR: МЕХАНИЗМ НЕ МОЖЕТ ЗАМКНУТЬСЯ",
                          fill="#c62828", font=("Arial", 16, "bold"))


if __name__ == "__main__":
    app_root = tk.Tk()
    FiveBarDemo(app_root)
    app_root.mainloop()
