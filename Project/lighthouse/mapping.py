"""
Χάρτης πλέγματος (occupancy grid) + χάρτης επισκεψιμότητας (visited grid).

Κάθε κελί (5x5 cm) κρατά μια τιμή log-odds:
  > +0.6  -> εμπόδιο      (μαύρο στην εικόνα)
  < -0.3  -> ελεύθερο     (λευκό)
  αλλιώς  -> άγνωστο      (γκρι)

Για κάθε μέτρηση του Multi-ranger (μπροστά/πίσω/αριστερά/δεξιά):
  - τα κελιά ΚΑΤΑ ΜΗΚΟΣ της ακτίνας γίνονται πιο "ελεύθερα",
  - το κελί στο ΤΕΛΟΣ της ακτίνας γίνεται πιο "κατειλημμένο" (αν υπήρξε εμπόδιο).
Επειδή ο αισθητήρας δίνει την κοντινότερη επιφάνεια μέσα σε έναν κώνο (~27°),
σημειώνουμε ως ελεύθερο και έναν στενό κώνο ±6° (3 ακτίνες).

Το visited grid σημειώνει τα κελιά από τα οποία πέρασε ΦΥΣΙΚΑ το drone
(σε ακτίνα 30 cm) – αυτό μετράει στην αξιολόγηση της εξερεύνησης.
"""
import math

import numpy as np

SENSOR_DIRS = {"front": 0.0, "left": 90.0, "back": 180.0, "right": -90.0}  # μοίρες, σώμα


class GridMap:
    def __init__(self, center=(0.0, 0.0), size=10.0, res=0.05,
                 max_range=3.0, no_hit_range=1.5, visit_radius=0.30,
                 sensor_offset=0.03, cone_deg=9.0, cone_rays=5,
                 l_occ=0.9, l_free=-0.4, l_min=-4.0, l_max=4.0,
                 occ_thr=0.6, free_thr=-0.3):
        self.res = res
        self.n = int(round(size / res))
        self.origin = np.array([center[0] - size / 2, center[1] - size / 2])
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.visited = np.zeros((self.n, self.n), bool)
        self.version = 0          # αυξάνεται σε κάθε ενημέρωση (για cache)
        self.max_range, self.no_hit_range = max_range, no_hit_range
        self.sensor_offset = sensor_offset
        self.cone = np.radians(np.linspace(-cone_deg, cone_deg, cone_rays))
        self.l_occ, self.l_free, self.l_min, self.l_max = l_occ, l_free, l_min, l_max
        self.occ_thr, self.free_thr = occ_thr, free_thr
        r = int(math.ceil(visit_radius / res))
        di, dj = np.mgrid[-r:r + 1, -r:r + 1]
        m = di * di + dj * dj <= r * r
        self._vdi, self._vdj = di[m], dj[m]

    # ------------------------------------------------------------- συντεταγμένες
    def world_to_grid(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.res)),
                int(math.floor((y - self.origin[1]) / self.res)))

    def grid_to_world(self, i, j):
        return (self.origin[0] + (i + 0.5) * self.res, self.origin[1] + (j + 0.5) * self.res)

    def in_bounds(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def _cells(self, x0, y0, x1, y1):
        L = math.hypot(x1 - x0, y1 - y0)
        k = max(2, int(L / (self.res * 0.5)) + 1)
        t = np.linspace(0.0, 1.0, k)
        i = np.floor((x0 + t * (x1 - x0) - self.origin[0]) / self.res).astype(int)
        j = np.floor((y0 + t * (y1 - y0) - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i[ok], j[ok]

    # ------------------------------------------------------------------ ενημέρωση
    def integrate(self, x, y, yaw_deg, ranges):
        """Μία μέτρηση: θέση (m), yaw (μοίρες), ranges = dict front/back/left/right σε m ή None."""
        free_i, free_j, occ = [], [], []
        for name, ang in SENSOR_DIRS.items():
            if name not in ranges:
                continue
            r = ranges[name]
            hit = r is not None and r < self.max_range
            if r is not None and r < 0.02:
                continue
            length = r if hit else self.no_hit_range
            a0 = math.radians(yaw_deg + ang)
            sx = x + self.sensor_offset * math.cos(a0)
            sy = y + self.sensor_offset * math.sin(a0)
            for da in self.cone:
                a = a0 + da
                L = length - (self.res if hit else 0.0) - (0.0 if da == 0 else self.res)
                if L <= 0:
                    continue
                ci, cj = self._cells(sx, sy, sx + L * math.cos(a), sy + L * math.sin(a))
                free_i.append(ci)
                free_j.append(cj)
            if hit:
                # Πριν: μόνο η κεντρική ακτίνα (da=0) σημείωνε "κατειλημμένο" κελί.
                # Ο αισθητήρας όμως έχει πλάτος δέσμης (~27 μοίρες) -- η επιφάνεια
                # που ανίχνευσε γεμίζει όλο αυτό το πλάτος, όχι μόνο ένα σημείο στο
                # κέντρο. Σημειώνουμε το τέλος ΚΑΘΕ ακτίνας του κώνου ως κατειλημμένο
                # (όχι μόνο του κέντρου), ώστε ο τοίχος/εμπόδιο να βγαίνει ως συνεχής
                # επιφάνεια στον χάρτη αντί για μεμονωμένα σημεία.
                for da in self.cone:
                    a = a0 + da
                    i, j = self.world_to_grid(sx + r * math.cos(a), sy + r * math.sin(a))
                    if self.in_bounds(i, j):
                        occ.append((i, j))
        if free_i:
            self.logodds[np.concatenate(free_i), np.concatenate(free_j)] += self.l_free
        for i, j in occ:
            self.logodds[i, j] += self.l_occ
        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)
        self.version += 1

    def mark_visited(self, x, y):
        ci, cj = self.world_to_grid(x, y)
        vi, vj = ci + self._vdi, cj + self._vdj
        ok = (vi >= 0) & (vi < self.n) & (vj >= 0) & (vj < self.n)
        self.visited[vi[ok], vj[ok]] = True

    # -------------------------------------------------------------------- μάσκες
    def occupied(self):
        return self.logodds > self.occ_thr

    def free(self):
        return self.logodds < self.free_thr

    def unknown(self):
        return ~(self.occupied() | self.free())

    # ------------------------------------------------------------ αποθήκευση/εικόνα
    def save(self, path, traj=None):
        np.savez_compressed(path, logodds=self.logodds, visited=self.visited,
                            origin=self.origin, res=self.res,
                            traj=np.array(traj if traj is not None else []))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        g = cls()
        g.logodds, g.visited = d["logodds"], d["visited"]
        g.origin, g.res = d["origin"], float(d["res"])
        g.n = g.logodds.shape[0]
        return g, d["traj"]

    def plot(self, path, traj=None, title="Χάρτης", extra_points=None):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        img = np.full(self.logodds.shape, 0.75)
        img[self.free()] = 1.0
        img[self.occupied()] = 0.0
        ext = [self.origin[0], self.origin[0] + self.n * self.res,
               self.origin[1], self.origin[1] + self.n * self.res]
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.imshow(img.T, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1,
                  interpolation="nearest")
        vis = np.ma.masked_where(~self.visited, np.ones_like(img))
        ax.imshow(vis.T, origin="lower", extent=ext, cmap="Greens", alpha=0.3,
                  vmin=0, vmax=1.5, interpolation="nearest")
        if traj is not None and len(traj):
            t = np.asarray(traj)
            ax.plot(t[:, 0], t[:, 1], color="tab:blue", lw=1.2, label="τροχιά")
            ax.plot(t[0, 0], t[0, 1], "*", color="gold", ms=15, mec="k", label="αφετηρία")
        if extra_points:
            p = np.asarray(extra_points)
            ax.scatter(p[:, 0], p[:, 1], marker="x", color="tab:red", label="στόχοι")
        known = ~self.unknown()
        if known.any():
            ii, jj = np.nonzero(known)
            ax.set_xlim(self.origin[0] + ii.min() * self.res - 0.5,
                        self.origin[0] + (ii.max() + 1) * self.res + 0.5)
            ax.set_ylim(self.origin[1] + jj.min() * self.res - 0.5,
                        self.origin[1] + (jj.max() + 1) * self.res + 0.5)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(title + "  (μαύρο=εμπόδιο, λευκό=ελεύθερο, γκρι=άγνωστο, πράσινο=επισκέφθηκε)",
                     fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        try:
            fig.savefig(path, dpi=130)
        except OSError:
            # π.χ. το αρχείο είναι ανοιχτό σε πρόγραμμα προβολής -> σώζουμε με άλλο όνομα
            import time as _t
            path = path.rsplit(".", 1)[0] + _t.strftime("_%H%M%S") + ".png"
            fig.savefig(path, dpi=130)
            print(f"  (το αρχικό αρχείο ήταν κλειδωμένο, αποθηκεύτηκε ως {path})")
        plt.close(fig)
        return path


# =====================================================================================
#  Βοηθητικά για τον σχεδιασμό (βήμα 5 και μετά)
# =====================================================================================
def dilate(mask, r):
    """Μορφολογική διαστολή με κυκλικό πυρήνα ακτίνας r κελιών."""
    if r <= 0:
        return mask.copy()
    nx, ny = mask.shape
    pad = np.pad(mask, r)
    out = np.zeros_like(mask)
    di, dj = np.mgrid[-r:r + 1, -r:r + 1]
    for a, b in zip(di[di * di + dj * dj <= r * r], dj[di * di + dj * dj <= r * r]):
        out |= pad[r + a:r + a + nx, r + b:r + b + ny]
    return out


def box_sum(arr, half):
    """Άθροισμα σε τετράγωνο παράθυρο (2*half+1)^2 (integral image)."""
    a = np.pad(arr.astype(np.float32), ((1, 0), (1, 0)))
    s = a.cumsum(0).cumsum(1)
    nx, ny = arr.shape
    i0 = np.clip(np.arange(nx) - half, 0, nx)
    i1 = np.clip(np.arange(nx) + half + 1, 0, nx)
    j0 = np.clip(np.arange(ny) - half, 0, ny)
    j1 = np.clip(np.arange(ny) + half + 1, 0, ny)
    return s[i1][:, j1] - s[i0][:, j1] - s[i1][:, j0] + s[i0][:, j0]