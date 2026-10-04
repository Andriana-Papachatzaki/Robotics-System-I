"""
Ο "εγκέφαλος" της εξερεύνησης (χωρίς καμία εξάρτηση από το drone -> δοκιμάζεται και σε προσομοίωση).

1. Ζώνες ασφαλείας γύρω από τα εμπόδια του χάρτη:
     lethal    (≤ LETHAL m από εμπόδιο)      -> απαγορευμένα κελιά
     inflated  (≤ INFLATION m)               -> επιτρεπτά αλλά ακριβά (κρατά απόσταση)
2. Dijkstra από τη θέση του drone προς ΟΛΑ τα κελιά (κόστος διαδρομής + προσβασιμότητα).
3. Υποψήφιοι στόχοι: ελεύθερα κελιά που ΔΕΝ έχει επισκεφθεί το drone + frontiers
   (άγνωστα κελιά δίπλα σε ελεύθερα). Βαθμολογία:
       score = απόσταση_διαδρομής - GAIN_WEIGHT * gain + HEADING_WEIGHT * (ποινή στροφής)
   gain = ποσοστό ανεπίσκεπτων κελιών γύρω από τον στόχο.
4. A* για διαδρομή προς συγκεκριμένο σημείο (επιστροφή / πλοήγηση).
5. Εξομάλυνση διαδρομής (line of sight) και pure-pursuit ακολουθία.
"""
import heapq
import math

import numpy as np

from mapping import box_sum, dilate

# ------------------------------------------------------------------- παράμετροι
LETHAL = 0.25          # m  (>= απόσταση αποφυγής του Multi-ranger + μισό drone)
INFLATION = 0.40       # m
TARGET_CLEARANCE = 0.27
INFLATION_COST = 6.0
UNKNOWN_COST = 3.0
UNKNOWN_HALO = 0.5     # m: άγνωστα κελιά πιο μακριά από γνωστό ελεύθερο χώρο -> μη διαβατά
GAIN_RADIUS = 0.35
GAIN_WEIGHT = 1.0
HEADING_WEIGHT = 0.3
MIN_GAIN_CELLS = 6
BLACKLIST_RADIUS = 0.25

SQ2 = math.sqrt(2.0)
_NEI = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]


# ---------------------------------------------------------------- χάρτης κόστους
def masks(grid):
    """Υπολογίζει (μία φορά ανά ενημέρωση του χάρτη) τις ζώνες ασφαλείας."""
    cached = getattr(grid, "_masks", None)
    if cached is not None and cached[0] == grid.version:
        return cached[1]
    occ = grid.occupied()
    r = lambda m: int(math.ceil(m / grid.res))  # noqa: E731

    def border(m):
        m[0, :] = m[-1, :] = m[:, 0] = m[:, -1] = True
        return m
    M = dict(occ=occ,
             free=grid.free(),
             lethal=border(dilate(occ, r(LETHAL))),
             inflated=border(dilate(occ, r(INFLATION))),
             forbidden=border(dilate(occ, r(TARGET_CLEARANCE))))
    grid._masks = (grid.version, M)
    return M


def cost_grid(grid, M, escape_from=None, unknown_cost=UNKNOWN_COST):
    cost = np.ones((grid.n, grid.n))
    unknown = ~(M["occ"] | M["free"])
    cost[unknown] = unknown_cost
    halo = dilate(M["free"], int(math.ceil(UNKNOWN_HALO / grid.res)))
    cost[unknown & ~halo] = np.inf
    cost[M["inflated"]] += INFLATION_COST
    cost[M["lethal"]] = np.inf
    if escape_from is not None:
        # αν το drone βρεθεί μέσα σε απαγορευμένη ζώνη, επιτρέπουμε (ακριβά) να βγει
        ci, cj = grid.world_to_grid(*escape_from)
        k = int(math.ceil(0.5 / grid.res))
        i0, i1, j0, j1 = max(1, ci - k), min(grid.n - 1, ci + k + 1), max(1, cj - k), min(grid.n - 1, cj + k + 1)
        sub = cost[i0:i1, j0:j1]
        sub[np.isinf(sub) & ~M["occ"][i0:i1, j0:j1]] = 50.0
    return cost


# ------------------------------------------------------------------- αλγόριθμοι
def dijkstra(cost, start, res):
    nx, n = cost.shape
    c = cost.ravel().tolist()
    dist = [math.inf] * cost.size
    parent = [-1] * cost.size
    s = start[0] * n + start[1]
    dist[s] = 0.0
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        ui, uj = divmod(u, n)
        for di, dj, step in _NEI:
            vi, vj = ui + di, uj + dj
            if not (0 <= vi < nx and 0 <= vj < n):
                continue
            v = vi * n + vj
            if c[v] == math.inf:
                continue
            if step > 1 and (c[ui * n + vj] == math.inf or c[vi * n + uj] == math.inf):
                continue
            nd = d + step * c[v] * res
            if nd < dist[v]:
                dist[v], parent[v] = nd, u
                heapq.heappush(pq, (nd, v))
    return np.array(dist).reshape(cost.shape), np.array(parent)


def extract_path(parent, n, goal):
    g, out = goal[0] * n + goal[1], []
    while g != -1:
        out.append(divmod(int(g), n))
        g = parent[g]
    return out[::-1]


def astar(cost, start, goal, res):
    nx, n = cost.shape
    c = cost.ravel().tolist()
    s, gidx = start[0] * n + start[1], goal[0] * n + goal[1]
    if c[gidx] == math.inf:
        return None
    gi, gj = goal

    def h(i, j):
        dx, dy = abs(i - gi), abs(j - gj)
        return (max(dx, dy) + (SQ2 - 1) * min(dx, dy)) * res
    g, parent, closed = {s: 0.0}, {s: -1}, set()
    pq = [(h(*start), 0.0, s)]
    while pq:
        _, d, u = heapq.heappop(pq)
        if u in closed:
            continue
        if u == gidx:
            out = []
            while u != -1:
                out.append(divmod(u, n))
                u = parent[u]
            return out[::-1]
        closed.add(u)
        ui, uj = divmod(u, n)
        for di, dj, step in _NEI:
            vi, vj = ui + di, uj + dj
            if not (0 <= vi < nx and 0 <= vj < n):
                continue
            v = vi * n + vj
            if c[v] == math.inf or v in closed:
                continue
            if step > 1 and (c[ui * n + vj] == math.inf or c[vi * n + uj] == math.inf):
                continue
            nd = d + step * c[v] * res
            if nd < g.get(v, math.inf):
                g[v], parent[v] = nd, u
                heapq.heappush(pq, (nd + h(vi, vj), nd, v))
    return None


def line_of_sight(cost, a, b, max_cost):
    k = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    t = np.linspace(0, 1, k + 1)
    ii = np.rint(a[0] + t * (b[0] - a[0])).astype(int)
    jj = np.rint(a[1] + t * (b[1] - a[1])).astype(int)
    return bool(np.all(cost[ii, jj] <= max_cost))


def smooth_path(cells, cost):
    """Συντόμευση: ευθείες γραμμές όπου δεν πλησιάζουμε εμπόδια περισσότερο από την αρχική διαδρομή."""
    if len(cells) <= 2:
        return list(cells)
    out, i = [cells[0]], 0
    while i < len(cells) - 1:
        best = i + 1
        seg_max = max(cost[cells[i]], cost[cells[i + 1]])
        for j in range(i + 2, len(cells)):
            seg_max = max(seg_max, cost[cells[j]])
            if line_of_sight(cost, cells[i], cells[j], seg_max):
                best = j
            elif j - best > 6:
                break
        out.append(cells[best])
        i = best
    return out


# ------------------------------------------------------------------ εξερεύνηση
class Explorer:
    def __init__(self, grid):
        self.grid = grid
        self.blacklist = []

    def select_target(self, x, y, heading=None):
        """(στόχος, διαδρομή) ή (None, None) όταν δεν μένει τίποτα να εξερευνηθεί."""
        g = self.grid
        M = masks(g)
        cost = cost_grid(g, M, escape_from=(x, y))
        start = g.world_to_grid(x, y)
        dist, parent = dijkstra(cost, start, g.res)

        unknown = ~(M["occ"] | M["free"])
        frontier = unknown & dilate(M["free"], 1)
        cand = ((M["free"] & ~g.visited) | frontier) & ~M["forbidden"] & np.isfinite(dist)
        for bx, by in self.blacklist:
            i, j = g.world_to_grid(bx, by)
            k = int(math.ceil(BLACKLIST_RADIUS / g.res))
            cand[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1] = False
        half = int(math.ceil(GAIN_RADIUS / g.res))
        gain_cells = box_sum(~g.visited & ~M["occ"] & ~M["lethal"], half)
        cand &= gain_cells >= MIN_GAIN_CELLS
        if not cand.any():
            return None, None
        score = dist - GAIN_WEIGHT * gain_cells / (2 * half + 1) ** 2
        if heading is not None:
            ii, jj = np.nonzero(cand)
            wx = g.origin[0] + (ii + 0.5) * g.res - x
            wy = g.origin[1] + (jj + 0.5) * g.res - y
            cosang = (wx * math.cos(heading) + wy * math.sin(heading)) / np.maximum(np.hypot(wx, wy), 1e-6)
            score[ii, jj] += HEADING_WEIGHT * 0.5 * (1 - cosang)
        score = np.where(cand, score, np.inf)
        goal = divmod(int(np.argmin(score)), g.n)
        cells = smooth_path(extract_path(parent, g.n, goal), cost)
        path = [g.grid_to_world(i, j) for i, j in cells]
        path[0] = (x, y)
        return g.grid_to_world(*goal), path

    def target_valid(self, target):
        g = self.grid
        i, j = g.world_to_grid(*target)
        return g.in_bounds(i, j) and not g.visited[i, j] and not masks(g)["forbidden"][i, j]

    def plan_to(self, x, y, goal, unknown_cost=UNKNOWN_COST * 2):
        """
        Διαδρομή προς σημείο. Αν ο στόχος δεν είναι ασφαλής (πάνω/μέσα/πολύ κοντά σε εμπόδιο,
        ή σε άγνωστη περιοχή που δεν είναι προσβάσιμη), πηγαίνει στο ΠΛΗΣΙΕΣΤΕΡΟ ΠΡΟΣΒΑΣΙΜΟ
        ασφαλές σημείο. Χρησιμοποιεί Dijkstra (δίνει ταυτόχρονα ποια σημεία είναι προσβάσιμα).
        """
        g = self.grid
        M = masks(g)
        cost = cost_grid(g, M, escape_from=(x, y), unknown_cost=unknown_cost)
        start = g.world_to_grid(x, y)
        gi, gj = g.world_to_grid(*goal)
        dist, parent = dijkstra(cost, start, g.res)
        safe = np.isfinite(dist) & M["free"] & ~M["inflated"]
        exact = g.in_bounds(gi, gj) and safe[gi, gj]
        if not exact:
            ii, jj = np.nonzero(safe)
            if len(ii) == 0:
                return None
            k = int(np.argmin((ii - gi) ** 2 + (jj - gj) ** 2))
            gi, gj = int(ii[k]), int(jj[k])
            sx, sy = g.grid_to_world(gi, gj)
            print(f"   (ο στόχος δεν είναι ασφαλής/προσβάσιμος -> πάω στο πλησιέστερο ασφαλές "
                  f"σημείο {sx:.2f}, {sy:.2f})")
        cells = extract_path(parent, g.n, (gi, gj))
        path = [g.grid_to_world(i, j) for i, j in smooth_path(cells, cost)]
        path[0] = (x, y)
        if exact:
            path[-1] = tuple(goal)
        return path

    def path_blocked(self, x, y, path):
        g = self.grid
        lethal = masks(g)["lethal"]
        ci, cj = g.world_to_grid(x, y)
        pts = [(x, y)] + list(path)
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            k = max(2, int(math.hypot(x1 - x0, y1 - y0) / (g.res * 0.5)))
            for t in np.linspace(0, 1, k):
                i, j = g.world_to_grid(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
                if not g.in_bounds(i, j):
                    return True
                if lethal[i, j] and abs(i - ci) + abs(j - cj) > 4:
                    return True
        return False

    def coverage(self):
        M = masks(self.grid)
        reach = M["free"] & ~M["lethal"]
        return float((reach & self.grid.visited).sum()) / max(1, int(reach.sum()))


# --------------------------------------------------------------- ακολουθία διαδρομής
class PathFollower:
    def __init__(self, v_max=0.3, lookahead=0.3, tol=0.10):
        self.v_max, self.lookahead, self.tol = v_max, lookahead, tol
        self.path, self.idx = [], 0

    def set(self, path):
        self.path = list(path or [])
        self.idx = 1 if len(self.path) > 1 else 0

    def remaining(self):
        return self.path[self.idx:]

    def remaining_length(self, x, y):
        """Μήκος διαδρομής που απομένει (μέτρο της προόδου, λαμβάνει υπόψη τις παρακάμψεις)."""
        pts = [(x, y)] + self.path[self.idx:]
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))

    def done(self, x, y):
        return not self.path or math.hypot(self.path[-1][0] - x, self.path[-1][1] - y) < self.tol

    def command(self, x, y):
        if not self.path:
            return 0.0, 0.0
        last = len(self.path) - 1
        while self.idx < last and math.hypot(self.path[self.idx][0] - x,
                                             self.path[self.idx][1] - y) < self.lookahead:
            self.idx += 1
        tx, ty = self.path[self.idx]
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return 0.0, 0.0
        speed = min(self.v_max, 1.2 * d) if self.idx == last else self.v_max
        return speed * dx / d, speed * dy / d