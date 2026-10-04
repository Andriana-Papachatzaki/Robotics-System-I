"""
Η αποστολή: αυτόνομη εξερεύνηση -> επιστροφή -> (πλοήγηση σε σημεία) -> προσγείωση.
Δεν ξέρει αν "μιλάει" με πραγματικό drone ή με προσομοίωση: παίρνει
  flight : αντικείμενο με .send(vx, vy, yawrate) -> dict μετρήσεων, και .home
  clock  : αντικείμενο με .now() και .sleep(dt)
"""
import math
import time

from explore_core import Explorer, PathFollower, avoid_map
from common import avoid


class RealClock:
    now = staticmethod(time.time)
    sleep = staticmethod(time.sleep)


class Mission:
    def __init__(self, flight, grid, clock=RealClock(), v_max=0.3, yaw_rate=20.0,
                 safety=0.20, dt=0.05, verbose=True):
        self.flight, self.grid, self.clock = flight, grid, clock
        self.v_max, self.yaw_rate, self.safety, self.dt = v_max, yaw_rate, safety, dt
        self.explorer = Explorer(grid)
        self.follower = PathFollower(v_max=v_max)
        self.traj = []
        self.rows = []               # όλες οι μετρήσεις (για flight_log.csv / make_map.py)
        self.targets = []
        self.verbose = verbose
        self._last_map = -1.0
        self._last_seq = None
        self.t0 = clock.now()

    def log(self, *a):
        if self.verbose:
            print(f"[{self.clock.now() - self.t0:6.1f}s]", *a)

    # ---------------------------------------------------------------- ένα βήμα ελέγχου
    def step(self, vx, vy, yaw_rate):
        """Αποφυγή εμποδίων + αποστολή εντολής + ενημέρωση χάρτη (10 Hz)."""
        d = self.flight.s.get()
        # 1) αποφυγή με τις τρέχουσες μετρήσεις (4 ακτίνες Multi-ranger)
        vx, vy, blocked = avoid(vx, vy, d["yaw"], d, safety=self.safety, vmax=self.v_max)
        # 2) αποφυγή με τα εμπόδια του χάρτη (όλες οι κατευθύνσεις, και διαγώνια)
        vx, vy, blocked_map = avoid_map(vx, vy, d["x"], d["y"], self.grid,
                                        safety=self.safety + 0.05, vmax=self.v_max)
        blocked = blocked or blocked_map
        d = self.flight.send(vx, vy, yaw_rate)
        now = self.clock.now()
        seq = d.get("seq")
        new_sample = (seq != self._last_seq) if seq is not None else (now - self._last_map >= 0.1)
        # Χάρτης ΜΟΝΟ όταν το drone είναι στο ύψος πτήσης: κατά την απογείωση/βύθιση ο
        # Multi-ranger μπορεί να "δει" το πάτωμα -> ψεύτικα εμπόδια.
        target_z = getattr(self.flight, "height", None)
        at_height = target_z is None or abs(d.get("z", target_z) - target_z) < 0.10
        if new_sample and at_height:
            self._last_seq = seq
            self._last_map = now
            self.grid.integrate(d["x"], d["y"], d["yaw"],
                                {k: d.get(k) for k in ("front", "back", "left", "right")})
            self.grid.mark_visited(d["x"], d["y"])
            self.traj.append((d["x"], d["y"]))
            self.rows.append([round(now - self.t0, 2), d["x"], d["y"], d["z"], d["yaw"]] +
                             ["" if d.get(k) is None else d[k] for k in ("front", "back", "left", "right")])
        self.clock.sleep(self.dt)
        return d, blocked

    def hold(self, seconds, yaw_rate=0.0):
        """Μένει στη θέση του (P-έλεγχος θέσης), προαιρετικά περιστρέφεται."""
        d = self.flight.s.get()
        hx, hy = d["x"], d["y"]
        t_end = self.clock.now() + seconds
        while self.clock.now() < t_end:
            d = self.flight.s.get()
            self.step(1.0 * (hx - d["x"]), 1.0 * (hy - d["y"]), yaw_rate)

    # ------------------------------------------------------------------ εξερεύνηση
    def explore(self, max_time=240.0, replan=1.0, stuck_time=8.0, target_coverage=1.0, min_time=20.0):
        """
        target_coverage: σταματά όταν το drone έχει ΠΕΡΑΣΕΙ από αυτό το ποσοστό του γνωστού
        ελεύθερου χώρου (π.χ. 0.8 = 80%). min_time: ελάχιστος χρόνος πριν επιτραπεί η διακοπή.
        """
        self.log("Σάρωση 360° στην αφετηρία")
        self.hold(360.0 / self.yaw_rate, self.yaw_rate)
        t_start = self.clock.now()
        target, next_check = None, 0.0
        best_d, best_t = math.inf, 0.0
        while self.clock.now() - t_start < max_time:
            d = self.flight.s.get()
            x, y, now = d["x"], d["y"], self.clock.now()
            need = target is None or self.follower.done(x, y)
            if now >= next_check and now - t_start > min_time:
                cov = self.explorer.coverage()
                if cov >= target_coverage:
                    self.log(f"Έφτασε τον στόχο κάλυψης: {100 * cov:.0f}% (στόχος {100 * target_coverage:.0f}%)")
                    return True
            if not need and now >= next_check:
                next_check = now + replan
                if not self.explorer.target_valid(target) or \
                        self.explorer.path_blocked(x, y, self.follower.remaining()):
                    need = True
                dist = math.hypot(target[0] - x, target[1] - y)
                if dist < best_d - 0.05:
                    best_d, best_t = dist, now
                elif now - best_t > stuck_time:
                    self.log(f"Δεν μπορώ να φτάσω τον στόχο ({target[0]:.2f}, {target[1]:.2f}) -> τον αφήνω")
                    self.explorer.blacklist.append(target)
                    need = True
            if need:
                heading = None
                if len(self.traj) > 5:
                    px, py = self.traj[-5]
                    if math.hypot(x - px, y - py) > 0.05:
                        heading = math.atan2(y - py, x - px)
                target, path = self.explorer.select_target(x, y, heading)
                if target is None:
                    self.log(f"Η εξερεύνηση ολοκληρώθηκε (κάλυψη ~{100 * self.explorer.coverage():.0f}%)")
                    return True
                self.follower.set(path)
                self.targets.append(target)
                best_d, best_t, next_check = math.inf, now, now + replan
                if len(self.targets) % 5 == 1:
                    self.log(f"Στόχος #{len(self.targets)} ({target[0]:.2f}, {target[1]:.2f}) | "
                             f"κάλυψη ~{100 * self.explorer.coverage():.0f}%")
            vx, vy = self.follower.command(x, y)
            self.step(vx, vy, self.yaw_rate)
        self.log("Τέλος χρόνου εξερεύνησης")
        return False

    # ------------------------------------------------------------------- πλοήγηση
    def go_to(self, goal, timeout=45.0, yaw_rate=0.0, abort=None, stuck_time=6.0):
        """
        Πλοήγηση σε σημείο. Επιστρέφει:
          "ok"       έφτασε
          "stuck"    δεν προχωράει (π.χ. ο στόχος είναι πίσω από εμπόδιο) -> τα παράτησε
          "aborted"  η συνάρτηση abort() επέστρεψε True (π.χ. δόθηκε νέος στόχος)
          "timeout" / "nopath"
        """
        d = self.flight.s.get()
        path = self.explorer.plan_to(d["x"], d["y"], goal)
        if path is None:
            self.log(f"Δεν βρέθηκε διαδρομή προς ({goal[0]:.2f}, {goal[1]:.2f})")
            return "nopath"
        self.follower.set(path)
        t0 = self.clock.now()
        next_check = t0 + 1.0
        best_d, best_t = math.inf, t0
        blocked_since = None
        while True:
            d = self.flight.s.get()
            x, y, now = d["x"], d["y"], self.clock.now()
            if self.follower.done(x, y):
                self.hold(0.5)
                return "ok"
            if abort is not None and abort():
                return "aborted"
            if now - t0 > timeout:
                self.log("Timeout")
                return "timeout"
            # πρόοδος: μήκος διαδρομής που απομένει (όχι ευθεία απόσταση, λόγω παρακάμψεων)
            dist = self.follower.remaining_length(x, y)
            if dist < best_d - 0.05:
                best_d, best_t = dist, now
            elif now - best_t > stuck_time:
                self.log("Δεν προχωράει προς τον στόχο -> σταματάω")
                self.hold(0.5)
                return "stuck"
            if now >= next_check:
                next_check = now + 1.0
                if self.explorer.path_blocked(x, y, self.follower.remaining()):
                    path = self.explorer.plan_to(x, y, self.follower.path[-1])
                    if path is None:
                        return "nopath"
                    self.follower.set(path)
                    best_d, best_t = math.inf, now
            vx, vy = self.follower.command(x, y)
            d, blocked = self.step(vx, vy, yaw_rate)
            if blocked:
                blocked_since = blocked_since or now
                if now - blocked_since > 4.0:
                    self.log("Εμπόδιο στη διαδρομή -> σταματάω")
                    self.hold(0.5)
                    return "stuck"
            else:
                blocked_since = None


def save_log(rows, path):
    import csv
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "yaw", "front", "back", "left", "right"])
        w.writerows(rows)
