"""
ΒΗΜΑ 4: Χαρτογράφηση κατά την πτήση.

    py step4_map.py

Το drone πετάει ένα τετράγωνο γύρω από την αφετηρία (ρυθμίζεται στο WAYPOINTS),
περιστρέφεται αργά γύρω από τον εαυτό του (ώστε οι 4 αισθητήρες να "σαρώνουν" όλες
τις κατευθύνσεις) και χτίζει χάρτη εμποδίων. Στο τέλος αποθηκεύει:
    results/map.png        εικόνα του χάρτη με την τροχιά
    results/map.npz        ο χάρτης (για τα επόμενα βήματα)
    results/flight_log.csv όλες οι μετρήσεις (για να ξαναφτιάξεις τον χάρτη: py make_map.py)

Αποφυγή εμποδίων και ασφάλεια όπως στο βήμα 3 (Ctrl+C -> προσγείωση).
"""
import csv
import math
import os
import time

from common import Flight, SafetyStop, Sensors, avoid, check_decks, connect, reset_estimator
from mapping import GridMap

WAYPOINTS = [(0.8, 0.0), (0.8, 0.8), (0.0, 0.8), (0.0, 0.0)]  # σχετικά με την αφετηρία
HEIGHT = 0.4
V_MAX = 0.25
YAW_RATE = 30.0       # μοίρες/s, αργή περιστροφή για πυκνότερη σάρωση
TOLERANCE = 0.10
BLOCKED_TIMEOUT = 4.0
POINT_TIMEOUT = 25.0
DT = 0.05
OUT = "results"


class Recorder:
    """Ενημερώνει τον χάρτη και καταγράφει τις μετρήσεις (10 Hz)."""

    def __init__(self, grid, path):
        self.grid = grid
        self.traj = []
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "z", "yaw", "front", "back", "left", "right"])
        self.t0 = time.time()
        self.last = 0.0

    def update(self, d):
        now = time.time()
        if now - self.last < 0.1:
            return
        self.last = now
        rng = {k: d.get(k) for k in ("front", "back", "left", "right")}
        self.grid.integrate(d["x"], d["y"], d["yaw"], rng)
        self.grid.mark_visited(d["x"], d["y"])
        self.traj.append((d["x"], d["y"]))
        self.w.writerow([f"{now - self.t0:.2f}", d["x"], d["y"], d["z"], d["yaw"]] +
                        ["" if rng[k] is None else rng[k] for k in ("front", "back", "left", "right")])

    def close(self):
        self.f.close()


def go_to(flight, rec, gx, gy):
    t0 = time.time()
    blocked_since = None
    last_print = 0.0
    while True:
        d = flight.s.get()
        rec.update(d)
        dx, dy = gx - d["x"], gy - d["y"]
        dist = math.hypot(dx, dy)
        if dist < TOLERANCE:
            return "ok"
        speed = min(V_MAX, 1.0 * dist)
        vx, vy = speed * dx / dist, speed * dy / dist
        vx, vy, blocked = avoid(vx, vy, d["yaw"], d, vmax=V_MAX)
        flight.send(vx, vy, YAW_RATE)
        now = time.time()
        if blocked:
            blocked_since = blocked_since or now
            if now - blocked_since > BLOCKED_TIMEOUT:
                return "blocked"
        else:
            blocked_since = None
        if now - t0 > POINT_TIMEOUT:
            return "timeout"
        if now - last_print > 0.5:
            last_print = now
            print(f"   απόσταση {dist:.2f} m {'[ΕΜΠΟΔΙΟ]' if blocked else ''} | {Sensors.fmt(d)}")
        time.sleep(DT)


def spin(flight, rec, seconds):
    """Περιστροφή επί τόπου (σάρωση)."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        rec.update(flight.s.get())
        flight.send(0.0, 0.0, YAW_RATE)
        time.sleep(DT)


os.makedirs(OUT, exist_ok=True)
with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    d0 = sensors.get()
    print("Αφετηρία:", Sensors.fmt(d0))

    grid = GridMap(center=(d0["x"], d0["y"]))
    rec = Recorder(grid, os.path.join(OUT, "flight_log.csv"))
    flight = Flight(cf, sensors, height=HEIGHT)
    try:
        flight.takeoff()
        x0, y0 = flight.home
        print("Σάρωση στην αφετηρία (360°) ...")
        spin(flight, rec, 360.0 / YAW_RATE)
        for i, (wx, wy) in enumerate(WAYPOINTS, 1):
            print(f"\nΣημείο {i}/{len(WAYPOINTS)}: ({wx:+.1f}, {wy:+.1f})")
            result = go_to(flight, rec, x0 + wx, y0 + wy)
            print({"ok": "   -> ΕΦΤΑΣΕ", "blocked": "   -> ΕΜΠΟΔΙΟ, παραλείπω",
                   "timeout": "   -> timeout, συνεχίζω"}[result])
    except SafetyStop as e:
        print("!!! ΑΣΦΑΛΕΙΑ:", e)
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        flight.land()
        sensors.stop()
        rec.close()

    grid.save(os.path.join(OUT, "map.npz"), rec.traj)
    grid.plot(os.path.join(OUT, "map.png"), rec.traj, title="Βήμα 4: χάρτης")
    print(f"\nΑποθηκεύτηκαν: {OUT}/map.png, {OUT}/map.npz, {OUT}/flight_log.csv")
print("Τέλος")
