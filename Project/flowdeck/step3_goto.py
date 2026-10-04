"""
ΒΗΜΑ 3: Κίνηση σε σημεία (waypoints) με αποφυγή εμποδίων.

    py step3_goto.py

Τα σημεία είναι ΣΧΕΤΙΚΑ με την αφετηρία (σε μέτρα, στους άξονες του Lighthouse).
Προεπιλογή: 1 m στο +x, πίσω, 1 m στο +y, πίσω. Άλλαξέ τα στη λίστα WAYPOINTS.

Δοκιμή αποφυγής: στάσου (ή βάλε ένα κουτί) στη διαδρομή. Το drone πρέπει να
σταματήσει ~40 cm πριν, να περιμένει και μετά να παραλείψει αυτό το σημείο.

Ασφάλεια: Ctrl+C / χέρι πάνω από το drone / z > 1 m / > 3 m από την αφετηρία -> προσγείωση.
"""
import math
import time

from common import Flight, SafetyStop, Sensors, avoid, check_decks, connect, reset_estimator

WAYPOINTS = [(1.0, 0.0), (0.0, 0.0), (0.0, 1.0), (0.0, 0.0)]
HEIGHT = 0.4          # m
V_MAX = 0.3           # m/s
TOLERANCE = 0.10      # m, "έφτασε"
BLOCKED_TIMEOUT = 4.0 # s μπλοκαρισμένο από εμπόδιο -> παράλειψη σημείου
POINT_TIMEOUT = 20.0  # s ανά σημείο
DT = 0.05             # s (20 Hz)


def go_to(flight, gx, gy):
    """Πηγαίνει στο (gx, gy). Επιστρέφει 'ok', 'blocked' ή 'timeout'."""
    t0 = time.time()
    blocked_since = None
    last_print = 0.0
    while True:
        d = flight.s.get()
        dx, dy = gx - d["x"], gy - d["y"]
        dist = math.hypot(dx, dy)
        if dist < TOLERANCE:
            flight.send(0.0, 0.0)
            return "ok"
        speed = min(V_MAX, 1.0 * dist)            # επιβράδυνση κοντά στον στόχο
        vx, vy = speed * dx / dist, speed * dy / dist
        vx, vy, blocked = avoid(vx, vy, d["yaw"], d, vmax=V_MAX)
        d = flight.send(vx, vy)

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


def hold(flight, seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        flight.send(0.0, 0.0)
        time.sleep(DT)


with connect() as scf:
    cf = scf.cf
    print("Σύνδεση OK")
    check_decks(cf)
    reset_estimator(cf, scf)
    sensors = Sensors(cf)
    sensors.wait_ready()
    print("Αφετηρία:", Sensors.fmt(sensors.get()))

    flight = Flight(cf, sensors, height=HEIGHT)
    try:
        flight.takeoff()
        x0, y0 = flight.home
        hold(flight, 1.0)
        for i, (wx, wy) in enumerate(WAYPOINTS, 1):
            gx, gy = x0 + wx, y0 + wy
            print(f"\nΣημείο {i}/{len(WAYPOINTS)}: ({wx:+.1f}, {wy:+.1f}) από την αφετηρία")
            result = go_to(flight, gx, gy)
            print({"ok": "   -> ΕΦΤΑΣΕ",
                   "blocked": "   -> ΕΜΠΟΔΙΟ, παραλείπω το σημείο",
                   "timeout": "   -> δεν έφτασε εγκαίρως, συνεχίζω"}[result])
            hold(flight, 1.0)
    except SafetyStop as e:
        print("!!! ΑΣΦΑΛΕΙΑ:", e)
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        flight.land()
        sensors.stop()
print("Τέλος")
