"""
ΒΗΜΑ 4: Χαρτογράφηση κατά την πτήση (με σταθερά σημεία).

    py step4_map.py

Το drone:
  1. απογειώνεται και κάνει σάρωση 360° ΚΡΑΤΩΝΤΑΣ τη θέση του,
  2. πηγαίνει στα σημεία του WAYPOINTS (σχετικά με την αφετηρία), σχεδιάζοντας τη διαδρομή
     ΠΑΝΩ ΣΤΟΝ ΧΑΡΤΗ (παρακάμπτει εμπόδια που έχει ήδη δει) και γυρίζοντας αργά για σάρωση,
  3. προσγειώνεται και αποθηκεύει:
       results/map.png, results/map.npz, results/flight_log.csv (για το make_map.py)

Αποφυγή εμποδίων: Multi-ranger (4 ακτίνες) + εμπόδια του χάρτη (όλες οι κατευθύνσεις).
Αν ένα σημείο πέφτει πάνω σε εμπόδιο -> πάει στο πλησιέστερο ασφαλές.
Ctrl+C -> προσγείωση.
"""
import os

from common import Flight, SafetyStop, Sensors, check_decks, connect, reset_estimator
from mapping import GridMap
from mission import Mission, save_log

WAYPOINTS = [(0.8, 0.0), (0.8, 0.8), (0.0, 0.8), (0.0, 0.0)]  # σχετικά με την αφετηρία
HEIGHT = 0.4
V_MAX = 0.25
YAW_RATE = 20.0      # μοίρες/s (πιο αργά = πιο καθαρός χάρτης)
OUT = "results"

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

    grid = GridMap(center=(d0["x"], d0["y"]), size=14.0)
    flight = Flight(cf, sensors, height=HEIGHT, fence=4.0)
    mission = None
    try:
        flight.takeoff()
        mission = Mission(flight, grid, v_max=V_MAX, yaw_rate=YAW_RATE, safety=0.25)
        x0, y0 = flight.home
        mission.log("Σάρωση 360° (κρατώντας θέση)")
        mission.hold(360.0 / YAW_RATE, YAW_RATE)
        for i, (wx, wy) in enumerate(WAYPOINTS, 1):
            mission.log(f"Σημείο {i}/{len(WAYPOINTS)}: ({wx:+.1f}, {wy:+.1f})")
            status = mission.go_to((x0 + wx, y0 + wy), yaw_rate=YAW_RATE)
            mission.log({"ok": "   -> ΕΦΤΑΣΕ", "stuck": "   -> εμπόδιο, το παραλείπω",
                         "timeout": "   -> timeout", "nopath": "   -> δεν υπάρχει διαδρομή",
                         "aborted": "   -> ακυρώθηκε"}[status])
    except SafetyStop as e:
        print("!!! ΑΣΦΑΛΕΙΑ:", e)
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        flight.land()
        sensors.stop()

    if mission is not None:
        grid.save(os.path.join(OUT, "map.npz"), mission.traj)
        save_log(mission.rows, os.path.join(OUT, "flight_log.csv"))
        out = grid.plot(os.path.join(OUT, "map.png"), mission.traj, title="Βήμα 4: χάρτης")
        print(f"\nΑποθηκεύτηκαν: {out}, {OUT}/map.npz, {OUT}/flight_log.csv")
print("Τέλος")
